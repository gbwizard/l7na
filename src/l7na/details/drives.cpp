#include <sys/time.h>
#include <errno.h>

#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <thread>
#include <mutex>
#include <chrono>
#include <functional>
#include <algorithm>
#include <map>
#include <list>
#include <atomic>

#include <boost/filesystem/path.hpp>
#include <boost/memory_order.hpp>
#include <boost/thread/thread.hpp>
#include <boost/chrono/chrono.hpp>

#include "ecrt.h"

#include "l7na/drives.h"
#include "l7na/logger.h"
#include "l7na/exceptions.h"
#include "types_int.h"
#include "axisparams.h"

/*! @todo
 *  1. DONE - Failed to get reference clock time
 *  2. Check drive parameters setting
 *  3. DONE - Stop/Disable priority
 *  4. Controlword/Statusword - pretty print.
 */

namespace Drives {

namespace fs = boost::filesystem;

using SysClock = boost::chrono::system_clock;
using TimePoint = SysClock::time_point;

DECLARE_EXCEPTION(Exception, common::Exception);
DECLARE_EXCEPTION(TestFailedException, common::Exception);

namespace {

const size_t gkTempSensorsCount = 3;
const uint64_t gkTempSensorsReadPeriodCycles = 1000;

} // namespaces

std::string GetSystemStateName(const SystemState& ss) {
    if (SYSTEM_OFF == ss) {
        return "off";
    } else if (SYSTEM_INIT == ss) {
        return "init";
    } else if (SYSTEM_READY == ss) {
        return "ready";
    } else if (SYSTEM_PROCESSING == ss) {
        return "processing";
    } else if (SYSTEM_WARNING == ss) {
        return "warning";
    } else if (SYSTEM_ERROR == ss) {
        return "error";
    } else if (SYSTEM_FATAL_ERROR == ss) {
        return "fatal";
    }

    LOG_ERROR("GetSystemStateName(): unknown system state '" << ss << "'");

    return "unknown";
}


std::string GetAxisStateName(const AxisState& as) {
    if (AXIS_DISABLED == as) {
        return "disabled";
    } else if (AXIS_INIT == as) {
        return "init";
    } else if (AXIS_IDLE == as) {
        return "idle";
    } else if (AXIS_ENABLED == as) {
        return "enabled";
    } else if (AXIS_STOP == as) {
        return "stop";
    } else if (AXIS_WARNING == as) {
        return "warning";
    } else if (AXIS_ERROR == as) {
        return "error";
    }

    LOG_ERROR("GetAxisStateName(): unknown axis state '" << as << "'");

    return "unknown";
}

AxisStatus::AxisStatus()
    : tgt_pos_deg(0.0)
    , cur_pos_deg(0.0)
    , dmd_pos_deg(0.0)
    , tgt_vel_deg(0.0)
    , cur_vel_deg(0.0)
    , dmd_vel_deg(0.0)
    , cur_pos_abs(0)
    , cur_pos(0)
    , dmd_pos(0)
    , tgt_pos(0)
    , cur_vel(0)
    , dmd_vel(0)
    , tgt_vel(0)
    , cur_torq(0)
    , state(AxisState::AXIS_DISABLED)
    , error_code(0)
    , cur_temperature0(0)
    , cur_temperature1(0)
    , cur_temperature2(0)
    , ctrlword(0)
    , statusword(0)
    , mode(OP_MODE_NOT_SET)
{}

SystemStatus::SystemStatus() noexcept
    : state(SystemState::SYSTEM_OFF)
    , reftime(0)
    , apptime(0)
    , dcsync(0)
{}

class Control::Impl {
public:
    ~Impl() {
        if (m_thread) {
            m_stop_flag.store(true, std::memory_order_relaxed);
            m_thread->join();
            m_thread.reset();
        }

        // Освобождаем мастер-объект
        if (m_master) {
            ecrt_release_master(m_master);
            LOG_INFO("Master released");
        }
    }

protected:
    friend class Control;

    Impl(const Config::Storage& config, const ParamsMode params_mode)
        : m_config(config)
        , m_sdo_cfg()
        , m_sys_info{}
        , m_sys_status{}
        , m_stop_flag(false)
        , m_thread()
        , m_tx_queues()
        , m_master(NULL)
        , m_domain(NULL)
        , m_domain_data(NULL)
    {
        m_move_modes[AZIMUTH_AXIS] = kAzimAutoMoveModeMap;
        m_move_modes[ELEVATION_AXIS] = kElevAutoMoveModeMap;

        std::memset(m_pos_abs_usr_off, 0, AXIS_COUNT * sizeof(decltype(m_pos_abs_usr_off[0])));
        std::memset(m_pos_abs_rel_off, 0, AXIS_COUNT * sizeof(decltype(m_pos_abs_rel_off[0])));

        for (int32_t axis = AXIS_MIN; axis < AXIS_COUNT; ++axis) {
            m_params_mode[axis] = params_mode;
            m_cur_move_mode[axis] = kMoveModeInvalid;
            m_tx_queues[axis].push_back(kTxCmdInit);
            m_tx_queues[axis].push_back(kTxCmdIdle);
        }

        try {
            // Создаем мастер-объект
            m_master = ecrt_request_master(0);

            if (m_master) {
                LOG_INFO("Master requested");
            } else {
                BOOST_THROW_EXCEPTION(Exception("Unable to request master"));
            }

            // Создаем объект для обмена PDO в циклическом режиме.
            m_domain = ecrt_master_create_domain(m_master);

            if (m_domain) {
                LOG_INFO("Process data domain created");
            } else {
                BOOST_THROW_EXCEPTION(Exception("Unable to create process data domain"));
            }

            // Создаем объекты конфигурации подчиненных.
            for (uint32_t d = 0; d < AXIS_COUNT; ++d) {
                m_slave_cfg[d] = ecrt_master_slave_config(m_master, 0, d, 0x00007595, 0x00010001);
            }

            bool all_slave_configs_ok = true;
            for (uint32_t d = 0; d < AXIS_COUNT; ++d) {
                all_slave_configs_ok |= (m_slave_cfg[d] != NULL);
            }

            if (all_slave_configs_ok) {
                LOG_INFO("Slave configuration objects (" << AXIS_COUNT << ") created");
            } else {
                BOOST_THROW_EXCEPTION(Exception("Failed to create some slave configuration"));
            }

            // Конфигурируем PDO подчиненных
            // TxPDO
            ec_pdo_entry_info_t l7na_tx_channel[] = {
                {0x6041, 0, 16},    // Statusword
                {0x6060, 0, 8},     // Actual mode of operation
                {0x607A, 0, 32},    // Target position value
                {0x6062, 0, 32},    // Demand position value
                {0x6064, 0, 32},    // Actual position value
                {0x2607, 0, 32},    // Actual position value (absolute)
                {0x603F, 0, 16},    // Error code
                {0x606B, 0, 32},    // Demand velocity Value
                {0x606C, 0, 32},    // Actual velocity value
                {0x6077, 0, 16},    // Actual torque value
            };

            ec_pdo_info_t l7na_tx_pdos[] = {
                {0x1A00, sizeof(l7na_tx_channel)/sizeof(l7na_tx_channel[0]), l7na_tx_channel}
            };

            // RxPDO
            ec_pdo_entry_info_t l7na_rx_channel[] = {
                {0x6040, 0, 16},    // Controlword
                {0x6060, 0, 8},     // Actual mode of operation
                {0x607A, 0, 32},    // Target position
                {0x60FF, 0, 32},    // Target velocity
            };

            ec_pdo_info_t l7na_rx_pdos[] = {
                {0x1600, sizeof(l7na_rx_channel)/sizeof(l7na_rx_channel[0]), l7na_rx_channel}
            };

            // Конфигурация SyncManagers 2 (FMMU0) и 3 (FMMU1)
            // { sync_mgr_idx, sync_mgr_direction, pdo_num, pdo_ptr, watch_dog_mode }
            // { 0xFF - end marker}
            ec_sync_info_t l7na_syncs[] = {
                {2, EC_DIR_OUTPUT, 1, l7na_rx_pdos, EC_WD_DEFAULT},
                {3, EC_DIR_INPUT, 1, l7na_tx_pdos, EC_WD_DEFAULT},
                {0xFF}
            };

            // Конфиугурируем PDO для подчиненных
            for (uint32_t d = AXIS_MIN; d < AXIS_COUNT; ++d) {
                if (ecrt_slave_config_pdos(m_slave_cfg[d], EC_END, l7na_syncs)) {
                    BOOST_THROW_EXCEPTION(Exception("Failed to configure slave #") << d << " pdos");
                }

                ecrt_slave_config_watchdog(m_slave_cfg[d], 0, 0);
            }

            LOG_INFO("Configuring slave PDOs and sync managers done");

            static const ec_pdo_entry_reg_t kDomainPDOs[] = {
                {0, ELEVATION_AXIS, 0x00007595, 0x00010001, 0x6040, 0, &m_offrw_ctrl[ELEVATION_AXIS]},        //!< Control word
                {0, ELEVATION_AXIS, 0x00007595, 0x00010001, 0x6041, 0, &m_offro_status[ELEVATION_AXIS]},      //!< Status word
                {0, ELEVATION_AXIS, 0x00007595, 0x00010001, 0x607A, 0, &m_offrw_tgt_pos[ELEVATION_AXIS]},     //!< Target position
                {0, ELEVATION_AXIS, 0x00007595, 0x00010001, 0x6062, 0, &m_offro_dmd_pos[ELEVATION_AXIS]},     //!< Demand position
                {0, ELEVATION_AXIS, 0x00007595, 0x00010001, 0x6064, 0, &m_offro_act_pos[ELEVATION_AXIS]},     //!< Actual position
                {0, ELEVATION_AXIS, 0x00007595, 0x00010001, 0x60FF, 0, &m_offrw_tgt_vel[ELEVATION_AXIS]},     //!< Target velocity
                {0, ELEVATION_AXIS, 0x00007595, 0x00010001, 0x606B, 0, &m_offro_dmd_vel[ELEVATION_AXIS]},     //!< Demand velocity
                {0, ELEVATION_AXIS, 0x00007595, 0x00010001, 0x606C, 0, &m_offro_act_vel[ELEVATION_AXIS]},     //!< Actual velocity
                {0, ELEVATION_AXIS, 0x00007595, 0x00010001, 0x2607, 0, &m_offro_act_pos_abs[ELEVATION_AXIS]}, //!< Actual position (absolute)
                {0, ELEVATION_AXIS, 0x00007595, 0x00010001, 0x6060, 0, &m_offrw_act_mode[ELEVATION_AXIS]},    //!< Actual drive mode of operation
                {0, ELEVATION_AXIS, 0x00007595, 0x00010001, 0x6077, 0, &m_offro_act_torq[ELEVATION_AXIS]},    //!< Actual torque
                {0, ELEVATION_AXIS, 0x00007595, 0x00010001, 0x603F, 0, &m_offro_err_code[ELEVATION_AXIS]},    //!< Error code

                {0, AZIMUTH_AXIS,   0x00007595, 0x00010001, 0x6040, 0, &m_offrw_ctrl[AZIMUTH_AXIS]},          //!< Control word
                {0, AZIMUTH_AXIS,   0x00007595, 0x00010001, 0x6041, 0, &m_offro_status[AZIMUTH_AXIS]},        //!< Status word
                {0, AZIMUTH_AXIS,   0x00007595, 0x00010001, 0x607A, 0, &m_offrw_tgt_pos[AZIMUTH_AXIS]},       //!< Target position
                {0, AZIMUTH_AXIS,   0x00007595, 0x00010001, 0x6062, 0, &m_offro_dmd_pos[AZIMUTH_AXIS]},       //!< Demand position
                {0, AZIMUTH_AXIS,   0x00007595, 0x00010001, 0x6064, 0, &m_offro_act_pos[AZIMUTH_AXIS]},       //!< Actual position
                {0, AZIMUTH_AXIS,   0x00007595, 0x00010001, 0x60FF, 0, &m_offrw_tgt_vel[AZIMUTH_AXIS]},       //!< Target velocity
                {0, AZIMUTH_AXIS,   0x00007595, 0x00010001, 0x606B, 0, &m_offro_dmd_vel[AZIMUTH_AXIS]},       //!< Demand velocity
                {0, AZIMUTH_AXIS,   0x00007595, 0x00010001, 0x606C, 0, &m_offro_act_vel[AZIMUTH_AXIS]},       //!< Actual velocity
                {0, AZIMUTH_AXIS,   0x00007595, 0x00010001, 0x2607, 0, &m_offro_act_pos_abs[AZIMUTH_AXIS]},   //!< Actual position (absolute)
                {0, AZIMUTH_AXIS,   0x00007595, 0x00010001, 0x6060, 0, &m_offrw_act_mode[AZIMUTH_AXIS]},      //!< Actual drive mode of operation
                {0, AZIMUTH_AXIS,   0x00007595, 0x00010001, 0x6077, 0, &m_offro_act_torq[AZIMUTH_AXIS]},      //!< Actual torque
                {0, AZIMUTH_AXIS,   0x00007595, 0x00010001, 0x603F, 0, &m_offro_err_code[AZIMUTH_AXIS]},      //!< Error code

                {}
            };

            // Регистируем PDO в домене
            if (ecrt_domain_reg_pdo_entry_list(m_domain, kDomainPDOs)) {
                BOOST_THROW_EXCEPTION(Exception("PDO entries registration failed"));
            }

            LOG_INFO("PDO entries registered in domain");

            // Читаем статическую информацию с подчиненных
            if (! read_system_info()) {
                BOOST_THROW_EXCEPTION(Exception("Read non-realtime system info failed"));
            }

            LOG_INFO("Static system info read");

            // Создаем sdo_requests для доступа к sdo-данным во время realtime-работы
            if (! create_sdo_requests()) {
                BOOST_THROW_EXCEPTION(Exception("Non-realtime data requests creation failed"));
            }

            LOG_INFO("Non-realtime data requests created");

            prerealtime_slave_setup();

            LOG_INFO("Pre-realtime slave setup done");

            // Задаем предполагаемый интервал обмена данными
            if (ecrt_master_set_send_interval(m_master, kCyclePeriodNs / 1000 /* us required here */)) {
                BOOST_THROW_EXCEPTION(Exception("Failed to setup master send interval"));
            }

            ///////////////////////////////////////////////////////////////////
            // Настраиваем DC-synchronization

            // Выбираем референсные часы
            if (int err = ecrt_master_select_reference_clock(m_master, m_slave_cfg[0])) {
                BOOST_THROW_EXCEPTION(Exception("Failed to select reference clock. Error code: ") << err);
            }

            // Включаем и настраиваем синхронизацию на подчиненных
            for (int32_t axis = AXIS_MIN; axis < AXIS_COUNT; ++axis) {
                ecrt_slave_config_dc(m_slave_cfg[axis], 0x300, kCyclePeriodNs, 125000, 0, 0);
            }

            // Записываем начальное application time
            ecrt_master_application_time(m_master, get_app_time());

            ///////////////////////////////////////////////////////////////////

            // "Включаем" мастер-объект
            if (ecrt_master_activate(m_master)) {
                BOOST_THROW_EXCEPTION(Exception("Master activation failed"));
            }

            LOG_INFO("Master activated");

            if (! (m_domain_data = ecrt_domain_data(m_domain)) ) {
                BOOST_THROW_EXCEPTION(Exception("Domain data initialization failed"));
            }

            LOG_INFO("Domain data registered");

            // Setup system status value
            SystemStatus s;
            s.state = SystemState::SYSTEM_INIT;
            m_sys_status.store(s);

            m_thread.reset(new std::thread(std::bind(&Impl::CyclicPolling, this)));
            sched_param param{5};
            const int ret = ::pthread_setschedparam(m_thread->native_handle(), SCHED_RR, &param);
            if (! ret) {
                LOG_WARN("Setup scheduling params for polling thread OK");
            } else {
                LOG_WARN("Setup scheduling params for polling thread failed, errno: " << errno);
            }

            LOG_INFO("Cyclic polling thread started");
        } catch (const std::exception& ex) {
            LOG_ERROR(ex.what());

            // Записываем сотояние системы
            SystemStatus s = m_sys_status.load(std::memory_order_acquire);
            s.state = SystemState::SYSTEM_FATAL_ERROR;
            // @todo Возвращать строку ошибки
            // s.error_str = ex.what();
            m_sys_status.store(s);
        }
    }

    bool SetPosAbsPulseOffset(const Axis& axis, int32_t offset) {
        if (! is_axis_valid(axis)) {
            return false;
        }

        m_pos_abs_usr_off[axis] = offset;

        return true;
    }

    bool SetModeRun(const Axis& axis, double pos /*deg*/, double vel /*deg/sec*/) {
        if (! is_axis_valid(axis)) {
            return false;
        }

        std::list<TXCmd> txcmd_list;

        const SystemStatus s = m_sys_status.load(std::memory_order_acquire);

        if (AXIS_DISABLED == s.axes[axis].state) {
            txcmd_list.push_back(kTxCmdInit);
            txcmd_list.push_back(kTxCmdIdle);
        } else if (AXIS_INIT == s.axes[axis].state) {
            txcmd_list.push_back(kTxCmdIdle);
        }

        MoveMode& axis_cur_move_mode = m_cur_move_mode[axis];
        const MoveMode axis_old_move_mode = m_cur_move_mode[axis];
        const MoveModeMap& axis_move_modes = m_move_modes[axis];
        const ParamsMode& axis_params_mode = m_params_mode[axis];

        if (vel) {
            /* Устанавливаем параметры для движения на постоянной скорости.
             * Для этого выбираем режим с максимальным значением.
             */
            if (PARAMS_MODE_AUTOMATIC == axis_params_mode
                    && ! axis_move_modes.empty()
                    && axis_cur_move_mode != axis_move_modes.rbegin()->first) {
                const AxisParams& params = axis_move_modes.rbegin()->second;
                TXCmd txcmd(TXCmd::kSetParams);
                for (const AxisParam& p: params) {
                    if (kWriteSdoIndices.find(p.index) == kWriteSdoIndices.end()) {
                        continue;
                    }
                    txcmd.param = p;
                    txcmd_list.push_back(txcmd);
                }

                axis_cur_move_mode = axis_move_modes.rbegin()->first;
            }

            TXCmd txcmd(TXCmd::kCmd);

            // Переходим в режим "Profile velocity mode" и сразу задаем требуемую скорость
            txcmd.ctrlword = 0xF;
            txcmd.op_mode = OP_MODE_SCAN;
            txcmd.tgt_vel = vel_deg2pulse(vel);
            txcmd.tgt_pos = 0;

            txcmd_list.push_back(txcmd);
        } else {
            // Current absolute position + user offset [pulses]
            const int32_t cur_pos_usr_pulse = s.axes[axis].cur_pos - m_pos_abs_rel_off[axis] - m_pos_abs_usr_off[axis];
            // Target absolute position + user offset [pulses]
            const int32_t tgt_pos_usr_pulse = pos_deg2pulse(pos, cur_pos_usr_pulse);
            // Target internal position [pulses]
            const int32_t tgt_pos = tgt_pos_usr_pulse + m_pos_abs_rel_off[axis] + m_pos_abs_usr_off[axis];
            const int32_t pos_diff_pulse = std::abs(tgt_pos - s.axes[axis].cur_pos);
            const double pos_diff_deg = pos_diff_pulse / kPulsesPerDegree;

            // Устанавливаем параметры для движения к указанной точке.
            if (PARAMS_MODE_AUTOMATIC == axis_params_mode) {
                const MoveMode move_mode = get_move_mode(axis, pos_diff_deg);
                if (move_mode != kMoveModeInvalid && axis_cur_move_mode != move_mode) {
                    const AxisParams& params = axis_move_modes.at(move_mode);
                    TXCmd txcmd(TXCmd::kSetParams);
                    for (const AxisParam& p: params) {
                        if (kWriteSdoIndices.find(p.index) == kWriteSdoIndices.end()) {
                            continue;
                        }
                        txcmd.param = p;
                        txcmd_list.push_back(txcmd);
                    }
                    axis_cur_move_mode = move_mode;
                }
            }

            TXCmd txcmd(TXCmd::kCmd);
            txcmd.tgt_pos = tgt_pos;

            // Переходим в режим "Profile position mode"
            txcmd.ctrlword = 0x2F;
            txcmd.op_mode = OP_MODE_POINT;

            txcmd_list.push_back(txcmd);

            // Задаем следующую точку для позиционирования
            txcmd.ctrlword = 0x3F;
            txcmd_list.push_back(txcmd);
        }

        // Перемещаем содержимое списка команд в очередь под локом
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            for (TXCmd& cmd: txcmd_list) {
                m_tx_queues[axis].push_back(std::move(cmd));
            }
            if (axis_old_move_mode != axis_cur_move_mode) {
                const AxisParams& params = axis_move_modes.at(axis_cur_move_mode);
                for (const AxisParam& p: params) {
                    m_cur_params[axis][p.index] = p.value;
                }
            }
        }

        return true;
    }

    bool AddMoveMode(const Axis& axis, const MoveMode& mode, const AxisParams& params) {
        if (! is_axis_valid(axis)) {
            return false;
        }

        {
            std::lock_guard<std::mutex> guard(m_mutex);
            m_move_modes[axis][mode] = params;
        }

        return true;
    }

    bool SetParamsMode(const Axis& axis, const ParamsMode& params_mode) {
        if (! is_axis_valid(axis)) {
            return false;
        }

        {
            std::lock_guard<std::mutex> guard(m_mutex);
            m_params_mode[axis] = params_mode;
        }

        return true;
    }

    bool GetParamsMode(const Axis& axis, ParamsMode& params_mode) const {
        if (! is_axis_valid(axis)) {
            return false;
        }

        params_mode = m_params_mode[axis];

        return true;
    }

    bool SetAxisParams(const Axis& axis, const AxisParams& params) {
        if (! is_axis_valid(axis)) {
            return false;
        }

        if (m_params_mode[axis] == PARAMS_MODE_AUTOMATIC) {
            LOG_WARN("SetAxisParams(axis=" << axis << ") skipped: 'Automatic' params mode is engaged");
            return false;
        }

        if (! check_axis_params(params)) {
            return false;
        }

        TXCmd txcmd(TXCmd::kSetParams);

        {
            std::lock_guard<std::mutex> guard(m_mutex);
            for (const AxisParam& p: params) {
                txcmd.param = p;
                m_tx_queues[axis].push_back(txcmd);
                m_cur_params[axis][p.index] = p.value;
            }
        }

        return true;
    }

    bool IsOperational() const {
        std::lock_guard<std::mutex> guard(m_mutex);
        return is_system_ready();
    }

    bool Enable(const Axis& axis) {
        if (! is_axis_valid(axis)) {
            return false;
        }

        const SystemStatus s = m_sys_status.load(std::memory_order_acquire);

        {
            std::lock_guard<std::mutex> guard(m_mutex);
            if (AXIS_DISABLED == s.axes[axis].state) {
                m_tx_queues[axis].push_back(kTxCmdInit);
                m_tx_queues[axis].push_back(kTxCmdEnable);
            } else if (AXIS_INIT == s.axes[axis].state || AXIS_IDLE == s.axes[axis].state || AXIS_STOP == s.axes[axis].state) {
                m_tx_queues[axis].push_back(kTxCmdEnable);
            } else if (AXIS_ENABLED == s.axes[axis].state) {
                LOG_INFO("Drive " << axis << " is in enabled state. Skip enable cmd");
            } else {
                LOG_ERROR("Can't enable drive " << axis << " from state '" << GetAxisStateName(s.axes[axis].state) << "'");
                return false;
            }
        }

        return true;
    }

    bool Disable(const Axis& axis) {
        if (! is_axis_valid(axis)) {
            return false;
        }

        {
            std::lock_guard<std::mutex> guard(m_mutex);
            m_tx_emerg_queues[axis].push_back(kTxCmdDisable);
        }

        return true;
    }

    bool Stop(const Axis& axis) {
        if (! is_axis_valid(axis)) {
            return false;
        }

        const SystemStatus s = m_sys_status.load(std::memory_order_acquire);
        const AxisStatus& a = s.axes[axis];
        {
            TXCmd txcmd(TXCmd::kCmd);

            // Clear 2bit and set 1bit
            txcmd.ctrlword = (a.ctrlword & ~0x4) | 0x2;
            txcmd.op_mode = OperationMode(a.mode);
            txcmd.tgt_pos = 0.0;
            txcmd.tgt_vel = 0.0;

            std::lock_guard<std::mutex> guard(m_mutex);
            m_tx_emerg_queues[axis].push_back(txcmd);
        }

        return true;
    }

    bool Idle(const Axis& axis) {
        if (! is_axis_valid(axis)) {
            return false;
        }

        const SystemStatus s = m_sys_status.load(std::memory_order_acquire);

        {
            std::lock_guard<std::mutex> guard(m_mutex);
            if (AXIS_DISABLED == s.axes[axis].state) {
                m_tx_queues[axis].push_back(kTxCmdInit);
                m_tx_queues[axis].push_back(kTxCmdIdle);
            } else if (AXIS_INIT == s.axes[axis].state || AXIS_ENABLED == s.axes[axis].state) {
                m_tx_queues[axis].push_back(kTxCmdIdle);
            } else if (AXIS_STOP == s.axes[axis].state) {
                m_tx_queues[axis].push_back(kTxCmdEnable);
                m_tx_queues[axis].push_back(kTxCmdIdle);
            } else if (AXIS_IDLE == s.axes[axis].state) {
                LOG_INFO("Drive " << axis << " is in idle state. Skip idle cmd");
            } else {
                LOG_ERROR("Can't enable drive " << axis << " from state '" << GetAxisStateName(s.axes[axis].state) << "'");
                return false;
            }
        }

        return true;
    }

    bool ResetFault(const Axis& axis) {
        if (! is_axis_valid(axis)) {
            return false;
        }

        {
            std::lock_guard<std::mutex> guard(m_mutex);
            m_tx_queues[axis].push_back(kTxCmdResetError);

        }

        return true;
    }

    const std::atomic<SystemStatus>& GetStatusRef() const {
        return m_sys_status;
    }

    SystemStatus GetStatusCopy() const {
        return m_sys_status.load(std::memory_order_acquire);
    }

    const SystemInfo& GetSystemInfo() const {
        // No need for mutex, as this data is written once in constructor
        return m_sys_info;
    }

    AxisParamIndexMap GetAvailableAxisParams(const Axis& /* axis */) const {
        return kWriteSdoIndices;
    }

    AxisParams GetCurAxisParams(const Axis& axis) const {
        AxisParams result;

        if (! is_axis_valid(axis)) {
            return result;
        }

        {
            std::lock_guard<std::mutex> guard(m_mutex);
            for (const auto& param_pair : m_cur_params[axis]) {
                const uint16_t index = param_pair.first;
                const int64_t value = param_pair.second;
                result.push_back({ index, value });
            }
        }

        return result;
    }

    const std::atomic<CycleTimeInfo>& GetCycleTimeInfoRef() const {
        return m_timing_info;
    }

    CycleTimeInfo GetCycleTimeInfo() const {
        return m_timing_info.load(std::memory_order_consume);
    }

    void ResetCycleTimeInfo() {
        m_timing_info.store(CycleTimeInfo());
    }

    void CyclicPolling() {
        uint64_t cycles_num = 0;
        TimePoint wakeup_time = SysClock::now(), last_start_time = {}, start_time = {}, end_time = {};
        while (! m_stop_flag.load(std::memory_order_consume)) {
            end_time = SysClock::now();
            wakeup_time += boost::chrono::nanoseconds(kCyclePeriodNs);
            boost::this_thread::sleep_until(wakeup_time);
            start_time = SysClock::now();

            update_cycle_timings(wakeup_time, last_start_time, start_time, end_time);
            last_start_time = start_time;

            receive_datagrams();

            SystemStatus sys = m_sys_status.load(std::memory_order_acquire);
            read_states(sys);
            process_pdo_data(sys);
            process_sdo_data(sys, cycles_num);
            m_sys_status.store(sys, std::memory_order_relaxed);

            prepare_emerg_cmds();
            if (is_system_ready()) {
                prepare_cmds();
            }

            sync_time();

            send_datagrams();

            ++cycles_num;
        }

        receive_datagrams();
    }

private:
    bool read_system_info() {
        SystemInfo sysinfo;

        for (int32_t axis = AXIS_MIN; axis < AXIS_COUNT; ++axis) {
            int result = 0;
            size_t result_size = 0;
            uint32_t abort_code;
            const size_t kStrLen = 1024;
            char str[kStrLen];
            result |= ecrt_master_sdo_upload(m_master, axis, 0x2002, 0, reinterpret_cast<uint8_t*>(&(sysinfo.axes[axis].encoder_pulses_per_rev)), sizeof(sysinfo.axes[axis].encoder_pulses_per_rev), &result_size, &abort_code);

            result |= ecrt_master_sdo_upload(m_master, axis, 0x1008, 0, reinterpret_cast<uint8_t*>(&str[0]), kStrLen, &result_size, &abort_code);
            if (result_size) {
                sysinfo.axes[axis].dev_name = std::string(str, result_size);
                result_size = 0;
            }

            result |= ecrt_master_sdo_upload(m_master, axis, 0x1009, 0, reinterpret_cast<uint8_t*>(&str[0]), kStrLen, &result_size, &abort_code);
            if (result_size) {
                sysinfo.axes[axis].hw_version = std::string(str, result_size);
                result_size = 0;
            }

            result |= ecrt_master_sdo_upload(m_master, axis, 0x100A, 0, reinterpret_cast<uint8_t*>(&str[0]), kStrLen, &result_size, &abort_code);
            if (result_size) {
                sysinfo.axes[axis].sw_version = std::string(str, result_size);
                result_size = 0;
            }

            if (result) {
                return false;
            }
        }

        m_sys_info = sysinfo;

        return true;
    }

    bool is_axis_valid(const Axis& axis) const {
        return (axis >= Axis::AXIS_MIN) && (axis < Axis::AXIS_COUNT);
    }

    static int32_t vel_deg2pulse(double vel_deg) {
        const int32_t vel_pulse = static_cast<decltype(vel_pulse)>(vel_deg * kPulsesPerDegree);
        return vel_pulse;
    }

    static double vel_pulse2deg(int32_t vel_pulse) {
        return static_cast<double>(vel_pulse) / kPulsesPerDegree;
    }

    static int32_t pos_deg2pulse(double tgt_pos_deg, int32_t cur_pos_pulse) {
        const static int32_t kPulsesPerHalfTurn = kPulsesPerTurn / 2;

        // Нормализуем количество градусов к диапазону [0, 360)
        tgt_pos_deg = std::fmod(tgt_pos_deg, static_cast<decltype(tgt_pos_deg)>(kDegPerTurn));
        if (tgt_pos_deg < 0) {
            tgt_pos_deg = static_cast<decltype(tgt_pos_deg)>(kDegPerTurn) + tgt_pos_deg;
        }

        const int32_t local_tgt_pos_pulse = static_cast<decltype(local_tgt_pos_pulse)>(tgt_pos_deg * kPulsesPerDegree);

        int32_t res_pos_pulse = 0;
        if (cur_pos_pulse >= 0) {
            const int32_t local_cur_pos_pulse = cur_pos_pulse % kPulsesPerTurn;
            const int32_t roundl_cur_pos_pulse = cur_pos_pulse - local_cur_pos_pulse;

            if (local_tgt_pos_pulse >= local_cur_pos_pulse) {
                if (local_tgt_pos_pulse - local_cur_pos_pulse >= kPulsesPerHalfTurn) {
                    res_pos_pulse = roundl_cur_pos_pulse - kPulsesPerTurn + local_tgt_pos_pulse;
                } else {
                    res_pos_pulse = roundl_cur_pos_pulse + local_tgt_pos_pulse;
                }
            } else {
                if (local_cur_pos_pulse - local_tgt_pos_pulse >= kPulsesPerHalfTurn) {
                    res_pos_pulse = roundl_cur_pos_pulse + kPulsesPerTurn + local_tgt_pos_pulse;
                } else {
                    res_pos_pulse = roundl_cur_pos_pulse + local_tgt_pos_pulse;
                }
            }
        } else {
            const int32_t local_cur_pos_pulse = kPulsesPerTurn + (cur_pos_pulse % kPulsesPerTurn);
            const int32_t roundl_cur_pos_pulse = cur_pos_pulse - local_cur_pos_pulse;

            if (local_tgt_pos_pulse >= local_cur_pos_pulse) {
                if (local_tgt_pos_pulse - local_cur_pos_pulse >= kPulsesPerHalfTurn) {
                    res_pos_pulse = roundl_cur_pos_pulse - kPulsesPerTurn + local_tgt_pos_pulse;
                } else {
                    res_pos_pulse = roundl_cur_pos_pulse + local_tgt_pos_pulse;
                }
            } else {
                if (local_cur_pos_pulse - local_tgt_pos_pulse >= kPulsesPerHalfTurn) {
                    res_pos_pulse = roundl_cur_pos_pulse + kPulsesPerTurn + local_tgt_pos_pulse;
                } else {
                    res_pos_pulse = roundl_cur_pos_pulse + local_tgt_pos_pulse;
                }
            }
        }

        return res_pos_pulse;
    }

    static double pos_pulse2deg(int32_t axis, int32_t pos_pulse) {
        const bool is_pos_negative = (pos_pulse < 0);
        const int32_t local_pos_pulse = pos_pulse % kPulsesPerTurn;

        double local_pos_deg = static_cast<double>(local_pos_pulse) / kPulsesPerDegree;
        if (is_pos_negative) {
            local_pos_deg += static_cast<double>(kDegPerTurn); // [-360, 0) -> [0, 360)
        }
        if (Axis::ELEVATION_AXIS == axis && local_pos_deg >= 180.0) {
            local_pos_deg -= static_cast<double>(kDegPerTurn); // [0, 360) -> [-180,180)
        }
        return local_pos_deg;
    }

    uint64_t get_app_time() {
        const uint64_t since_epoch_ns = boost::chrono::system_clock::now().time_since_epoch().count();
        const uint64_t since_1_1_2000_ns = since_epoch_ns - kEpoch112000DiffNs;

        return since_1_1_2000_ns;
    }

    bool create_sdo_requests() {
        for (int32_t axis = AXIS_MIN; axis < AXIS_COUNT; ++axis) {
            // Temperature
            for (size_t i = 0; i < gkTempSensorsCount; ++i) {
                m_temperature_sdo[axis][i] = ecrt_slave_config_create_sdo_request(m_slave_cfg[axis], 0x260B, 0, 2);
                if (! m_temperature_sdo[axis][i]) {
                    return false;
                }
                // @todo Вынести в настройки
                ecrt_sdo_request_timeout(m_temperature_sdo[axis][i], 10000 /*ms*/);
            }

            // Sdos for download
            std::map<uint16_t, ec_sdo_request_t*>& axis_write_sdos = m_write_sdos[axis];
            for (const std::pair<uint16_t, uint16_t> sdo_info: kWriteSdoIndices) {
                const uint16_t& sdo_idx = sdo_info.first;
                const uint16_t& sdo_data_size = sdo_info.second;
                ec_sdo_request* sdo_req = ecrt_slave_config_create_sdo_request(m_slave_cfg[axis], sdo_idx, 0, sdo_data_size);
                if (! sdo_req) {
                    LOG_ERROR("Failed to create sdo idx=" << sdo_idx);
                    return false;
                }

                assert(axis_write_sdos.end() == axis_write_sdos.find(sdo_idx));

                axis_write_sdos[sdo_idx] = sdo_req;
                // @todo Вынести в настройки
                ecrt_sdo_request_timeout(sdo_req, 10000 /*ms*/);
            }
        }

        return true;
    }

    void prerealtime_slave_setup() {
        uint32_t abort_code = 0;
        const Config::Storage::KeyValueDict& cfg = m_config.GetWholeDict();
        for (auto pair_it = cfg.begin(); pair_it != cfg.end(); ++pair_it) {
            const Config::Storage::Key& key_tup = pair_it->first;
            const Config::Storage::Value& val_tup = pair_it->second;

            const uint16_t axis = boost::get<0>(key_tup);
            const uint16_t index = boost::get<1>(key_tup);
            const uint8_t subindex = boost::get<2>(key_tup);
            int32_t val = boost::get<0>(val_tup);
            const uint8_t val_size = boost::get<1>(val_tup);

            const int result = ecrt_master_sdo_download(m_master, axis, index, subindex, reinterpret_cast<uint8_t*>(&val), val_size, &abort_code);
            if (result) {
                BOOST_THROW_EXCEPTION(Exception("Pre-realtime slave setup failed. Key=") << axis << ":"
                                      << index << " :" << static_cast<uint16_t>(subindex) << " = "
                                      << val << ":" << val_size << ", abort_code=" << abort_code);
            }

            // Добавляем в текущие значения 'значимых' параметров для оси
            if (kWriteSdoIndices.find(index) != kWriteSdoIndices.end()) {
                m_cur_params[axis][index] = val;
            }
        }

        const auto uploadEthercatRegister = [this](uint16_t axis, uint16_t index, uint8_t subindex) -> int32_t {
            uint32_t abort_code;
            size_t result_size;
            int32_t value;
            const int err = ecrt_master_sdo_upload(m_master, axis, index, subindex, reinterpret_cast<uint8_t*>(&value), sizeof(value), &result_size, &abort_code);
            if (err) {
                BOOST_THROW_EXCEPTION(Exception("Pre-realtime slave setup: failed to upload value for axis=") << axis
                                      << " index=" << index << "/" << static_cast<uint16_t>(subindex)
                                      << " abort_code=" << abort_code);
            }

            return value;
        };

        // Get absolute-relative position offset for axes
        for (int32_t axis = AXIS_MIN; axis < AXIS_COUNT; ++axis) {
            const int32_t rel_pos = uploadEthercatRegister(axis, 0x6064, 0);
            const int32_t abs_pos = uploadEthercatRegister(axis, 0x2607, 0);
            m_pos_abs_rel_off[axis] = (rel_pos % kPulsesPerTurn) - abs_pos;
        }
    }

    AxisState compute_axis_state(uint16_t statusword) {
        if ((statusword & 0xF) == 0x8) {
            return AXIS_ERROR;
        } else if ((statusword & 0x80) == 0x80) {
            return AXIS_WARNING;
        } else if ((statusword & 0x6F) == 0x7) {
            return AXIS_STOP;
        } else if ((statusword & 0x6F) == 0x27) {
            return AXIS_ENABLED;
        } else if ((statusword & 0x6F) == 0x23) {
            return AXIS_IDLE;
        } else if ((statusword & 0x6F) == 0x21) {
            return AXIS_INIT;
        } else if ((statusword & 0x4F) == 0x0 || (statusword & 0x4F) == 0x40) {
            return AXIS_DISABLED;
        }

        LOG_ERROR("Unknown axis stasusword: 0x" << std::hex << statusword);

        return AXIS_ERROR;
    }

    void update_cycle_timings(const TimePoint& wakeup_time, const TimePoint& last_start_time
                              , const TimePoint& start_time, const TimePoint& end_time) {
        // Gather cycle time info
        CycleTimeInfo timing_info = m_timing_info.load(std::memory_order_consume);
        timing_info.latency_ns = (start_time - wakeup_time).count();
        timing_info.period_ns = (start_time - last_start_time).count();
        timing_info.exec_ns = (end_time - last_start_time).count();

        if (timing_info.latency_ns > timing_info.latency_max_ns) {
            timing_info.latency_max_ns = timing_info.latency_ns;
        }
        if (timing_info.latency_ns < timing_info.latency_min_ns) {
            timing_info.latency_min_ns = timing_info.latency_ns;
        }
        if (timing_info.period_ns > timing_info.period_max_ns) {
            timing_info.period_max_ns = timing_info.period_ns;
        }
        if (timing_info.period_ns < timing_info.period_min_ns) {
            timing_info.period_min_ns = timing_info.period_ns;
        }
        if (timing_info.exec_ns > timing_info.exec_max_ns) {
            timing_info.exec_max_ns = timing_info.exec_ns;
        }
        if (timing_info.exec_ns < timing_info.exec_min_ns) {
            timing_info.exec_min_ns = timing_info.exec_ns;
        }
        m_timing_info.store(timing_info);
    }

    void receive_datagrams() {
        // Fetches received frames from the hardware and processes the datagrams
        ecrt_master_receive(m_master);
        ecrt_domain_process(m_domain);
    }

    void read_states(SystemStatus& sys) {
        ec_master_state_t ms;
        ecrt_master_state(m_master, &ms);
        if (ms.slaves_responding != m_master_state.slaves_responding) {
            LOG_INFO("Master: slaves responding " << m_master_state.slaves_responding << " -> " << ms.slaves_responding);
        }
        m_master_state = ms;

        ec_domain_state_t ds;
        ecrt_domain_state(m_domain, &ds);
        if (ds.wc_state != m_domain_state.wc_state) {
            LOG_DEBUG("Domain: WC state " << domain_wc_name(m_domain_state.wc_state) << " -> " << domain_wc_name(ds.wc_state));
        }
        if (ds.wc_state == EC_WC_COMPLETE) {
            sys.state = SYSTEM_READY;
        } else /* if (ds.wc_state == EC_WC_INCOMPLETE)*/ {
            sys.state = SYSTEM_PROCESSING;
        }
        m_domain_state = ds;

        ec_slave_config_state_t scs[AXIS_COUNT];
        for (int32_t aidx = AXIS_MIN; aidx < AXIS_COUNT; ++aidx) {
            ecrt_slave_config_state(m_slave_cfg[aidx], &scs[aidx]);
            if (scs[aidx].online != m_slave_cfg_states[aidx].online) {
                LOG_INFO("Slave " << aidx <<  ": online " << uint32_t(m_slave_cfg_states[aidx].online) << " -> " << uint32_t(scs[aidx].online));
            }
            if (scs[aidx].operational != m_slave_cfg_states[aidx].operational) {
                LOG_INFO("Slave " << aidx << ": operational " << uint32_t(m_slave_cfg_states[aidx].operational) << " -> " << uint32_t(scs[aidx].operational));
            }
            m_slave_cfg_states[aidx] = scs[aidx];
        }
    }

    std::string domain_wc_name(const ec_wc_state_t& wcs) {
        if (wcs == EC_WC_INCOMPLETE) {
            return "incomplete";
        } else if (wcs == EC_WC_COMPLETE) {
            return "ready";
        }

        return "zero";
    }

    void process_pdo_data(SystemStatus& sys) {
        for (int32_t axis = AXIS_MIN; axis < AXIS_COUNT; ++axis) {
            // Читаем данные PDO для двигателя c индексом axis
            sys.axes[axis].cur_pos = EC_READ_S32(m_domain_data + m_offro_act_pos[axis]);
            sys.axes[axis].cur_pos_abs = EC_READ_S32(m_domain_data + m_offro_act_pos_abs[axis]);

            sys.axes[axis].tgt_pos = EC_READ_S32(m_domain_data + m_offrw_tgt_pos[axis]);
            sys.axes[axis].dmd_pos = EC_READ_S32(m_domain_data + m_offro_dmd_pos[axis]);
            sys.axes[axis].cur_vel = EC_READ_S32(m_domain_data + m_offro_act_vel[axis]);
            sys.axes[axis].dmd_vel = EC_READ_S32(m_domain_data + m_offro_dmd_vel[axis]);

            sys.axes[axis].cur_pos_deg = pos_pulse2deg(axis, sys.axes[axis].cur_pos - m_pos_abs_rel_off[axis] - m_pos_abs_usr_off[axis]);
            sys.axes[axis].tgt_pos_deg = pos_pulse2deg(axis, sys.axes[axis].tgt_pos - m_pos_abs_rel_off[axis] - m_pos_abs_usr_off[axis]);
            sys.axes[axis].dmd_pos_deg = pos_pulse2deg(axis, sys.axes[axis].dmd_pos - m_pos_abs_rel_off[axis] - m_pos_abs_usr_off[axis]);
            sys.axes[axis].cur_vel_deg = vel_pulse2deg(sys.axes[axis].cur_vel);
            sys.axes[axis].dmd_vel_deg = vel_pulse2deg(sys.axes[axis].dmd_vel);

            sys.axes[axis].cur_torq = EC_READ_S16(m_domain_data + m_offro_act_torq[axis]);
            sys.axes[axis].ctrlword = EC_READ_U16(m_domain_data + m_offrw_ctrl[axis]);
            sys.axes[axis].statusword = EC_READ_U16(m_domain_data + m_offro_status[axis]);
            sys.axes[axis].mode = OperationMode(EC_READ_S8(m_domain_data + m_offrw_act_mode[axis]));

            sys.axes[axis].state = compute_axis_state(sys.axes[axis].statusword);
            sys.axes[axis].error_code = EC_READ_U16(m_domain_data + m_offro_err_code[axis]);

            // Отладочные данные
            sys.axes[axis].params_mode = m_params_mode[axis];
            sys.axes[axis].move_mode = m_cur_move_mode[axis];
        }

        // Upper estimation of the maximum time difference in ns
        const uint32_t dcsync = ecrt_master_sync_monitor_process(m_master);

        // Get reference clock time
        uint32_t lo_reftime = 0;
        const int ret = ecrt_master_reference_clock_time(m_master, &lo_reftime);
        if (ret) {
            if (ret == -EIO) {
                LOG_WARN("Slave sync datagram wasn't received");
            } else if (ret == -ENXIO) {
                LOG_WARN("No reference clock found");
            }
            // SystemStatus s = m_sys_status.load(std::memory_order_acquire);
            // s.state = SystemState::SYSTEM_WARNING;
            // m_sys_status.store(s);
            // @todo save error code
        }

        const uint64_t apptime = get_app_time();
        const uint64_t reftime = (apptime & 0xFFFFFFFF00000000UL) | lo_reftime;

        sys.reftime = reftime + kEpoch112000DiffNs;
        sys.apptime = apptime + kEpoch112000DiffNs;
        sys.dcsync = dcsync;

        for (int32_t aidx = AXIS_MIN; aidx < AXIS_COUNT; ++aidx) {
            AxisStatus& axis = sys.axes[aidx];
            if (axis.state == AXIS_ERROR || axis.error_code) {
                sys.state = SystemState::SYSTEM_ERROR;
                break;
            } else if (axis.state == AXIS_WARNING) {
                sys.state = SystemState::SYSTEM_WARNING;
                break;
            }
        }
    }

    void process_sdo_data(SystemStatus& sys, uint64_t cycle_num) {
        for (int32_t axis = AXIS_MIN; axis < AXIS_COUNT; ++axis) {
            for (size_t i = 0; i < gkTempSensorsCount; ++i) {
                const ec_request_state_t sdo_req_state = ecrt_sdo_request_state(m_temperature_sdo[axis][i]);
                if (sdo_req_state == EC_REQUEST_SUCCESS) {
                    sys.axes[axis].cur_temperature0 = EC_READ_S16(ecrt_sdo_request_data(m_temperature_sdo[axis][i]));
                    // @todo Вынести в настройки
                    if (cycle_num % gkTempSensorsReadPeriodCycles == 0) {
                        ecrt_sdo_request_read(m_temperature_sdo[axis][i]);
                    }
                } else if (sdo_req_state == EC_REQUEST_UNUSED) {
                    ecrt_sdo_request_read(m_temperature_sdo[axis][i]);
                }
            }
        }
    }

    bool is_system_ready() const {
        bool all_slaves_op = true;
        for (uint32_t aidx = 0; aidx < AXIS_COUNT; ++aidx) {
            all_slaves_op |= m_slave_cfg_states[aidx].operational;
        }

        return all_slaves_op && m_domain_state.wc_state == EC_WC_COMPLETE;
    }

    void prepare_emerg_cmds() {
        std::lock_guard<std::mutex> guard(m_mutex);

        auto queue_emerg_cmd = [this](int32_t aidx, const TXCmd& txcmd) {
            EC_WRITE_U8 (m_domain_data + m_offrw_act_mode[aidx], txcmd.op_mode);
            EC_WRITE_U16(m_domain_data + m_offrw_ctrl[aidx],     txcmd.ctrlword);
        };

        for (int32_t aidx = AXIS_MIN; aidx < AXIS_COUNT; ++aidx) {
            const std::list<TXCmd>& queue = m_tx_emerg_queues[aidx];

            if(! queue.size()) {
                continue;
            }

            bool emerg_cmd_found = false;
            // Search for disable cmd
            for (const TXCmd& txcmd: queue) {
                if (kTxCmdDisable.ctrlword == txcmd.ctrlword) {
                    queue_emerg_cmd(aidx, txcmd);
                    emerg_cmd_found = true;
                    break;
                }
            }

            if (! emerg_cmd_found) {
                // Search for stop cmd
                for (const TXCmd& txcmd: queue) {
                    if (! (txcmd.ctrlword & 0x4) && (txcmd.ctrlword & 0x2)) {
                        queue_emerg_cmd(aidx, txcmd);
                        emerg_cmd_found = true;
                        break;
                    }
                }
            }

            if (emerg_cmd_found) {
                // Remove all commands from all queues
                m_tx_emerg_queues[aidx].clear();
                m_tx_queues[aidx].clear();
                m_cur_move_mode[aidx] = kMoveModeInvalid;
                continue;
            }
        }
    }

    void prepare_cmds() {
        std::lock_guard<std::mutex> guard(m_mutex);
        for (int32_t aidx = AXIS_MIN; aidx < AXIS_COUNT; ++aidx) {
            std::list<TXCmd>& queue = m_tx_queues[aidx];

            if(! queue.size()) {
                continue;
            }

            // One command at a time
            TXCmd txcmd = queue.front();
            if (TXCmd::kCmd == txcmd.type) {
                if (txcmd.op_mode == OP_MODE_NOT_SET) {
                    EC_WRITE_U8 (m_domain_data + m_offrw_act_mode[aidx], txcmd.op_mode);
                    EC_WRITE_U16(m_domain_data + m_offrw_ctrl[aidx],     txcmd.ctrlword);
                } else if (txcmd.op_mode == OP_MODE_POINT) {
                    EC_WRITE_U8 (m_domain_data + m_offrw_act_mode[aidx], txcmd.op_mode);
                    EC_WRITE_U16(m_domain_data + m_offrw_ctrl[aidx],     txcmd.ctrlword);
                    EC_WRITE_S32(m_domain_data + m_offrw_tgt_pos[aidx],  txcmd.tgt_pos);
                } else if (txcmd.op_mode == OP_MODE_SCAN) {
                    EC_WRITE_U8 (m_domain_data + m_offrw_act_mode[aidx], txcmd.op_mode);
                    EC_WRITE_U16(m_domain_data + m_offrw_ctrl[aidx],     txcmd.ctrlword);
                    EC_WRITE_S32(m_domain_data + m_offrw_tgt_vel[aidx],  txcmd.tgt_vel);
                }
                // Remove last command from queue
                queue.pop_front();
            } else if (TXCmd::kSetParams == txcmd.type) {
                bool axis_params_set = false;
                std::map<uint16_t, ec_sdo_request*>& axis_write_sdos = m_write_sdos[aidx];

                //! @todo Сheck if all of them have been applied from current params set
                while(true) {
                    // This index MUST be there. We've checked it earlier
                    ec_sdo_request* sdo_req = axis_write_sdos.at(txcmd.param.index);
                    const ec_request_state_t sdo_state = ecrt_sdo_request_state(sdo_req);
                    const uint16_t value_size = kWriteSdoIndices.at(txcmd.param.index);

                    //! @todo Infinite loop if state is BUSY all the time
                    if (EC_REQUEST_BUSY == sdo_state) {
                        break;
                    }

                    if (1 == value_size) {
                        EC_WRITE_S8(ecrt_sdo_request_data(sdo_req), txcmd.param.value);
                    } else if (2 == value_size) {
                        EC_WRITE_S16(ecrt_sdo_request_data(sdo_req), txcmd.param.value);
                    } else if (4 == value_size) {
                        EC_WRITE_S32(ecrt_sdo_request_data(sdo_req), txcmd.param.value);
                    } else {
                        assert(false);
                    }

                    // Ставим в очередь запрос на запись SDO
                    ecrt_sdo_request_write(sdo_req);

                    // Устанавливаем флаг произошедшей записи
                    axis_params_set = true;

                    // Удаляем команду из очереди
                    queue.pop_front();

                    if(! queue.size()) {
                        break;
                    }

                    // Если следующая команда - установка параметра оси, то пытаемся ее обработать
                    txcmd = queue.front();
                    if (TXCmd::kSetParams != txcmd.type) {
                        break;
                    }
                }

                // Выходим из цикла обработки осей, в случае, если мы записали хотя бы один параметр для оси,
                // чтобы не писать информацию по другим осям в этом цикле
                if (axis_params_set) {
                    break;
                }
            } else {
                assert(false);
            }
        }
    }

    void sync_time() {
        // Устанавливаем application-time
        ecrt_master_application_time(m_master, get_app_time());

        // Добавляем команды на синхронизацию времени
        ecrt_master_sync_reference_clock(m_master);
        ecrt_master_sync_slave_clocks(m_master);
        // ВАЖНО: Кладет в очередь отправки запрос на получение от дочерних узлов значение регистра их оффсета от SystemTime.
        ecrt_master_sync_monitor_queue(m_master);
    }

    void send_datagrams() {
        ecrt_domain_queue(m_domain);
        ecrt_master_send(m_master);
    }

    bool check_axis_params(const AxisParams& params) {
        for (const AxisParam& p: params) {
            if (kWriteSdoIndices.find(p.index) == kWriteSdoIndices.end()) {
                LOG_ERROR("Writing sdo index=" << std::hex << "0x" << p.index << " is not supported");
                return false;
            }
        }

        return true;
    }

    MoveMode get_move_mode(const Axis& axis, const double pos_diff_deg) {
        MoveMode result = kMoveModeInvalid;

        for (const auto& mm_pair: m_move_modes[axis]) {
            const double mm_val = static_cast<double>(mm_pair.first);
            if (pos_diff_deg < mm_val) {
                result = mm_pair.first;
                break;
            }
        }

        return result;
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TESTS

    static void TEST_pos_deg2pulse() {
        struct TestResultChecker {
            void operator()(int32_t result, int32_t expected, const std::string& name) {
                if (result == expected) {
                    std::cout << "Test " << name << ": OK" << std::endl;
                } else {
                    std::cout << "Test " << name << ": FAILED (result=" << result
                              << ", expect=" << expected << ")" << std::endl;
                }
            }
        };

        TestResultChecker checker;
        checker(pos_deg2pulse(0, 0), 0, "Zero2zero");
        checker(pos_deg2pulse(0, 100000), 0, "Zero2zero2");
        checker(pos_deg2pulse(0, 600000), 1048576, "Zero2zero3");

        checker(pos_deg2pulse(300, 7400000), 7165269, "CurPosPulse>=0,LocalTgtPosPulse>=LocalCurPosPulse,LocalTgt2Cur>=HalfTurn");
        checker(pos_deg2pulse(50, 7400000), 7485667, "CurPosPulse>=0,LocalTgtPosPulse>=LocalCurPosPulse,LocalTgt2Cur<HalfTurn");

        checker(pos_deg2pulse(50, 8300000), 8534243, "CurPosPulse>=0,LocalTgtPosPulse<LocalCurPosPulse,LocalTgt2Cur>=HalfTurn");
        checker(pos_deg2pulse(300, 8300000), 8213845, "CurPosPulse>=0,LocalTgtPosPulse<LocalCurPosPulse,LocalTgt2Cur<HalfTurn");

        checker(pos_deg2pulse(300, -7300000), -7514795, "CurPosPulse<0,TgtPosPulse>=CurPosPulse,Tgt2Cur>=HalfTurn");
        checker(pos_deg2pulse(50, -7300000), -7194397, "CurPosPulse<0,TgtPosPulse>=CurPosPulse,Tgt2Cur<HalfTurn");

        checker(pos_deg2pulse(50, -7400000), -7194397, "CurPosPulse<0,TgtPosPulse<CurPosPulse,Tgt2Cur>=HalfTurn");
        checker(pos_deg2pulse(300, -7400000), -7514795, "CurPosPulse<0,TgtPosPulse<CurPosPulse,Tgt2Cur<HalfTurn");
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    struct TXCmd {
        enum Type : int8_t {
            kTypeUnknown = -1,
            kSetParams,
            kCmd
        };

        TXCmd(const OperationMode& op, uint16_t ctrl, int32_t pos, int32_t vel)
            : tgt_pos(pos)
            , tgt_vel(vel)
            , ctrlword(ctrl)
            , op_mode(op)
            , type(kCmd)
        {}

        TXCmd(uint16_t par, int64_t val)
            : type(kSetParams)
            , param{par, val}
        {}

        explicit TXCmd(const Type& t)
            : type(t)
        {}

        int32_t tgt_pos = 0;
        int32_t tgt_vel = 0;
        uint16_t ctrlword = 0;
        OperationMode op_mode = OP_MODE_NOT_SET;
        Type type = kTypeUnknown;
        AxisParam param = {0,0};
    };

    Config::Storage                 m_config;       //!< Конфигурация двигателей, задаваемая пользователем
    std::map<uint16_t, int64_t>     m_sdo_cfg;      //!< Конфигурация SDO
    SystemInfo                      m_sys_info;     //!< Структура с статической информацией о системе
    std::atomic<SystemStatus>       m_sys_status;   //!< Структура с динамической информацией о системе
    std::atomic<CycleTimeInfo>      m_timing_info;  //!< Структура с информацией о временных характеристиках работы

    //! Данные для взаимодействия с потоком циклического взаимодействия с сервоусилителями
    std::atomic<bool>               m_stop_flag;    //!< Флаг остановки потока взаимодействия
    std::unique_ptr<std::thread>    m_thread;       //!< Поток циклического обмена данными со сервоусилителями

    constexpr static uint64_t       kMaxAxisReadyCycles     = 8192;
    constexpr static uint64_t       kMaxDomainInitCycles    = 8192;
    constexpr static int32_t        kPulsesPerTurn          = 1048576; // 2^20
    constexpr static int32_t        kDegPerTurn             = 360;
    constexpr static double         kPulsesPerDegree        = 1048576.0 / 360.0;
    constexpr static uint64_t       kEpoch112000DiffNs      = 946684800000000000ULL;
    constexpr static uint32_t       kCmdQueueCapacity       = 128;
    constexpr static uint32_t       kCyclePeriodNs          = 10000000; // 10ms
    constexpr static uint32_t       kRegPerDriveCount       = 12;
    constexpr static MoveMode       kMoveModeInvalid        = -1;

    mutable std::mutex              m_mutex;

    std::list<TXCmd>                m_tx_emerg_queues[AXIS_COUNT];  //!< Emergency queues split by axis
    std::list<TXCmd>                m_tx_queues[AXIS_COUNT];        //!< Transmit queues split by axis
    const TXCmd                     kTxCmdInit     = {OP_MODE_NOT_SET, 0x6, 0, 0};
    const TXCmd                     kTxCmdIdle      = {OP_MODE_NOT_SET, 0x7, 0, 0};
    const TXCmd                     kTxCmdEnable    = {OP_MODE_NOT_SET, 0xF, 0, 0};
    const TXCmd                     kTxCmdDisable   = {OP_MODE_NOT_SET, 0x0, 0, 0};
    const TXCmd                     kTxCmdResetError = {OP_MODE_NOT_SET, 0x80, 0, 0};

    //! Структуры для обмена данными по EtherCAT
    ec_master_t*                    m_master = nullptr;
    ec_master_state_t               m_master_state = {0, 0, 0};
    ec_domain_t*                    m_domain = nullptr;
    ec_domain_state_t               m_domain_state = {0, EC_WC_ZERO, 0};
    uint8_t*                        m_domain_data = nullptr;
    ec_slave_config_t*              m_slave_cfg[AXIS_COUNT] = {nullptr, nullptr};
    ec_slave_config_state_t         m_slave_cfg_states[AXIS_COUNT] {{0, 0 , 0}, {0, 0, 0}};

    ec_sdo_request_t*               m_temperature_sdo[AXIS_COUNT][3];

    /* SDO index -> SDO data size */
    const AxisParamIndexMap kWriteSdoIndices = {
        { 0x2100, 2 }
        , { 0x2101, 2 }
        , { 0x2102, 2 }
        , { 0x2103, 2 }
        , { 0x2104, 2 }
        , { 0x2109, 2 }
        , { 0x210A, 2 }
        , { 0x210B, 2 }
        , { 0x210C, 2 }
        , { 0x210D, 2 }
        , { 0x210E, 2 }
        , { 0x210F, 2 }
        , { 0x2114, 2 }
        , { 0x2115, 2 }
        , { 0x2116, 2 }
        , { 0x2117, 2 }
        , { 0x2118, 2 }
        , { 0x6081, 4 }
        , { 0x6083, 4 }
        , { 0x6084, 4 }
    };
    using SdoReqMap = std::map<uint16_t, ec_sdo_request_t*>;
    SdoReqMap                       m_write_sdos[AXIS_COUNT];

    ParamsMode                      m_params_mode[AXIS_COUNT];

    MoveModeMap                     m_move_modes[AXIS_COUNT];
    MoveMode                        m_cur_move_mode[AXIS_COUNT];
    AxisParamValueMap               m_cur_params[AXIS_COUNT];

    //! Смещения в данных домена для параметров управления осями.
    uint32_t                        m_offrw_ctrl[AXIS_COUNT];
    uint32_t                        m_offro_status[AXIS_COUNT];
    uint32_t                        m_offrw_tgt_pos[AXIS_COUNT];
    uint32_t                        m_offro_dmd_pos[AXIS_COUNT];
    uint32_t                        m_offro_act_pos[AXIS_COUNT];
    uint32_t                        m_offrw_tgt_vel[AXIS_COUNT];
    uint32_t                        m_offro_dmd_vel[AXIS_COUNT];
    uint32_t                        m_offro_act_vel[AXIS_COUNT];
    uint32_t                        m_offro_act_pos_abs[AXIS_COUNT];
    uint32_t                        m_offrw_act_mode[AXIS_COUNT];
    uint32_t                        m_offro_act_torq[AXIS_COUNT];
    uint32_t                        m_offro_err_code[AXIS_COUNT];

    //! Пользовательские смещения коордиант [pulses] относительно абсолютных систем координат осей
    int32_t                         m_pos_abs_usr_off[AXIS_COUNT];
    int32_t                         m_pos_abs_rel_off[AXIS_COUNT];
};

constexpr uint64_t  Control::Impl::kMaxAxisReadyCycles;
constexpr uint64_t  Control::Impl::kMaxDomainInitCycles;
constexpr int32_t   Control::Impl::kPulsesPerTurn;
constexpr int32_t   Control::Impl::kDegPerTurn;
constexpr double    Control::Impl::kPulsesPerDegree;
constexpr uint64_t  Control::Impl::kEpoch112000DiffNs;
constexpr uint32_t  Control::Impl::kCmdQueueCapacity;
constexpr uint32_t  Control::Impl::kCyclePeriodNs;
constexpr uint32_t  Control::Impl::kRegPerDriveCount;
constexpr MoveMode  Control::Impl::kMoveModeInvalid;

Control::Control(const Config::Storage& config, const ParamsMode params_mode /*= PARAMS_MODE_AUTOMATIC*/)
    : m_pimpl(new Control::Impl(config, params_mode))
{}

Control::~Control() {
}

bool Control::SetPosAbsPulseOffset(const Axis& axis, int32_t offset) {
    return m_pimpl->SetPosAbsPulseOffset(axis, offset);
}

bool Control::AddMoveMode(const Axis& axis, const MoveMode& mode, const AxisParams& params) {
    return m_pimpl->AddMoveMode(axis, mode, params);
}

bool Control::SetParamsMode(const Axis& axis, const ParamsMode& params_mode) {
    return m_pimpl->SetParamsMode(axis, params_mode);
}

bool Control::GetParamsMode(const Axis& axis, ParamsMode& params_mode) const {
    return m_pimpl->GetParamsMode(axis, params_mode);
}

bool Control::SetAxisParams(const Axis& axis, const AxisParams& params) {
    return m_pimpl->SetAxisParams(axis, params);
}

bool Control::IsOperational() const {
    return m_pimpl->IsOperational();
}

bool Control::RunToPoint(const Axis& axis, double pos) {
    return m_pimpl->SetModeRun(axis, pos, 0.0);
}

bool Control::RunAtVelocity(const Axis& axis, double vel) {
    return m_pimpl->SetModeRun(axis, 0.0, vel);
}

bool Control::Enable(const Axis& axis) {
    return m_pimpl->Enable(axis);
}

bool Control::Disable(const Axis& axis) {
    return m_pimpl->Disable(axis);
}

bool Control::Stop(const Axis& axis) {
    return m_pimpl->Stop(axis);
}

bool Control::Idle(const Axis& axis) {
    return m_pimpl->Idle(axis);
}

bool Control::ResetFault(const Axis& axis) {
    return m_pimpl->ResetFault(axis);
}

const std::atomic<SystemStatus>& Control::GetStatusRef() const {
    return m_pimpl->GetStatusRef();
}

SystemStatus Control::GetStatusCopy() const {
    return m_pimpl->GetStatusCopy();
}

const SystemInfo& Control::GetSystemInfo() const {
    return m_pimpl->GetSystemInfo();
}

AxisParamIndexMap Control::GetAvailableAxisParams(const Axis& axis) const {
    return m_pimpl->GetAvailableAxisParams(axis);
}

AxisParams Control::GetCurAxisParams(const Axis& axis) const {
    return m_pimpl->GetCurAxisParams(axis);
}

const std::atomic<CycleTimeInfo> &Control::GetCycleTimeInfoRef() const {
    return m_pimpl->GetCycleTimeInfoRef();
}

CycleTimeInfo Control::GetCycleTimeInfo() const {
    return m_pimpl->GetCycleTimeInfo();
}

void Control::ResetCycleTimeInfo() {
    m_pimpl->ResetCycleTimeInfo();
}

void Control::RunStaticTests() {
    Control::Impl::TEST_pos_deg2pulse();
}

} // namespaces
