#include <sys/time.h>
#include <errno.h>

#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <thread>
#include <chrono>
#include <functional>
#include <algorithm>
#include <map>

#include <boost/filesystem/path.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/atomic/atomic.hpp>
#include <boost/memory_order.hpp>
#include <boost/thread/thread.hpp>
#include <boost/chrono/chrono.hpp>

#include "ecrt.h"

#include "l7na/drives.h"
#include "l7na/logger.h"
#include "l7na/exceptions.h"

namespace Drives {

namespace fs = boost::filesystem;

typedef boost::chrono::system_clock SysClock;

DECLARE_EXCEPTION(Exception, common::Exception);
DECLARE_EXCEPTION(TestFailedException, common::Exception);

struct CycleTimeInfo {
    SysClock::duration period_ns;
    SysClock::duration exec_ns;
    SysClock::duration latency_ns;
    SysClock::duration latency_min_ns;
    SysClock::duration latency_max_ns;
    SysClock::duration period_min_ns;
    SysClock::duration period_max_ns;
    SysClock::duration exec_min_ns;
    SysClock::duration exec_max_ns;

    CycleTimeInfo()
        : period_ns(SysClock::duration::zero())
        , exec_ns(SysClock::duration::zero())
        , latency_ns(SysClock::duration::zero())
        , latency_min_ns(SysClock::duration::max())
        , latency_max_ns(SysClock::duration::min())
        , period_min_ns(SysClock::duration::max())
        , period_max_ns(SysClock::duration::min())
        , exec_min_ns(SysClock::duration::max())
        , exec_max_ns(SysClock::duration::min())
    {}
};

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
    , state(AxisState::AXIS_OFF)
    , error_code(0)
    , cur_temperature(0)
    , ctrlword(0)
    , statusword(0)
    , mode(0)
{}

bool AxisStatus::IsReady() const {
    return state == AxisState::AXIS_IDLE || state == AxisState::AXIS_SCAN || state == AxisState::AXIS_POINT || state == AxisState::AXIS_ERROR;
}

SystemStatus::SystemStatus()
    : state(SystemState::SYSTEM_OFF)
    , reftime(0)
    , apptime(0)
    , dcsync(0)
{}

class Control::Impl {
public:
    ~Impl() {
        if (m_thread) {
            m_stop_flag.store(true, boost::memory_order_relaxed);
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

    Impl(const Config::Storage& config)
        : m_config(config)
        , m_sdo_cfg()
        , m_sys_info{}
        , m_sys_status{}
        , m_stop_flag(false)
        , m_thread()
        , m_master(NULL)
        , m_domain(NULL)
        , m_domain_data(NULL)
    {
        std::memset(m_pos_abs_usr_off, 0, AXIS_COUNT * sizeof(decltype(m_pos_abs_usr_off[0])));
        std::memset(m_pos_abs_rel_off, 0, AXIS_COUNT * sizeof(decltype(m_pos_abs_rel_off[0])));

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
                m_slave_cfg[d] = ecrt_master_slave_config(m_master, 0, d, 0x00007595, 0x00000000);
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
                {0x260D, 0, 32},    // Actual position value (absolute)
                {0x603F, 0, 16},    // Error code
                {0x606B, 0, 32},    // Demand velocity Value
                {0x606C, 0, 32},    // Actual velocity value
                {0x6077, 0, 16},    // Actual torque value
            };

            ec_pdo_info_t l7na_tx_pdos[] = {
                {0x1A00, 10, l7na_tx_channel}
            };

            // RxPDO
            ec_pdo_entry_info_t l7na_rx_channel[] = {
                {0x6040, 0, 16},    // Controlword
                {0x6060, 0, 8},     // Actual mode of operation
                {0x607A, 0, 32},    // Target position
                {0x60FF, 0, 32},    // Target velocity
                {0x6081, 0, 32},    // Profile velocity
            };

            ec_pdo_info_t l7na_rx_pdos[] = {
                {0x1600, 5, l7na_rx_channel}
            };

            // Конфигурация SyncManagers 2 (FMMU0) и 3 (FMMU1)
            // { sync_mgr_idx, sync_mgr_direction, pdo_num, pdo_ptr, watch_dog_mode }
            // { 0xFF - end marker}
            ec_sync_info_t l7na_syncs[] = {
                {2, EC_DIR_OUTPUT, 1, l7na_rx_pdos, EC_WD_DISABLE},
                {3, EC_DIR_INPUT, 1, l7na_tx_pdos, EC_WD_DISABLE},
                {0xFF}
            };

            // Конфиугурируем PDO для подчиненных
            for (uint32_t d = AXIS_MIN; d < AXIS_COUNT; ++d) {
                if (ecrt_slave_config_pdos(m_slave_cfg[d], EC_END, l7na_syncs)) {
                    BOOST_THROW_EXCEPTION(Exception("Failed to configure slave #") << d << " pdos");
                }
            }

            LOG_INFO("Configuring slave PDOs and sync managers done");

            static const ec_pdo_entry_reg_t kDomainPDOs[] = {
                {0, ELEVATION_AXIS, 0x00007595, 0x00000000, 0x6040, 0, &m_offrw_ctrl[ELEVATION_AXIS]},        //!< Control word
                {0, ELEVATION_AXIS, 0x00007595, 0x00000000, 0x6041, 0, &m_offro_status[ELEVATION_AXIS]},      //!< Status word
                {0, ELEVATION_AXIS, 0x00007595, 0x00000000, 0x607A, 0, &m_offrw_tgt_pos[ELEVATION_AXIS]},     //!< Target position
                {0, ELEVATION_AXIS, 0x00007595, 0x00000000, 0x6062, 0, &m_offro_dmd_pos[ELEVATION_AXIS]},     //!< Demand position
                {0, ELEVATION_AXIS, 0x00007595, 0x00000000, 0x6064, 0, &m_offro_act_pos[ELEVATION_AXIS]},     //!< Actual position
                {0, ELEVATION_AXIS, 0x00007595, 0x00000000, 0x60FF, 0, &m_offrw_tgt_vel[ELEVATION_AXIS]},     //!< Target velocity
                {0, ELEVATION_AXIS, 0x00007595, 0x00000000, 0x606B, 0, &m_offro_dmd_vel[ELEVATION_AXIS]},     //!< Demand velocity
                {0, ELEVATION_AXIS, 0x00007595, 0x00000000, 0x606C, 0, &m_offro_act_vel[ELEVATION_AXIS]},     //!< Actual velocity
                {0, ELEVATION_AXIS, 0x00007595, 0x00000000, 0x6081, 0, &m_offrw_prof_vel[ELEVATION_AXIS]},    //!< Profile velocity
                {0, ELEVATION_AXIS, 0x00007595, 0x00000000, 0x260D, 0, &m_offro_act_pos_abs[ELEVATION_AXIS]}, //!< Actual position (absolute)
                {0, ELEVATION_AXIS, 0x00007595, 0x00000000, 0x6060, 0, &m_offrw_act_mode[ELEVATION_AXIS]},    //!< Actual drive mode of operation
                {0, ELEVATION_AXIS, 0x00007595, 0x00000000, 0x6077, 0, &m_offro_act_torq[ELEVATION_AXIS]},    //!< Actual torque
                {0, ELEVATION_AXIS, 0x00007595, 0x00000000, 0x603F, 0, &m_offro_err_code[ELEVATION_AXIS]},    //!< Error code

                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x6040, 0, &m_offrw_ctrl[AZIMUTH_AXIS]},          //!< Control word
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x6041, 0, &m_offro_status[AZIMUTH_AXIS]},        //!< Status word
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x607A, 0, &m_offrw_tgt_pos[AZIMUTH_AXIS]},       //!< Target position
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x6062, 0, &m_offro_dmd_pos[AZIMUTH_AXIS]},       //!< Demand position
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x6064, 0, &m_offro_act_pos[AZIMUTH_AXIS]},       //!< Actual position
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x60FF, 0, &m_offrw_tgt_vel[AZIMUTH_AXIS]},       //!< Target velocity
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x606B, 0, &m_offro_dmd_vel[AZIMUTH_AXIS]},       //!< Demand velocity
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x606C, 0, &m_offro_act_vel[AZIMUTH_AXIS]},       //!< Actual velocity
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x6081, 0, &m_offrw_prof_vel[AZIMUTH_AXIS]},      //!< Profile velocity
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x260D, 0, &m_offro_act_pos_abs[AZIMUTH_AXIS]},   //!< Actual position (absolute)
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x6060, 0, &m_offrw_act_mode[AZIMUTH_AXIS]},      //!< Actual drive mode of operation
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x6077, 0, &m_offro_act_torq[AZIMUTH_AXIS]},      //!< Actual torque
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x603F, 0, &m_offro_err_code[AZIMUTH_AXIS]},      //!< Error code

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
            supply_app_time();

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

            m_thread.reset(new std::thread(std::bind(&Impl::CyclicPolling, this)));

            LOG_INFO("Cyclic polling thread started");

            // Записываем состояние системы
            SystemStatus s;
            s.state = SystemState::SYSTEM_INIT;
            s.axes[AZIMUTH_AXIS].state = AxisState::AXIS_INIT;
            s.axes[ELEVATION_AXIS].state = AxisState::AXIS_INIT;
            m_sys_status.store(s);
        } catch (const std::exception& ex) {
            LOG_ERROR(ex.what());

            // Записываем сотояние системы
            SystemStatus s = m_sys_status.load(boost::memory_order_acquire);
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
        const SystemStatus s = m_sys_status.load(boost::memory_order_acquire);
        if (! is_system_ready(s)) {
            return false;
        }

        if (! is_axis_valid(axis)) {
            return false;
        }

        if (vel) {
            TXCmd txcmd;

            // Переходим в режим "Profile velocity mode" и сразу задаем требуемую скорость
            txcmd.ctrlword = 0xF;
            txcmd.op_mode = OP_MODE_SCAN;
            txcmd.tgt_vel = vel_deg2pulse(vel);
            txcmd.tgt_pos = 0;

            m_tx_queues[axis].push(txcmd);
        } else {
            TXCmd txcmd;

            // Переходим в режим "Profile position mode"
            txcmd.ctrlword = 0xF;
            txcmd.op_mode = OP_MODE_POINT;
            txcmd.tgt_pos = 0;
            m_tx_queues[axis].push(txcmd);

            // Задаем следующую точку для позиционирования
            txcmd.ctrlword = 0x1F;
            txcmd.op_mode = OP_MODE_POINT;

            // Current absolute position + user offset [pulses]
            const int32_t cur_pos_abs_pulse_off = s.axes[axis].cur_pos - m_pos_abs_rel_off[axis] - m_pos_abs_usr_off[axis];
            // Tategt absolute position + user offset [pulses]
            const int32_t tgt_pos_abs_pulse_off = pos_deg2pulse(pos, cur_pos_abs_pulse_off);
            // Target internal position [pulses]
            txcmd.tgt_pos = tgt_pos_abs_pulse_off + m_pos_abs_rel_off[axis] + m_pos_abs_usr_off[axis];

            m_tx_queues[axis].push(txcmd);
        }

        return true;
    }

    bool SetModeIdle(const Axis& axis) {
        const SystemStatus s = m_sys_status.load(boost::memory_order_acquire);
        if (! is_system_ready(s)) {
            return false;
        }

        if (! is_axis_valid(axis)) {
            return false;
        }

        TXCmd txcmd;
        txcmd.ctrlword = 0x6;
        txcmd.op_mode = OP_MODE_IDLE;
        txcmd.tgt_vel = 0;
        txcmd.tgt_pos = 0;

        m_tx_queues[axis].push(txcmd);

        return true;
    }

    bool ResetFault(const Axis& axis) {
        const SystemStatus s = m_sys_status.load(boost::memory_order_acquire);
        if (! is_system_ready(s)) {
            return false;
        }

        if (! is_axis_valid(axis)) {
            return false;
        }

        // Set idle mode
        TXCmd txcmd;
        txcmd.ctrlword = 0x6;
        txcmd.op_mode = OP_MODE_IDLE;
        txcmd.tgt_vel = 0;
        txcmd.tgt_pos = 0;

        m_tx_queues[axis].push(txcmd);

        // Alarm/error reset
        txcmd.ctrlword = 0x86;
        txcmd.op_mode = OP_MODE_IDLE;
        txcmd.tgt_vel = 0;
        txcmd.tgt_pos = 0;

        m_tx_queues[axis].push(txcmd);

        return true;
    }

    const boost::atomic<SystemStatus>& GetStatusRef() const {
        return m_sys_status;
    }

    SystemStatus GetStatusCopy() const {
        return m_sys_status.load(boost::memory_order_acquire);
    }

    const SystemInfo& GetSystemInfo() const {
        return m_sys_info;
    }

    void CyclicPolling() {
        bool op_state = false;
        uint64_t cycles_total = 0;

        SysClock::time_point wakeup_time = SysClock::now()/*, last_start_time = {}, start_time = {}, end_time = {}*/;

        while (! op_state && ! m_stop_flag.load(boost::memory_order_consume)) {
            wakeup_time += boost::chrono::nanoseconds(kCyclePeriodNs);
            boost::this_thread::sleep_until(wakeup_time);

            // Получаем данные от подчиненных
            ecrt_master_receive(m_master);
            ecrt_domain_process(m_domain);

            ++cycles_total;

            // Получаем статус домена
            ecrt_domain_state(m_domain, &m_domain_state);

            // Получаем статус подчиненных
            ec_slave_config_state_t slave_cfg_state[AXIS_COUNT];
            for (uint32_t d = 0; d < AXIS_COUNT; ++d) {
                ecrt_slave_config_state(m_slave_cfg[d], &slave_cfg_state[d]);
            }

            bool all_slaves_up = true;
            for (uint32_t d = 0; d < AXIS_COUNT; ++d) {
                all_slaves_up |= slave_cfg_state[d].operational;
            }

            if (m_domain_state.wc_state == EC_WC_COMPLETE && all_slaves_up) {
                LOG_INFO("Domain is up at " << cycles_total << " cycles");
                op_state = true;
            } else if (cycles_total % 10000 == 0) {
                //! @todo выход из цикла и сообщение об ошибке
                LOG_WARN("Domain is NOT up at " << cycles_total << " cycles. Domain state=" << m_domain_state.wc_state
                         << ", slave0 state=" << slave_cfg_state[0].al_state
                         << ", slave1 state=" << slave_cfg_state[1].al_state
                         );
            }

            // Добавляем команды на синхронизацию времени
            supply_app_time();
            ecrt_master_sync_reference_clock(m_master);
            ecrt_master_sync_slave_clocks(m_master);
            ecrt_master_sync_monitor_queue(m_master);

            // Send queued data
            ecrt_domain_queue(m_domain);     // Помечаем данные как готовые к отправке
            ecrt_master_send(m_master);      // Отправляем все датаграммы, помещенные в очередь
        }

        // Устанавливаем статус системы в IDLE
        SystemStatus s = m_sys_status.load(boost::memory_order_acquire);
        s.state = SystemState::SYSTEM_OK;
        s.axes[AZIMUTH_AXIS].state = AxisState::AXIS_IDLE;
        s.axes[ELEVATION_AXIS].state = AxisState::AXIS_IDLE;
        m_sys_status.store(s);

        cycles_total = 0;

        CycleTimeInfo cycle_info;

        while (! m_stop_flag.load(boost::memory_order_consume)) {
            wakeup_time += boost::chrono::nanoseconds(kCyclePeriodNs);
            boost::this_thread::sleep_until(wakeup_time);
/*
            // Cycle time info gathering
            start_time = SysClock::now();
            cycle_info.latency_ns = start_time - wakeup_time;
            cycle_info.period_ns = start_time - last_start_time;
            cycle_info.exec_ns = end_time - last_start_time;
            last_start_time = start_time;

            if (cycle_info.latency_ns > cycle_info.latency_max_ns) {
                cycle_info.latency_max_ns = cycle_info.latency_ns;
            }
            if (cycle_info.latency_ns < cycle_info.latency_min_ns) {
                cycle_info.latency_min_ns = cycle_info.latency_ns;
            }
            if (cycle_info.period_ns > cycle_info.period_max_ns) {
                cycle_info.period_max_ns = cycle_info.period_ns;
            }
            if (cycle_info.period_ns < cycle_info.period_min_ns) {
                cycle_info.period_min_ns = cycle_info.period_ns;
            }
            if (cycle_info.exec_ns > cycle_info.exec_max_ns) {
                cycle_info.exec_max_ns = cycle_info.exec_ns;
            }
            if (cycle_info.exec_ns < cycle_info.exec_min_ns) {
                cycle_info.exec_min_ns = cycle_info.exec_ns;
            }
*/
            // Получаем данные от подчиненных
            ecrt_master_receive(m_master);
            ecrt_domain_process(m_domain);

            // Получаем статус домена
            ecrt_domain_state(m_domain, &m_domain_state);

            // Получаем верхнюю оценку синхронизации
            const uint32_t dcsync = ecrt_master_sync_monitor_process(m_master);

            // Получаем значение референсных часов
            uint32_t lo_ref_time = 0;
            const int err = ecrt_master_reference_clock_time(m_master, &lo_ref_time);
            if (err) {
                SystemStatus s = m_sys_status.load(boost::memory_order_acquire);
                s.state = SystemState::SYSTEM_ERROR;
                m_sys_status.store(s);
                // @todo save error code
            }

            const uint64_t app_time = get_app_time();
            const uint64_t ref_time = (app_time & 0xFFFFFFFF00000000UL) | lo_ref_time;

            // Обрабатываем пришедшие данные
            const SystemStatus s = process_received_data(cycles_total, app_time, ref_time, dcsync, cycle_info);

            // Если есть новые команды - передаем их подчиненным
            prepare_new_commands(s);

            // Устанавливаем application-time
            supply_app_time();
            // Добавляем команды на синхронизацию времени
            ecrt_master_sync_reference_clock(m_master);
            ecrt_master_sync_slave_clocks(m_master);
            // ВАЖНО: Кладет в очередь отправки запрос на получение от дочерних узлов значение регистра их оффсета от SystemTime.
            ecrt_master_sync_monitor_queue(m_master);

            // Отправляем данные подчиненным
            ecrt_domain_queue(m_domain);
            ecrt_master_send(m_master);
        }

        ecrt_master_receive(m_master);
        ecrt_domain_process(m_domain);
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
            result |= ecrt_master_sdo_upload(m_master, axis, 0x2002, 0, reinterpret_cast<uint8_t*>(&(sysinfo.axes[axis].encoder_resolution)), sizeof(sysinfo.axes[axis].encoder_resolution), &result_size, &abort_code);

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

    bool is_system_ready(const SystemStatus& s) const {
        for (int32_t axis = AXIS_MIN; axis < AXIS_COUNT; ++axis) {
            if (! s.axes[axis].IsReady()) {
                return false;
            }
        }

        return true;
    }

    bool is_axis_valid(const Axis& axis) {
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
        if (Axis::ELEVATION_AXIS == axis && local_pos_deg > 180.0) {
            local_pos_deg -= static_cast<double>(kDegPerTurn); // [0, 360) -> [-180,180)
        }
        return local_pos_deg;
    }

    uint64_t get_app_time() {
        const uint64_t since_epoch_ns = boost::chrono::system_clock::now().time_since_epoch().count();
        const uint64_t since_1_1_2000_ns = since_epoch_ns - kEpoch112000DiffNs;

        return since_1_1_2000_ns;
    }

    void supply_app_time() {
        const uint64_t app_time = get_app_time();
        ecrt_master_application_time(m_master, app_time);
    }

    bool create_sdo_requests() {
        for (int32_t axis = AXIS_MIN; axis < AXIS_COUNT; ++axis) {
            m_temperature_sdo[axis] = ecrt_slave_config_create_sdo_request(m_slave_cfg[axis], 0x2610, 0, 16);
            if (! m_temperature_sdo[axis]) {
                return false;
            }
            // @todo Вынести в настройки
            ecrt_sdo_request_timeout(m_temperature_sdo[axis], 10000 /*ms*/);
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
                                      << index << ":" << static_cast<uint16_t>(subindex)
                                      << " = "
                                      << val << ":" << val_size << ", abort_code=" << abort_code);
            }
        }
    }

    SystemStatus process_received_data(uint64_t cycle_num, uint64_t apptime, uint64_t reftime, uint32_t dcsync, const CycleTimeInfo& cycle_info) {
        SystemStatus sys = m_sys_status.load(boost::memory_order_acquire);

        for (int32_t axis = AXIS_MIN; axis < AXIS_COUNT; ++axis) {
            // Читаем данные PDO для двигателя c индексом axis

            sys.axes[axis].cur_pos = EC_READ_S32(m_domain_data + m_offro_act_pos[axis]);
            sys.axes[axis].cur_pos_abs = EC_READ_S32(m_domain_data + m_offro_act_pos_abs[axis]);

            //! @todo Do it once during prerealtime setup
            if (! m_pos_abs_rel_off[axis]) {
                m_pos_abs_rel_off[axis] = sys.axes[axis].cur_pos - sys.axes[axis].cur_pos_abs;
            }

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
            sys.axes[axis].mode = EC_READ_S8(m_domain_data + m_offrw_act_mode[axis]);
            if (sys.axes[axis].mode == OP_MODE_POINT) {
                sys.axes[axis].state = AxisState::AXIS_POINT;
            } else if (sys.axes[axis].mode == OP_MODE_SCAN) {
                sys.axes[axis].state = AxisState::AXIS_SCAN;
            } else if (sys.axes[axis].mode == OP_MODE_IDLE) {
                sys.axes[axis].state = AxisState::AXIS_IDLE;
            } else {
                sys.axes[axis].state = AxisState::AXIS_ERROR;
            }
            sys.axes[axis].error_code = EC_READ_U16(m_domain_data + m_offro_err_code[axis]);
            if (sys.axes[axis].error_code) {
                sys.axes[axis].state = AxisState::AXIS_ERROR;
            }

            // Читаем данные sdo
            ec_request_state_t sdo_req_state = ecrt_sdo_request_state(m_temperature_sdo[axis]);
            if (sdo_req_state == EC_REQUEST_SUCCESS) {
                sys.axes[axis].cur_temperature = EC_READ_S16(ecrt_sdo_request_data(m_temperature_sdo[axis]));
                // @todo Вынести в настройки
                if (cycle_num %= 10000) {
                    ecrt_sdo_request_read(m_temperature_sdo[axis]);
                }
            } else if (sdo_req_state == EC_REQUEST_UNUSED) {
                ecrt_sdo_request_read(m_temperature_sdo[axis]);
            }
        }

        sys.reftime = reftime + kEpoch112000DiffNs;
        sys.apptime = apptime + kEpoch112000DiffNs;
        sys.dcsync = dcsync;

        // @todo Слишком часто
        /*
        sys.latency_ns = cycle_info.latency_ns.count();
        sys.latency_min_ns = cycle_info.latency_min_ns.count();
        sys.latency_max_ns = cycle_info.latency_max_ns.count();
        sys.period_ns = cycle_info.period_ns.count();
        sys.period_min_ns = cycle_info.period_min_ns.count();
        sys.period_max_ns = cycle_info.period_max_ns.count();
        sys.exec_ns = cycle_info.exec_ns.count();
        sys.exec_min_ns = cycle_info.exec_min_ns.count();
        sys.exec_max_ns = cycle_info.exec_max_ns.count();
        */

        if (SystemState::SYSTEM_OK == sys.state) {
            if (std::any_of(std::begin(sys.axes), std::begin(sys.axes), [](const AxisStatus& axis) {
                return AxisState::AXIS_ERROR == axis.state;
            })) {
                sys.state = SystemState::SYSTEM_ERROR;
            }
        }

        m_sys_status.store(sys, boost::memory_order_relaxed);

        return sys;
    }

    void prepare_new_commands(const SystemStatus& sys) {
        static uint64_t cycles_cur = 0;                         // Номер текущего цикла в рамках работы
        static uint64_t cycles_cmd_start[AXIS_COUNT] = {0};     // Номер цикла начала ожидания исполнения команды

        for (int32_t axis = AXIS_MIN; axis < AXIS_COUNT; ++axis) {
            if (sys.axes[axis].mode == OP_MODE_POINT) {
                if ((sys.axes[axis].statusword & 0x7) == 0x7) {
                    if (cycles_cmd_start[axis]) {
                        LOG_DEBUG("Axis (" << axis << ") command data exchanged in " << cycles_cur - cycles_cmd_start[axis] << " cycles");
                        cycles_cmd_start[axis] = 0;
                    }
                } else if (cycles_cur - cycles_cmd_start[axis] > kMaxAxisReadyCycles) {
                    // @todo Сообщить об ошибке
                    continue;
                } else {
                    continue;
                }
            }

            TXCmd txcmd;
            if (m_tx_queues[axis].pop(txcmd)) {
                if (txcmd.op_mode == OP_MODE_IDLE) {
                    EC_WRITE_U8 (m_domain_data + m_offrw_act_mode[axis], txcmd.op_mode);
                    EC_WRITE_U16(m_domain_data + m_offrw_ctrl[axis],     txcmd.ctrlword);
                } else if (txcmd.op_mode == OP_MODE_POINT) {
                    EC_WRITE_U8 (m_domain_data + m_offrw_act_mode[axis], txcmd.op_mode);
                    EC_WRITE_U16(m_domain_data + m_offrw_ctrl[axis],     txcmd.ctrlword);
                    EC_WRITE_S32(m_domain_data + m_offrw_tgt_pos[axis],  txcmd.tgt_pos);
                    EC_WRITE_U32(m_domain_data + m_offrw_prof_vel[axis], 200000);

                    cycles_cmd_start[axis] = cycles_cur;
                } else if (txcmd.op_mode == OP_MODE_SCAN) {
                    EC_WRITE_U8 (m_domain_data + m_offrw_act_mode[axis], txcmd.op_mode);
                    EC_WRITE_U16(m_domain_data + m_offrw_ctrl[axis],     txcmd.ctrlword);
                    EC_WRITE_S32(m_domain_data + m_offrw_tgt_vel[axis],  txcmd.tgt_vel);
                }
            }
        }

        ++cycles_cur;
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

    enum OperationMode : uint8_t {
        OP_MODE_IDLE = 0,
        OP_MODE_POINT = 1,
        OP_MODE_SCAN = 3
    };

    struct TXCmd {
        TXCmd()
            : tgt_pos(0)
            , tgt_vel(0)
            , ctrlword(0)
            , op_mode(OP_MODE_IDLE)
        {}

        int32_t tgt_pos;
        int32_t tgt_vel;
        uint16_t ctrlword;
        OperationMode op_mode;
    };

    Config::Storage                 m_config;       //!< Конфигурация двигателей, задаваемая пользователем
    std::map<uint16_t, int64_t>     m_sdo_cfg;      //!< Конфигурация SDO
    SystemInfo                      m_sys_info;     //!< Структура с статической информацией о системе
    boost::atomic<SystemStatus>     m_sys_status;   //!< Структура с динамической информацией о системе

    //! Данные для взаимодействия с потоком циклического взаимодействия с сервоусилителями
    boost::atomic<bool>             m_stop_flag;    //!< Флаг остановки потока взаимодействия
    std::unique_ptr<std::thread>    m_thread;       //!< Поток циклического обмена данными со сервоусилителями


    constexpr static uint64_t           kMaxAxisReadyCycles     = 8192;
    constexpr static uint64_t           kMaxDomainInitCycles    = 8192;
    constexpr static int32_t            kPulsesPerTurn          = 1048576; // 2^20
    constexpr static int32_t            kDegPerTurn             = 360;
    constexpr static double             kPulsesPerDegree        = 1048576.0 / 360.0;
    constexpr static uint64_t           kEpoch112000DiffNs      = 946684800000000000ULL;
    constexpr static uint32_t           kCmdQueueCapacity       = 128;
    constexpr static uint32_t           kCyclePeriodNs          = 1000000;
    constexpr static uint32_t           kRegPerDriveCount       = 12;


    typedef boost::lockfree::spsc_queue<TXCmd, boost::lockfree::capacity<kCmdQueueCapacity>> TXCmdQueue;
    TXCmdQueue                      m_tx_queues[AXIS_COUNT]; //!< Очереди команд по осям

    //! Структуры для обмена данными по EtherCAT
    ec_master_t*                    m_master;
    ec_master_state_t               m_master_state;
    ec_domain_t*                    m_domain;
    ec_domain_state_t               m_domain_state;
    uint8_t*                        m_domain_data;
    ec_slave_config_t*              m_slave_cfg[AXIS_COUNT];

    ec_sdo_request_t*               m_temperature_sdo[AXIS_COUNT];

    uint32_t                        m_offrw_ctrl[AXIS_COUNT];
    uint32_t                        m_offro_status[AXIS_COUNT];
    uint32_t                        m_offrw_tgt_pos[AXIS_COUNT];
    uint32_t                        m_offro_dmd_pos[AXIS_COUNT];
    uint32_t                        m_offro_act_pos[AXIS_COUNT];
    uint32_t                        m_offrw_tgt_vel[AXIS_COUNT];
    uint32_t                        m_offro_dmd_vel[AXIS_COUNT];
    uint32_t                        m_offro_act_vel[AXIS_COUNT];
    uint32_t                        m_offrw_prof_vel[AXIS_COUNT];
    uint32_t                        m_offro_act_pos_abs[AXIS_COUNT];
    uint32_t                        m_offrw_act_mode[AXIS_COUNT];
    uint32_t                        m_offro_act_torq[AXIS_COUNT];
    uint32_t                        m_offro_err_code[AXIS_COUNT];

    int32_t                         m_pos_abs_usr_off[AXIS_COUNT];
    int32_t                         m_pos_abs_rel_off[AXIS_COUNT];
};

Control::Control(const Config::Storage& config)
    : m_pimpl(new Control::Impl(config))
{}

Control::~Control() {
}

bool Control::SetPosAbsPulseOffset(const Axis& axis, int32_t offset) {
    return m_pimpl->SetPosAbsPulseOffset(axis, offset);
}

bool Control::SetModeRun(const Axis& axis, double pos, double vel) {
    return m_pimpl->SetModeRun(axis, pos, vel);
}

bool Control::SetModeIdle(const Axis& axis) {
    return m_pimpl->SetModeIdle(axis);
}

bool Control::ResetFault(const Axis& axis) {
    return m_pimpl->ResetFault(axis);
}

const boost::atomic<SystemStatus>& Control::GetStatusRef() const {
    return m_pimpl->GetStatusRef();
}

SystemStatus Control::GetStatusCopy() const {
    return m_pimpl->GetStatusCopy();
}

const SystemInfo& Control::GetSystemInfo() const {
    return m_pimpl->GetSystemInfo();
}

void Control::RunStaticTests() {
    Control::Impl::TEST_pos_deg2pulse();
}

} // namespaces
