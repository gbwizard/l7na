#include <cstdint>
#include <cmath>
#include <thread>
#include <chrono>
#include <functional>
#include <algorithm>
#include <atomic>
#include <map>

#include <boost/filesystem/path.hpp>

#include "ecrt.h"

#include "l7na/drives.h"
#include "l7na/details/logger.h"
#include "l7na/details/exceptions.h"

namespace Drives {

namespace fs = boost::filesystem;

DECLARE_EXCEPTION(Exception, common::Exception);

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

    Impl(const char* cfg_file_path)
        : m_cfg_path (cfg_file_path)
        , m_sdo_cfg()
        , m_sys_info{}
        , m_sys_status{}
        , m_stop_flag(false)
        , m_thread()
    {
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
            const bool all_slave_configs_ok = std::all_of(&m_slave_cfg[0], &m_slave_cfg[AXIS_COUNT], [](const ec_slave_config_t* const sc) -> bool {
                return sc;
            });

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
                {0x60FF, 0, 32},    // Target velocity value
                {0x606B, 0, 32},    // Demand velocity Value
                {0x606C, 0, 32},    // Actual velocity value
                {0x6081, 0, 32},    // Profile velocity value
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
                {0, ELEVATION_AXIS, 0x00007595, 0x00000000, 0x6060, 0, &m_offrw_act_mode[ELEVATION_AXIS]},    //!< Actual drive mode of operation
                {0, ELEVATION_AXIS, 0x00007595, 0x00000000, 0x6077, 0, &m_offro_act_torq[ELEVATION_AXIS]},    //!< Actual torque

                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x6040, 0, &m_offrw_ctrl[AZIMUTH_AXIS]},          //!< Control word
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x6041, 0, &m_offro_status[AZIMUTH_AXIS]},        //!< Status word
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x607A, 0, &m_offrw_tgt_pos[AZIMUTH_AXIS]},       //!< Target position
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x6062, 0, &m_offro_dmd_pos[AZIMUTH_AXIS]},       //!< Demand position
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x6064, 0, &m_offro_act_pos[AZIMUTH_AXIS]},       //!< Actual position
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x60FF, 0, &m_offrw_tgt_vel[AZIMUTH_AXIS]},       //!< Target velocity
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x606B, 0, &m_offro_dmd_vel[AZIMUTH_AXIS]},       //!< Demand velocity
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x606C, 0, &m_offro_act_vel[AZIMUTH_AXIS]},       //!< Actual velocity
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x6081, 0, &m_offrw_prof_vel[AZIMUTH_AXIS]},      //!< Profile velocity
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x6060, 0, &m_offrw_act_mode[AZIMUTH_AXIS]},      //!< Actual drive mode of operation
                {0, AZIMUTH_AXIS,   0x00007595, 0x00000000, 0x6077, 0, &m_offro_act_torq[AZIMUTH_AXIS]},      //!< Actual torque

                {}
            };

            // Регистируем PDO в домене
            if (ecrt_domain_reg_pdo_entry_list(m_domain, kDomainPDOs)) {
                BOOST_THROW_EXCEPTION(Exception("PDO entries registration failed"));
            }

            LOG_INFO("PDO entries registered in domain");

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
            s.azimuth.state = AxisState::AXIS_INIT;
            s.elevation.state = AxisState::AXIS_INIT;
            m_sys_status.store(s);

            for (uint32_t d = AXIS_MIN; d < AXIS_COUNT; ++d) {
                m_tx_requested[d].store(false, std::memory_order_relaxed);
            }
        } catch (const std::exception& ex) {
            LOG_ERROR(ex.what());

            // Записываем сотояние системы
            SystemStatus s;
            s.state = SystemState::SYSTEM_ERROR;
            // @todo Возвращать строку ошибки
            // s.error_str = ex.what();
            m_sys_status.store(s);
        }
    }

    void SetModeRun(const Axis& axis, int32_t pos, int32_t vel) {
        if (axis == AZIMUTH_AXIS) {
            if (vel) {
                // profile velocity mode for azimuth drive

                TXValues tx_values;
                tx_values.controlword = 0xF;
                tx_values.op_mode = 3;
                tx_values.target_vel = vel;
                tx_values.target_pos = 0;

                m_tx_values[AZIMUTH_AXIS].store(tx_values);
                m_tx_requested[AZIMUTH_AXIS].store(true, std::memory_order_release);
            } else {
                // profile positon mode for azimuth drive

                TXValues tx_values;
                tx_values.controlword = 0xF;
                tx_values.op_mode = 1;
                tx_values.target_vel = 0;
                tx_values.target_pos = pos % kPositionMaxValue;

                m_tx_values[AZIMUTH_AXIS].store(tx_values);
                m_tx_requested[AZIMUTH_AXIS].store(true, std::memory_order_release);
            }
        } else if (axis == ELEVATION_AXIS) {
            if (vel) {
                // profile velocity mode for elevation drive

                TXValues tx_values;
                tx_values.controlword = 0xF;
                tx_values.op_mode = 3;
                tx_values.target_vel = vel;
                tx_values.target_pos = 0;

                m_tx_values[ELEVATION_AXIS].store(tx_values);
                m_tx_requested[ELEVATION_AXIS].store(true, std::memory_order_release);
            } else {
                // profile positon mode for elevation drive

                TXValues tx_values;
                tx_values.controlword = 0xF;
                tx_values.op_mode = 1;
                tx_values.target_vel = 0;
                tx_values.target_pos = pos % kPositionMaxValue;

                m_tx_values[ELEVATION_AXIS].store(tx_values);
                m_tx_requested[ELEVATION_AXIS].store(true, std::memory_order_release);
            }
        }
    }

    void SetModeIdle(const Axis& axis) {
        if (axis == AZIMUTH_AXIS) {
            TXValues tx_values;
            tx_values.controlword = 0x6;
            tx_values.op_mode = 0;
            tx_values.target_vel = 0;
            tx_values.target_pos = 0;

            m_tx_values[AZIMUTH_AXIS].store(tx_values);
            m_tx_requested[AZIMUTH_AXIS].store(true, std::memory_order_release);
        } else if (axis == ELEVATION_AXIS) {
            TXValues tx_values;
            tx_values.controlword = 0x6;
            tx_values.op_mode = 0;
            tx_values.target_vel = 0;
            tx_values.target_pos = 0;

            m_tx_values[ELEVATION_AXIS].store(tx_values);
            m_tx_requested[ELEVATION_AXIS].store(true, std::memory_order_release);
        }
    }

    const std::atomic<SystemStatus>& GetStatus() const {
        return m_sys_status;
    }

    const std::atomic<SystemInfo>& GetSystemInfo() const {
        return m_sys_info;
    }

    void CyclicPolling() {
        bool op_state = false;
        uint64_t cycle_cnt = 0;

        while (! op_state && ! m_stop_flag.load(std::memory_order_consume)) {
            // Получаем данные от подчиненных
            ecrt_master_receive(m_master);
            ecrt_domain_process(m_domain);

            ++cycle_cnt;

            // Получаем статус домена
            ecrt_domain_state(m_domain, &m_domain_state);

            // Получаем статус подчиненных
            ec_slave_config_state_t slave_cfg_state[AXIS_COUNT];
            for (uint32_t d = 0; d < AXIS_COUNT; ++d) {
                ecrt_slave_config_state(m_slave_cfg[d], &slave_cfg_state[d]);
            }

            const bool all_slaves_up = std::all_of(&slave_cfg_state[0], &slave_cfg_state[AXIS_COUNT], [](const ec_slave_config_state_t& scs) -> bool {
                return scs.operational;
            });

           if (m_domain_state.wc_state == EC_WC_COMPLETE && all_slaves_up) {
              LOG_INFO("Domain is up at " << cycle_cnt << " cycles");
              op_state = true;
           } else if (cycle_cnt % 10000 == 0) {
               LOG_WARN("Domain is NOT up at " << cycle_cnt << " cycles. Domain state=" << m_domain_state.wc_state);
           }

           // Send queued data
           ecrt_domain_queue(m_domain);     // Помечаем данные как готовые к отправке
           ecrt_master_send(m_master);      // Отправляем все датаграммы, помещенные в очередь

           std::this_thread::sleep_for(std::chrono::microseconds(kCyclePollingSleepUs));
        }

        // Устанавливаем статус системы в IDLE
        SystemStatus s;
        s.state = SystemState::SYSTEM_OK;
        s.azimuth.state = AxisState::AXIS_IDLE;
        s.elevation.state = AxisState::AXIS_IDLE;
        m_sys_status.store(s);

        cycle_cnt = 0;
        while (! m_stop_flag.load(std::memory_order_consume)) {
            // Получаем данные от подчиненных
            ecrt_master_receive(m_master);
            ecrt_domain_process(m_domain);

            // Обрабатываем пришедшие данные
            process_received_data();

            // Если есть новые команды - передаем их подчиненным
            prepare_new_commands();

            // Отправляем данные подчиненным
            ecrt_domain_queue(m_domain);
            ecrt_master_send(m_master);

            ++cycle_cnt;
            if (cycle_cnt % 10000 == 0) {
                LOG_DEBUG("Cycle count = " << cycle_cnt);
            }

            std::this_thread::sleep_for(std::chrono::microseconds(kCyclePollingSleepUs));
        }

        ecrt_master_receive(m_master);
        ecrt_domain_process(m_domain);
    }

private:
    void process_received_data() {
        SystemStatus sys;

        // Читаем данные для азимутального двигателя
        sys.azimuth.cur_pos = EC_READ_S32(m_domain_data + m_offro_act_pos[AZIMUTH_AXIS]) % kPositionMaxValue;
        sys.azimuth.tgt_pos = EC_READ_S32(m_domain_data + m_offrw_tgt_pos[AZIMUTH_AXIS]) % kPositionMaxValue;
        sys.azimuth.dmd_pos = EC_READ_S32(m_domain_data + m_offro_dmd_pos[AZIMUTH_AXIS]) % kPositionMaxValue;
        sys.azimuth.cur_vel = EC_READ_S32(m_domain_data + m_offro_act_vel[AZIMUTH_AXIS]);
        sys.azimuth.tgt_vel = EC_READ_S32(m_domain_data + m_offrw_tgt_vel[AZIMUTH_AXIS]);
        sys.azimuth.dmd_vel = EC_READ_S32(m_domain_data + m_offro_dmd_vel[AZIMUTH_AXIS]);
        sys.azimuth.cur_torq = EC_READ_S16(m_domain_data + m_offro_act_torq[AZIMUTH_AXIS]);
        sys.azimuth.statusword = EC_READ_U16(m_domain_data + m_offro_status[AZIMUTH_AXIS]);
        sys.azimuth.mode = EC_READ_S8(m_domain_data + m_offrw_act_mode[AZIMUTH_AXIS]);
        if (sys.azimuth.mode == 1) {
            sys.azimuth.state = AxisState::AXIS_POINT;
        } else if (sys.azimuth.mode == 3) {
            sys.azimuth.state = AxisState::AXIS_SCAN;
        } else if (sys.azimuth.mode == 0) {
            sys.azimuth.state = AxisState::AXIS_IDLE;
        } else {
            sys.azimuth.state = AxisState::AXIS_ERROR;
        }
        //! @todo Читать из регистра 0x603F
        sys.azimuth.error_code = 0;

        // Читаем данные для угломестного двигателя
        sys.elevation.cur_pos = EC_READ_S32(m_domain_data + m_offro_act_pos[ELEVATION_AXIS]) % kPositionMaxValue;
        sys.elevation.tgt_pos = EC_READ_S32(m_domain_data + m_offrw_tgt_pos[ELEVATION_AXIS]) % kPositionMaxValue;
        sys.elevation.dmd_pos = EC_READ_S32(m_domain_data + m_offro_dmd_pos[ELEVATION_AXIS]) % kPositionMaxValue;
        sys.elevation.cur_vel = EC_READ_S32(m_domain_data + m_offro_act_vel[ELEVATION_AXIS]);
        sys.elevation.tgt_vel = EC_READ_S32(m_domain_data + m_offrw_tgt_vel[ELEVATION_AXIS]);
        sys.elevation.dmd_vel = EC_READ_S32(m_domain_data + m_offro_dmd_vel[ELEVATION_AXIS]);
        sys.elevation.cur_torq = EC_READ_S16(m_domain_data + m_offro_act_torq[ELEVATION_AXIS]);
        sys.elevation.statusword = EC_READ_U16(m_domain_data + m_offro_status[ELEVATION_AXIS]);
        sys.elevation.mode = EC_READ_S8(m_domain_data + m_offrw_act_mode[ELEVATION_AXIS]);
        if (sys.elevation.mode == 1) {
            sys.elevation.state = AxisState::AXIS_POINT;
        } else if (sys.elevation.mode == 3) {
            sys.elevation.state = AxisState::AXIS_SCAN;
        } else if (sys.elevation.mode == 0) {
            sys.elevation.state = AxisState::AXIS_IDLE;
        } else {
            sys.elevation.state = AxisState::AXIS_ERROR;
        }
        //! @todo Читать из регистра 0x603F
        sys.elevation.error_code = 0;

        //! @todo Выставлять исходя из состояний двигателей
        sys.state = SystemState::SYSTEM_OK;

        m_sys_status.store(sys, std::memory_order_relaxed);
    }

    void prepare_new_commands() {
        if (m_tx_requested[AZIMUTH_AXIS].load(std::memory_order_acquire)) {
            const TXValues txv = m_tx_values[AZIMUTH_AXIS].load(std::memory_order_relaxed);
            if (txv.op_mode == 0) {
                EC_WRITE_U8(m_domain_data + m_offrw_act_mode[AZIMUTH_AXIS], txv.op_mode);
                EC_WRITE_U16(m_domain_data + m_offrw_ctrl[AZIMUTH_AXIS], txv.controlword);
            } else if (txv.op_mode == 1) {
                EC_WRITE_U8(m_domain_data + m_offrw_act_mode[AZIMUTH_AXIS], txv.op_mode);
                EC_WRITE_U16(m_domain_data + m_offrw_ctrl[AZIMUTH_AXIS], txv.controlword);
                EC_WRITE_S32(m_domain_data + m_offrw_tgt_pos[AZIMUTH_AXIS], txv.target_pos);
            } else if (txv.op_mode == 3) {
                EC_WRITE_U8(m_domain_data + m_offrw_act_mode[AZIMUTH_AXIS], txv.op_mode);
                EC_WRITE_U16(m_domain_data + m_offrw_ctrl[AZIMUTH_AXIS], txv.controlword);
                EC_WRITE_U8(m_domain_data + m_offrw_tgt_vel[AZIMUTH_AXIS], txv.target_vel);
            }

            m_tx_requested[AZIMUTH_AXIS].store(false, std::memory_order_relaxed);
        }

        if (m_tx_requested[ELEVATION_AXIS].load(std::memory_order_acquire)) {
            const TXValues txv = m_tx_values[ELEVATION_AXIS].load(std::memory_order_relaxed);
            if (txv.op_mode == 0) {
                EC_WRITE_U8(m_domain_data + m_offrw_act_mode[ELEVATION_AXIS], txv.op_mode);
                EC_WRITE_U16(m_domain_data + m_offrw_ctrl[ELEVATION_AXIS], txv.controlword);
            } else if (txv.op_mode == 1) {
                EC_WRITE_U8(m_domain_data + m_offrw_act_mode[ELEVATION_AXIS], txv.op_mode);
                EC_WRITE_U16(m_domain_data + m_offrw_ctrl[ELEVATION_AXIS], txv.controlword);
                EC_WRITE_S32(m_domain_data + m_offrw_tgt_pos[ELEVATION_AXIS], txv.target_pos);
            } else if (txv.op_mode == 3) {
                EC_WRITE_U8(m_domain_data + m_offrw_act_mode[ELEVATION_AXIS], txv.op_mode);
                EC_WRITE_U16(m_domain_data + m_offrw_ctrl[ELEVATION_AXIS], txv.controlword);
                EC_WRITE_U8(m_domain_data + m_offrw_tgt_vel[ELEVATION_AXIS], txv.target_vel);
            }

            m_tx_requested[ELEVATION_AXIS].store(false, std::memory_order_relaxed);
        }
    }

    struct TXValues {
        TXValues() noexcept
            : target_pos(0)
            , target_vel(0)
            , controlword(0)
            , op_mode(0)
        {}

        int32_t target_pos;
        int32_t target_vel;
        uint16_t controlword;
        uint8_t op_mode;
    };

    fs::path                        m_cfg_path;     //!< Путь к файлу конфигурации
    std::map<uint16_t, int64_t>     m_sdo_cfg;      //!< Конфигурация SDO
    std::atomic<SystemInfo>         m_sys_info;     //!< Структура с статической информацией о системе
    std::atomic<SystemStatus>       m_sys_status;   //!< Структура с динамической информацией о системе

    //! Данные для взаимодействия с потоком циклического взаимодействия с сервоусилителями
    std::atomic<bool>               m_stop_flag;    //!< Флаг остановки потока взаимодействия
    std::unique_ptr<std::thread>    m_thread;       //!< Поток циклического обмена данными со сервоусилителями
    std::atomic<TXValues>           m_tx_values[AXIS_COUNT];
    std::atomic<bool>               m_tx_requested[AXIS_COUNT];

    //! Структуры для обмена данными по EtherCAT
    ec_master_t*                    m_master = nullptr;
    ec_master_state_t               m_master_state = {};
    ec_domain_t*                    m_domain = nullptr;
    ec_domain_state_t               m_domain_state = {};
    uint8_t*                        m_domain_data = nullptr;
    ec_slave_config_t*              m_slave_cfg[AXIS_COUNT];

    static const uint32_t           kCyclePollingSleepUs;
    static const uint32_t           kRegPerDriveCount;
    static const uint32_t           kPositionMaxValue;

    uint32_t                        m_offrw_ctrl[AXIS_COUNT];
    uint32_t                        m_offro_status[AXIS_COUNT];
    uint32_t                        m_offrw_tgt_pos[AXIS_COUNT];
    uint32_t                        m_offro_dmd_pos[AXIS_COUNT];
    uint32_t                        m_offro_act_pos[AXIS_COUNT];
    uint32_t                        m_offrw_tgt_vel[AXIS_COUNT];
    uint32_t                        m_offro_dmd_vel[AXIS_COUNT];
    uint32_t                        m_offro_act_vel[AXIS_COUNT];
    uint32_t                        m_offrw_prof_vel[AXIS_COUNT];
    uint32_t                        m_offrw_act_mode[AXIS_COUNT];
    uint32_t                        m_offro_act_torq[AXIS_COUNT];
 };

const uint32_t Control::Impl::kCyclePollingSleepUs = 800;
const uint32_t Control::Impl::kRegPerDriveCount = 12;
const uint32_t Control::Impl::kPositionMaxValue = std::pow(2, 20);

Control::Control(const char* cfg_file_path)
    : m_pimpl(new Control::Impl(cfg_file_path))
{}

Control::~Control() {
}

void Control::SetModeRun(const Axis& axis, int32_t pos, int32_t vel) {
    m_pimpl->SetModeRun(axis, pos, vel);
}

void Control::SetModeIdle(const Axis& axis) {
    m_pimpl->SetModeIdle(axis);
}

const std::atomic<SystemStatus>& Control::GetStatus() const {
    return m_pimpl->GetStatus();
}

const std::atomic<SystemInfo>& Control::GetSystemInfo() const {
    return m_pimpl->GetSystemInfo();
}

} // namespaces
