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
        ecrt_release_master(m_master);

        LOG_INFO("Master released");
    }

protected:
    friend class Control;

    Impl(const char* cfg_file_path)
        : m_cfg_path (cfg_file_path)
        , m_sdo_cfg()
        , m_sys_info{SystemInfo()}
        , m_sys_status{SystemStatus()}
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
            for (uint32_t d = 0; d < DRIVE_COUNT; ++d) {
                m_slave_cfg[d] = ecrt_master_slave_config(m_master, 0, d, 0x00007595, 0x00000000);
            }
            const bool all_slave_configs_ok = std::all_of(&m_slave_cfg[0], &m_slave_cfg[DRIVE_COUNT], [](const ec_slave_config_t* const sc) -> bool {
                return sc;
            });

            if (all_slave_configs_ok) {
                LOG_INFO("Slave configuration objects (" << DRIVE_COUNT << ") created");
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
            ec_pdo_entry_info_t l7na_rx_channel1[] = {
                {0x6040, 0, 16},    // Controlword
                {0x6060, 0, 8},     // Actual mode of operation
                {0x607A, 0, 32},    // Target position
                {0x60FF, 0, 32},    // Target velocity
                {0x6081, 0, 32},    // Profile velocity
            };

            ec_pdo_info_t l7na_rx_pdos[] = {
                {0x1600, 5, l7na_rx_channel1}
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
            for (uint32_t d = 0; d < DRIVE_COUNT; ++d) {
                if (ecrt_slave_config_pdos(m_slave_cfg[d], EC_END, l7na_syncs)) {
                    BOOST_THROW_EXCEPTION(Exception("Failed to configure slave #") << d << " pdos");
                }
            }

            LOG_INFO("Configuring slave PDOs and sync managers done");

            static const ec_pdo_entry_reg_t kDomainPDOs[] = {
                {0, ELEVATION_DRIVE, 0x00007595, 0x00000000, 0x6040, 0, &m_offrw_ctrl[ELEVATION_DRIVE]},        //!< Control word
                {0, ELEVATION_DRIVE, 0x00007595, 0x00000000, 0x6041, 0, &m_offro_status[ELEVATION_DRIVE]},      //!< Status word
                {0, ELEVATION_DRIVE, 0x00007595, 0x00000000, 0x607A, 0, &m_offrw_tgt_pos[ELEVATION_DRIVE]},     //!< Target position
                {0, ELEVATION_DRIVE, 0x00007595, 0x00000000, 0x6062, 0, &m_offro_dmd_pos[ELEVATION_DRIVE]},     //!< Demand position
                {0, ELEVATION_DRIVE, 0x00007595, 0x00000000, 0x6064, 0, &m_offro_act_pos[ELEVATION_DRIVE]},     //!< Actual position
                {0, ELEVATION_DRIVE, 0x00007595, 0x00000000, 0x60FF, 0, &m_offrw_tgt_vel[ELEVATION_DRIVE]},     //!< Target velocity
                {0, ELEVATION_DRIVE, 0x00007595, 0x00000000, 0x606B, 0, &m_offro_dmd_vel[ELEVATION_DRIVE]},     //!< Demand velocity
                {0, ELEVATION_DRIVE, 0x00007595, 0x00000000, 0x606C, 0, &m_offro_act_vel[ELEVATION_DRIVE]},     //!< Actual velocity
                {0, ELEVATION_DRIVE, 0x00007595, 0x00000000, 0x6081, 0, &m_offrw_prof_vel[ELEVATION_DRIVE]},    //!< Profile velocity
                {0, ELEVATION_DRIVE, 0x00007595, 0x00000000, 0x6060, 0, &m_offrw_act_mode[ELEVATION_DRIVE]},    //!< Actual drive mode of operation
                {0, ELEVATION_DRIVE, 0x00007595, 0x00000000, 0x6077, 0, &m_offro_act_torq[ELEVATION_DRIVE]},    //!< Actual torque

                {0, AZIMUTH_DRIVE,   0x00007595, 0x00000000, 0x6040, 0, &m_offrw_ctrl[AZIMUTH_DRIVE]},          //!< Control word
                {0, AZIMUTH_DRIVE,   0x00007595, 0x00000000, 0x6041, 0, &m_offro_status[AZIMUTH_DRIVE]},        //!< Status word
                {0, AZIMUTH_DRIVE,   0x00007595, 0x00000000, 0x607A, 0, &m_offrw_tgt_pos[AZIMUTH_DRIVE]},       //!< Target position
                {0, AZIMUTH_DRIVE,   0x00007595, 0x00000000, 0x6062, 0, &m_offro_dmd_pos[AZIMUTH_DRIVE]},       //!< Demand position
                {0, AZIMUTH_DRIVE,   0x00007595, 0x00000000, 0x6064, 0, &m_offro_act_pos[AZIMUTH_DRIVE]},       //!< Actual position
                {0, AZIMUTH_DRIVE,   0x00007595, 0x00000000, 0x60FF, 0, &m_offrw_tgt_vel[AZIMUTH_DRIVE]},       //!< Target velocity
                {0, AZIMUTH_DRIVE,   0x00007595, 0x00000000, 0x606B, 0, &m_offro_dmd_vel[AZIMUTH_DRIVE]},       //!< Demand velocity
                {0, AZIMUTH_DRIVE,   0x00007595, 0x00000000, 0x606C, 0, &m_offro_act_vel[AZIMUTH_DRIVE]},       //!< Actual velocity
                {0, AZIMUTH_DRIVE,   0x00007595, 0x00000000, 0x6081, 0, &m_offrw_prof_vel[AZIMUTH_DRIVE]},      //!< Profile velocity
                {0, AZIMUTH_DRIVE,   0x00007595, 0x00000000, 0x6060, 0, &m_offrw_act_mode[AZIMUTH_DRIVE]},      //!< Actual drive mode of operation
                {0, AZIMUTH_DRIVE,   0x00007595, 0x00000000, 0x6077, 0, &m_offro_act_torq[AZIMUTH_DRIVE]},      //!< Actual torque

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
        } catch (const std::exception& ex) {
            LOG_ERROR(ex.what());
            SystemStatus s;
            s.state = SYSTEM_ERROR;
            s.error_str = ex.what();
            m_sys_status.store(s);
        }
    }

    void SetModeRun(int32_t azimuth_angle, int32_t azimuth_velocity, int32_t elevation_angle, int32_t elevation_velocity) {
        ;
    }

    void SetModeIdle() {
        ;
    }

    const std::atomic<SystemStatus>& GetStatus() const {
        return m_sys_status;
    }

    const std::atomic<SystemInfo>& GetSystemInfo() const {
        return m_sys_info;
    }

    void CyclicPolling() {
        bool op_state = false;
        uint32_t cycle_cnt = 0;

        while (! op_state || m_stop_flag.load(std::memory_order_consume)) {
            // Receive data from slaves
            ecrt_master_receive(m_master);
            ecrt_domain_process(m_domain);

            ++cycle_cnt;

            // Получаем статус домена
            ecrt_domain_state(m_domain, &m_domain_state);

            // Получаем статус подчиненных
            ec_slave_config_state_t slave_cfg_state[DRIVE_COUNT];
            for (uint32_t d = 0; d < DRIVE_COUNT; ++d) {
                ecrt_slave_config_state(m_slave_cfg[d], &slave_cfg_state[d]);
            }

            const bool all_slaves_up = std::all_of(&slave_cfg_state[0], &slave_cfg_state[DRIVE_COUNT], [](const ec_slave_config_state_t& scs) -> bool {
                return scs.operational;
            });

           if (m_domain_state.wc_state == EC_WC_COMPLETE && all_slaves_up) {
              LOG_INFO("Domain is up at " << cycle_cnt << " cycles");
              op_state = true;
           } else if (cycle_cnt % 10000 == 0) {
               LOG_WARN("Domain is NOT up at " << cycle_cnt << " cycles. Domain state=" << m_domain_state.wc_state);
           }

           // Send queued data
           ecrt_domain_queue(m_domain);     // MARK THE DOMAIN DATA AS READY FOR EXCHANGE
           ecrt_master_send(m_master);      // SEND ALL QUEUED DATAGRAMS

           std::this_thread::sleep_for(std::chrono::microseconds(kCyclePollingSleepUs));
        }

        while (m_stop_flag.load(std::memory_order_consume)) {
            // Receive data from slaves
            ecrt_master_receive(m_master);
            ecrt_domain_process(m_domain);

            // Send queued data
            ecrt_domain_queue(m_domain);
            ecrt_master_send(m_master);

            std::this_thread::sleep_for(std::chrono::microseconds(kCyclePollingSleepUs));
        }

        ecrt_master_receive(m_master);
        ecrt_domain_process(m_domain);
    }

private:
    enum : uint32_t {
        ELEVATION_DRIVE = 0,
        AZIMUTH_DRIVE,

        DRIVE_COUNT
    };

    fs::path                        m_cfg_path;     //!< Путь к файлу конфигурации
    std::map<uint16_t, int64_t>     m_sdo_cfg;      //!< Конфигурация SDO
    std::atomic<SystemInfo>         m_sys_info;     //!< Структура с статической информацией о системе
    std::atomic<SystemStatus>       m_sys_status;   //!< Структура с динамической информацией о системе

    //! Данные для взаимодействия с потоком циклического взаимодействия с сервоусилителями
    std::atomic<bool>               m_stop_flag;    //!< Флаг остановки потока взаимодействия
    std::unique_ptr<std::thread>    m_thread;               //!< Поток циклического обмена данными со сервоусилителями

    //! Структуры для обмена данными по EtherCAT
    ec_master_t*                    m_master = nullptr;
    ec_master_state_t               m_master_state = {};
    ec_domain_t*                    m_domain = nullptr;
    ec_domain_state_t               m_domain_state = {};
    uint8_t*                        m_domain_data = nullptr;
    ec_slave_config_t*              m_slave_cfg[DRIVE_COUNT];

    static const uint32_t           kCyclePollingSleepUs;
    static const uint32_t           kRegPerDriveCount;

    uint32_t                        m_offrw_ctrl[DRIVE_COUNT];
    uint32_t                        m_offro_status[DRIVE_COUNT];
    uint32_t                        m_offrw_tgt_pos[DRIVE_COUNT];
    uint32_t                        m_offro_dmd_pos[DRIVE_COUNT];
    uint32_t                        m_offro_act_pos[DRIVE_COUNT];
    uint32_t                        m_offrw_tgt_vel[DRIVE_COUNT];
    uint32_t                        m_offro_dmd_vel[DRIVE_COUNT];
    uint32_t                        m_offro_act_vel[DRIVE_COUNT];
    uint32_t                        m_offrw_prof_vel[DRIVE_COUNT];
    uint32_t                        m_offrw_act_mode[DRIVE_COUNT];
    uint32_t                        m_offro_act_torq[DRIVE_COUNT];
 };

const uint32_t  Control::Impl::kCyclePollingSleepUs = 500;
const uint32_t  Control::Impl::kRegPerDriveCount = 12;

Control::Control(const char* cfg_file_path)
    : m_pimpl(new Control::Impl(cfg_file_path))
{}

Control::~Control() {
}

void Control::SetModeRun(int32_t azimuth_angle, int32_t azimuth_velocity, int32_t elevation_angle, int32_t elevation_velocity) {
    m_pimpl->SetModeRun(azimuth_angle, azimuth_velocity, elevation_angle, elevation_velocity);
}

void Control::SetModeIdle() {
    m_pimpl->SetModeIdle();
}

const std::atomic<SystemStatus>& Control::GetStatus() const {
    return m_pimpl->GetStatus();
}

const std::atomic<SystemInfo>& Control::GetSystemInfo() const {
    return m_pimpl->GetSystemInfo();
}

} // namespaces
