#include <cstdint>
#include <cmath>
#include <thread>
#include <chrono>
#include <functional>
#include <algorithm>
#include <atomic>
#include <map>

#include <boost/filesystem/path.hpp>
#include <boost/lockfree/spsc_queue.hpp>

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

    Impl(const std::string& cfg_file_path)
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
            s.axes[AZIMUTH_AXIS].state = AxisState::AXIS_INIT;
            s.axes[ELEVATION_AXIS].state = AxisState::AXIS_INIT;
            m_sys_status.store(s);
        } catch (const std::exception& ex) {
            LOG_ERROR(ex.what());

            // Записываем сотояние системы
            SystemStatus s = m_sys_status.load(std::memory_order_acquire);
            s.state = SystemState::SYSTEM_ERROR;
            // @todo Возвращать строку ошибки
            // s.error_str = ex.what();
            m_sys_status.store(s);
        }
    }

    void SetModeRun(const Axis& axis, int32_t pos, int32_t vel) {
        if (vel) {
            TXCmd txcmd;

            // Переходим в режим "Profile velocity mode" и сразу задаем требуемую скорость
            txcmd.controlword = 0xF;
            txcmd.op_mode = 3;
            txcmd.target_vel = vel;
            txcmd.target_pos = 0;
            m_tx_queues[axis].push(txcmd);
        } else {
            TXCmd txcmd;

            // Переходим в режим "Profile position mode"
            txcmd.controlword = 0xF;
            txcmd.op_mode = 1;
            txcmd.target_vel = 0;
            txcmd.target_pos = 0;
            m_tx_queues[axis].push(txcmd);

            // Задаем следующую точку для позиционирования
            txcmd.controlword = 0x11F;
            txcmd.target_pos = pos % kPositionMaxValue;
            m_tx_queues[axis].push(txcmd);
        }
    }

    void SetModeIdle(const Axis& axis) {
        TXCmd txcmd;
        txcmd.controlword = 0x6;
        txcmd.op_mode = 0;
        txcmd.target_vel = 0;
        txcmd.target_pos = 0;

        m_tx_queues[axis].push(txcmd);
    }

    const std::atomic<SystemStatus>& GetStatus() const {
        return m_sys_status;
    }

    const std::atomic<SystemInfo>& GetSystemInfo() const {
        return m_sys_info;
    }

    void CyclicPolling() {
        bool op_state = false;
        uint64_t cycles_cur = 0;

        while (! op_state && ! m_stop_flag.load(std::memory_order_consume)) {
            // Получаем данные от подчиненных
            ecrt_master_receive(m_master);
            ecrt_domain_process(m_domain);

            ++cycles_cur;

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
              LOG_INFO("Domain is up at " << cycles_cur << " cycles");
              op_state = true;
           } else if (cycles_cur % 10000 == 0) {
               //! @todo выход из цикла и сообщение об ошибке
               LOG_WARN("Domain is NOT up at " << cycles_cur << " cycles. Domain state=" << m_domain_state.wc_state);
           }

           // Send queued data
           ecrt_domain_queue(m_domain);     // Помечаем данные как готовые к отправке
           ecrt_master_send(m_master);      // Отправляем все датаграммы, помещенные в очередь

           std::this_thread::sleep_for(std::chrono::microseconds(kCyclePollingSleepUs));
        }

        // Устанавливаем статус системы в IDLE
        SystemStatus s = m_sys_status.load(std::memory_order_acquire);;
        s.state = SystemState::SYSTEM_OK;
        s.axes[AZIMUTH_AXIS].state = AxisState::AXIS_IDLE;
        s.axes[ELEVATION_AXIS].state = AxisState::AXIS_IDLE;
        m_sys_status.store(s);

        cycles_cur = 0;
        while (! m_stop_flag.load(std::memory_order_consume)) {
            // Получаем данные от подчиненных
            ecrt_master_receive(m_master);
            ecrt_domain_process(m_domain);

            // Получаем статус домена
            ecrt_domain_state(m_domain, &m_domain_state);

            // Обрабатываем пришедшие данные
            process_received_data();

            // Если есть новые команды - передаем их подчиненным
            prepare_new_commands();

            // Отправляем данные подчиненным
            ecrt_domain_queue(m_domain);
            ecrt_master_send(m_master);

            ++cycles_cur;
            if (cycles_cur % 10000 == 0) {
                // LOG_DEBUG("Cycle count = " << cycles_cur);
            }

            std::this_thread::sleep_for(std::chrono::microseconds(kCyclePollingSleepUs));
        }

        ecrt_master_receive(m_master);
        ecrt_domain_process(m_domain);
    }

private:
    void process_received_data() {
        SystemStatus sys = m_sys_status.load(std::memory_order_acquire);

        for (int32_t axis = AXIS_MIN; axis < AXIS_COUNT; ++axis) {
            // Читаем данные для двигателя c индексом axis
            sys.axes[axis].cur_pos = EC_READ_S32(m_domain_data + m_offro_act_pos[axis]) % kPositionMaxValue;
            sys.axes[axis].tgt_pos = EC_READ_S32(m_domain_data + m_offrw_tgt_pos[axis]) % kPositionMaxValue;
            sys.axes[axis].dmd_pos = EC_READ_S32(m_domain_data + m_offro_dmd_pos[axis]) % kPositionMaxValue;
            sys.axes[axis].cur_vel = EC_READ_S32(m_domain_data + m_offro_act_vel[axis]);
            sys.axes[axis].tgt_vel = EC_READ_S32(m_domain_data + m_offrw_tgt_vel[axis]);
            sys.axes[axis].dmd_vel = EC_READ_S32(m_domain_data + m_offro_dmd_vel[axis]);
            sys.axes[axis].cur_torq = EC_READ_S16(m_domain_data + m_offro_act_torq[axis]);
            sys.axes[axis].ctrlword = EC_READ_U16(m_domain_data + m_offrw_ctrl[axis]);
            sys.axes[axis].statusword = EC_READ_U16(m_domain_data + m_offro_status[axis]);
            sys.axes[axis].mode = EC_READ_S8(m_domain_data + m_offrw_act_mode[axis]);
            if (sys.axes[axis].mode == 1) {
                sys.axes[axis].state = AxisState::AXIS_POINT;
            } else if (sys.axes[axis].mode == 3) {
                sys.axes[axis].state = AxisState::AXIS_SCAN;
            } else if (sys.axes[axis].mode == 0) {
                sys.axes[axis].state = AxisState::AXIS_IDLE;
            } else {
                sys.axes[axis].state = AxisState::AXIS_ERROR;
            }
            //! @todo Читать из регистра 0x603F
            sys.axes[axis].error_code = 0;
        }

        //! @todo Выставлять исходя из состояний двигателей
        if (sys.state == SystemState::SYSTEM_OK) {
            sys.state = SystemState::SYSTEM_OK;
        }

        m_sys_status.store(sys, std::memory_order_relaxed);
    }

    void prepare_new_commands() {
        static uint64_t cycles_cur = 0;
        static uint64_t cycles_cmd_start[AXIS_COUNT] = {0};

        TXCmd txcmd;
        const SystemStatus sys = m_sys_status.load(std::memory_order_acquire);

        for (int32_t axis = AXIS_MIN; axis < AXIS_COUNT; ++axis) {
            if (sys.axes[axis].mode == 1) {
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

            if (m_tx_queues[axis].pop(txcmd)) {
                if (txcmd.op_mode == 0) {
                    EC_WRITE_U8(m_domain_data + m_offrw_act_mode[axis], txcmd.op_mode);
                    EC_WRITE_U16(m_domain_data + m_offrw_ctrl[axis], txcmd.controlword);
                } else if (txcmd.op_mode == 1) {
                    EC_WRITE_U8(m_domain_data + m_offrw_act_mode[axis], txcmd.op_mode);
                    EC_WRITE_U16(m_domain_data + m_offrw_ctrl[axis], txcmd.controlword);
                    EC_WRITE_S32(m_domain_data + m_offrw_tgt_pos[axis], txcmd.target_pos);
                    EC_WRITE_U32(m_domain_data + m_offrw_prof_vel[axis], 100000);

                    cycles_cmd_start[axis] = cycles_cur;
                } else if (txcmd.op_mode == 3) {
                    EC_WRITE_U8(m_domain_data + m_offrw_act_mode[axis], txcmd.op_mode);
                    EC_WRITE_U16(m_domain_data + m_offrw_ctrl[axis], txcmd.controlword);
                    EC_WRITE_S32(m_domain_data + m_offrw_tgt_vel[axis], txcmd.target_vel);
                }
            }
        }

        ++cycles_cur;
    }

    struct TXCmd {
        TXCmd() noexcept
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

    constexpr static uint32_t       kCmdQueueCapacity = 128;
    typedef boost::lockfree::spsc_queue<TXCmd, boost::lockfree::capacity<kCmdQueueCapacity>> TXCmdQueue;
    TXCmdQueue                      m_tx_queues[AXIS_COUNT]; //!< Очереди команд по осям

    //! Структуры для обмена данными по EtherCAT
    ec_master_t*                    m_master = nullptr;
    ec_master_state_t               m_master_state = {};
    ec_domain_t*                    m_domain = nullptr;
    ec_domain_state_t               m_domain_state = {};
    uint8_t*                        m_domain_data = nullptr;
    ec_slave_config_t*              m_slave_cfg[AXIS_COUNT];

    constexpr static uint32_t       kCyclePollingSleepUs = 800;
    constexpr static uint32_t       kRegPerDriveCount = 12;
    constexpr static uint32_t       kPositionMaxValue = std::pow(2, 20);
    constexpr static uint64_t       kMaxAxisReadyCycles = 8192;
    constexpr static uint64_t       kMaxDomainInitCycles = 8192;

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

constexpr uint32_t Control::Impl::kCyclePollingSleepUs;
constexpr uint32_t Control::Impl::kRegPerDriveCount;
constexpr uint32_t Control::Impl::kPositionMaxValue;
constexpr uint64_t Control::Impl::kMaxAxisReadyCycles;
constexpr uint64_t Control::Impl::kMaxDomainInitCycles;

Control::Control(const std::string &cfg_file_path)
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
