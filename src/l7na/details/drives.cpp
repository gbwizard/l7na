#include <thread>
#include <map>

#include <boost/filesystem/path.hpp>

#include "ecrt.h"

#include "l7na/drives.h"

namespace Drives {

namespace fs = boost::filesystem;

class Control::Impl {
public:
    ~Impl() {

    }

protected:
    friend class Control;

    Impl(const char* cfg_file_path)
        : m_cfg_path (cfg_file_path)
    {

    }

    void SetModeRun(int32_t azimuth_angle, int32_t azimuth_velocity, int32_t elevation_angle, int32_t elevation_velocity) {
        ;
    }

    void SetModeIdle() {
        ;
    }

    const SystemStatus& GetStatus() const {
        return m_sys_status;
    }

    const SystemInfo& GetSystemInfo() const {
        return m_sys_info;
    }

private:
    fs::path                        m_cfg_path;     //!< Путь к файлу конфигурации
    std::map<uint16_t, uint32_t>    m_sdo_cfg;      //!< Конфигурация SDO
    std::unique_ptr<std::thread>    m_thread;       //!< Поток циклического обмена данными со сервоусилителями
    SystemInfo                      m_sys_info;     //!< Структура с статической информацией о системе
    SystemStatus                    m_sys_status;   //!< Структура с динамической информацией о системе
 };

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

const SystemStatus& Control::GetStatus() const {
    return m_pimpl->GetStatus();
}

const SystemInfo& Control::GetSystemInfo() const {
    return m_pimpl->GetSystemInfo();
}

} // namespaces
