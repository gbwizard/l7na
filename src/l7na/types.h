#pragma once

#include <cstdint>

#include <map>
#include <string>
#include <vector>
#include <limits>

namespace Drives {

//! @brief Индекс двигателя
enum Axis: int32_t {
    AXIS_NONE = -1,

    AXIS_MIN = 0,

    AZIMUTH_AXIS = 0,
    ELEVATION_AXIS = 1,

    AXIS_COUNT
};

//! @brief Возможные состояния каждого двигателя
enum AxisState : int32_t {
    AXIS_OFF = -1,                  //!< Выключен (до вызова Init или после вызова Release)
    AXIS_INIT,                      //!< Инициализируется
    AXIS_IDLE,                      //!< Включен, готов к работе
    AXIS_SCAN,                      //!< Работает в режиме "Сканирование"
    AXIS_POINT,                     //!< Работает в режиме "Установка в точку"
    AXIS_ERROR                      //!< Состояние ошибки
};

//! @brief Режим установки параметров осей
enum ParamsMode : int16_t {
    PARAMS_MODE_AUTOMATIC,
    PARAMS_MODE_MANUAL
};

/*! @brief Режим движения, соответствующий определенному набору параметров оси.
 *
 *  Cоответствует максимальному расстоянию в градусах между текущей и запрашиваемой позицией.
 *  Для режима AXIS_SCAN используется режим с максимальным идентификатором.
 */
using MoveMode = uint16_t;

//! @brief Текущие значения для одной оси системы
struct AxisStatus {
    AxisStatus();

    bool IsReady() const;

    double      tgt_pos_deg;            //!< Целевая позиция [градусы]
    double      cur_pos_deg;            //!< Текущая позиция [градусы]
    double      dmd_pos_deg;            //!< Запрашиваемая позиция [градусы]
    double      tgt_vel_deg;            //!< Целевая скорость [градусы/с]
    double      cur_vel_deg;            //!< Текущая скорость [градусы/с]
    double      dmd_vel_deg;            //!< Запрашиваемая скорость [импульсы энкодера/c]
    int32_t     cur_pos_abs;            //!< Текущая позиция абсолютная [импульсы энкодера]
    int32_t     cur_pos;                //!< Текущая позиция [импульсы энкодера]
    int32_t     dmd_pos;                //!< Запрашиваемая позиция [импульсы энкодера]
    int32_t     tgt_pos;                //!< Целевая позиция [импульсы энкодера]
    int32_t     cur_vel;                //!< Текущая скорость [импульсы энкодера/с]
    int32_t     dmd_vel;                //!< Запрашиваемая скорость [импульсы энкодера/c]
    int32_t     tgt_vel;                //!< Целевая скорость [импульсы энкодера/с]
    int32_t     cur_torq;               //!< Текущий момент [единиц 0,1% от _номинального_ момента двигателя]
    AxisState   state;                  //!< Текущее состояние системы управления осью
    uint32_t    error_code;             //!< Код ошибки двигателя по CiA402
    int32_t     cur_temperature;        //!< Текущая температура для сервоусилителя
    uint16_t    ctrlword;               //!< Битовая маска управления приводом (для отладки)
    uint16_t    statusword;             //!< Битовая маска текущего состояния привода (для отладки)
    uint16_t    mode;                   //!< Текущий режим работы (для отладки)
    MoveMode    move_mode;
    ParamsMode  params_mode;
};

enum SystemState : int32_t {
    SYSTEM_OFF = -1,
    SYSTEM_OK = 0,
    SYSTEM_INIT,
    SYSTEM_ERROR,
    SYSTEM_FATAL_ERROR,
};

//! @brief Текущие значения, возвращаемые системой управления
struct SystemStatus {
    SystemStatus() noexcept;

    AxisStatus axes[AXIS_COUNT];    //!< Статус двигателей по осям
    SystemState state;              //!< Состояние системы
    uint64_t reftime;               //!< Время привязки координат в системном времени [наносекунды с начала Epoch]
    uint64_t apptime;               //!< Текущее системное время [наносекунды с начала Epoch]
    uint32_t dcsync;                //!< Оценка сверху разницы во времени между хостом и двигателем [наносекунды]
};

//! @brief Статическая информация для одной оси. Заполняется один раз при инициализации.
struct AxisInfo {
    AxisInfo()
        : encoder_resolution(0)
        , dev_name()
        , hw_version()
        , sw_version()
    {}

    uint16_t        encoder_resolution; //!< Разрешение энкодера
    std::string     dev_name;           //!< Название устройства
    std::string     hw_version;         //!< Версия аппаратного обепечения
    std::string     sw_version;         //!< Версия программного обеспечения
};

//! @brief Статическая информация для системы
struct SystemInfo {
    SystemInfo()
    {}

    AxisInfo axes[AXIS_COUNT];
};

//! @brief Параметры настройки оси
struct AxisParam {
    uint16_t index;
    int64_t value;
};

struct CycleTimeInfo {
    uint64_t period_ns;
    uint64_t exec_ns;
    uint64_t latency_ns;
    uint64_t latency_min_ns;
    uint64_t latency_max_ns;
    uint64_t period_min_ns;
    uint64_t period_max_ns;
    uint64_t exec_min_ns;
    uint64_t exec_max_ns;

    CycleTimeInfo() noexcept
        : period_ns(0)
        , exec_ns(0)
        , latency_ns(0)
        , latency_min_ns(std::numeric_limits<decltype(latency_min_ns)>::max())
        , latency_max_ns(0)
        , period_min_ns(std::numeric_limits<decltype(period_min_ns)>::max())
        , period_max_ns(0)
        , exec_min_ns(std::numeric_limits<decltype(exec_min_ns)>::max())
        , exec_max_ns(0)
    {}
};

using AxisParams = std::vector<AxisParam>;
using AxisParamIndexMap = std::map<uint16_t, uint16_t>;
using AxisParamValueMap = std::map<uint16_t, int64_t>;

} // namespaces
