#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <atomic>

#include "types.h"
#include "configfile.h"

/*! @brief API системы управления двигателями метеорологической антенны ДМРЛ-3
 *
 *  Описание работы с системой управления.
 *
 *  До начала работы с системой управления ее состояние описывается константой STATE_OFF.
 *
 *  Для начала работы необходимо вызывать функцию Init(config_file_path).
 *  Синтаксис конфигурационного файла - набор строчек вида:
 *  @code{.unparsed}
 *  60F7=35     // Комментарий 1
 *  6083=20000  // Комментарий 2
 *  # Комментарий 3
 *  6084=20000
 *  ...
 *  @endcode
 *  где поле слева от '=' - адрес регистра, а справа - значение, которое туда нужно записать
 *  при инициализации системы.
 *
 *  После успешной инициализации система переходит в статут STATE_IDLE.
 *
 *  Далее возможен переход в один из основных режимов работы - сканирование по азимуту или
 *  позиционирование в точку. Для каждого режима выделена отдельная функция. Состояние системы описывается одной
 *  из констант [STATE_SCAN, STATE_POINT].
 *
 *  Получение текущих значений для обоих осей осуществляется вызов функции GetStatus,
 *  возвращающей развернутый текущий статус системы.
 *
 *  При возникновении ошибки состояние системы становится STATE_ERROR. Поле error_code для двигателя, вызвавшего ошибку
 *  установлено в соответствующее значение. Для продолжения работы необходимо сначала перевести систему в STATE_IDLE
 *  соответствующим вызовом.
 *
 *  При завершении работы необходимо из любого режима вызвать метод Release(), который остановит двигатели
 *  и произведет необходимую деинициализацию.
 */

namespace Drives {

/*! @brief Get system state name by enum value
 */
std::string GetSystemStateName(const SystemState& ss);

/*! @brief Get axis state name by enum value
 */
std::string GetAxisStateName(const AxisState& as);

/*! @brief Объект управления системой вращения.
 *
 *  Созданием объекта == подлключение к системе управления
 *
 */
class Control {
public:
    /*! @brief Ctor initializes control system
     *
     *  @param   cfg_file_path  Path to system config file (abs or relative to current working directory)
     */
    Control(const Config::Storage& config, const ParamsMode params_mode = PARAMS_MODE_AUTOMATIC);

    /*! @brief Dtor switches off the drives
     */
    ~Control();

    /*! @brief Задаем смещения координат для осей. Возможно задавать в любое время работы.
     *
     *  От этих смещений зависит значения положений в градусах, которые возвращаются пользователю
     *  и которые принимаются от пользователя в командах управления.
     *
     *  @return Флаг успешности операции.
     */
    bool SetPosAbsPulseOffset(const Axis& axis, int32_t offset);

    /*! @brief Добавляем в систему новый режим работы по параметрам.
     *
     *  @param axis     Drive identifier
     *  @param mode     Move mode identifier. It corresponds to the maximum distance in degrees between current
     *                  and new position. For 'Scan' operation mode the 'highest' identifier mode is used
     *  @param params   Drive params which are added under this identifier
     *
     *  @return         Operation success flag
     */
    bool AddMoveMode(const Axis& axis, const MoveMode& mode, const AxisParams& params);

    /*! @brief Set the mode for drive params setup
     *
     *  @param axis     Drive identifier
     *  @param params_mode   Mode of drive params setup
     *
     *  @return         Operation success flag
     */
    bool SetParamsMode(const Axis& axis, const ParamsMode& params_mode);

    /*! @brief Get the mode for drive params setup
     *
     *  @param axis     Drive identifier
     *  @param [out] params_mode   Mode of drive params setup
     *
     *  @return         Operation success flag
     */
    bool GetParamsMode(const Axis& axis, ParamsMode& params_mode) const;

    /*! @brief Set parameters for the specified axis
     *
     *  @param axis     Drive identifier
     *  @param params   Drive params to setup
     *
     *  @return         Operation success flag
     */
    bool SetAxisParams(const Axis& axis, const AxisParams& params);

    /*! @brier Check if the system is in operational mode
     */
    bool IsOperational() const;

    /*! @brief Move the drive to the specified point
     *
     *  @attention In thismode after the target position reached drives are holding the positing using
     *  fixed torque value which is specified as the drive parameter
     *
     *  @param  axis    Drive identifier
     *  @param  pos     Specified point to run to [deg]
     *
     *  @return         Operation success flag
     */
    bool RunToPoint(const Axis& axis, double pos /*deg*/);


    /*! @brief Move the drive to the specified point
     *
     *  For each axis:
     *
     *  For azimuth axis:
     *  if (velocity > 0) {
     *      CW rotation;
     *  } else {
     *      CCW rotaion;
     *  }
     *
     *  For elevation axis:
     *  if (velocity > 0) {
     *      Rotation to rising antenna;
     *  } else {
     *      Rotation to lowering antenna;
     *  }
     *
     *  @param  axis    Drive identifier
     *  @param  vel     Specified rotation speed [deg/s]
     *
     *  @return         Operation success flag
     */
    bool RunAtVelocity(const Axis& axis, double vel /*deg/s*/);

    /*! @brief Enables drive operation (CiA402 status = Operation Enabled)
     *
     *  @param  axis    Drive identifier
     *
     *  @return         Operation success flag
     */
    bool Enable(const Axis& axis);

    /*! @brief Disables drive operation (CiA402 status = Switch on Disabled)
     *
     *  @param  axis    Drive identifier
     *
     *  @return         Operation success flag
     */
    bool Disable(const Axis& axis);

    /*! @brief Stop the drive operation (CiA402 status = Quick stop)
     *
     *  @param  axis    Drive identifier
     *
     *  @return         Operation success flag
     */
    bool Stop(const Axis& axis);

    /*! @brief Switch the drive to idle state (CiA402 status = Switched On)
     *
     *  @param  axis    Drive identifier
     *
     *  @return         Operation success flag
     */
    bool Idle(const Axis& axis);

    /*! @brief Reset fault of the specified axis
     *
     *  @param  axis    Drive identifier
     *
     *  @return         Operation success flag
     */
    bool ResetFault(const Axis& axis);

    /*! @brief Get the reference to current system status (dynamic)
     *
     *  @return Reference to structure with actual parameters
     */
    const std::atomic<SystemStatus>& GetStatusRef() const;

    /*! @brief Get the copy of current system status (dynamic)
     *
     *  @return Copy of structure with actual parameters
     */
    SystemStatus GetStatusCopy() const;

    /*! @brief Get static system info
     *
     *  @return Structure with actual system info
     */
    const SystemInfo& GetSystemInfo() const;

    AxisParamIndexMap GetAvailableAxisParams(const Axis& axis) const;
    AxisParams GetCurAxisParams(const Axis& axis) const;

    const std::atomic<CycleTimeInfo>& GetCycleTimeInfoRef() const;
    CycleTimeInfo GetCycleTimeInfo() const;
    void ResetCycleTimeInfo();

    /*! @brief Run static tests, print the result
     */
    static void RunStaticTests();

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespaces
