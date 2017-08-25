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

/*! @brief Объект управления системой вращения.
 *
 *  Созданием объекта == подлключение к системе управления
 *
 */
class Control {
public:
    /*! @brief Конструктор. Инициализирует систему управления.
     *
     *  @param   cfg_file_path  Путь к файлу с конфигурацией системы (абсолютный или относительно текущей рабочей директории)
     */
    Control(const Config::Storage& config, const ParamsMode params_mode = PARAMS_MODE_AUTOMATIC);

    /*! @brief Деструктор. Приводит систему управления в первоначальное состояние/выключает систему управления.
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

    /*! @brief В зависимости от значений параметров, задает разные режимы работы двигателей по осям (позиционирование в точку или сканирование).
     *
     *  Соответственно для каждой оси:
     *  if (velocity == 0) {
     *      Позиционирование в точку;
     *  } else {
     *      Вращение с указанной скоростью;
     *  }
     *
     *  Для азимута:
     *  if (velocity > 0) {
     *      Вращение _по_ часовой стрелке;
     *  } else {
     *      Вращение _против_ часовой стрелки;
     *  }
     *
     *  Для угла места:
     *  if (velocity > 0) {
     *      Вращение поднимающее антенну;
     *  } else {
     *      Вращение опускающее антенну;
     *  }
     *
     *  @attention В режиме позиционирования в точку при достижении указанной позиции двигатели фиксируют положение
     *  фиксированным моментом (задаваемым в настройках).
     *
     *  @param  axis                Идентификатор двигателя
     *  @param  pos                 Фиксированный позиция, в которую перемещается двигатель [градусы]
     *  @param  vel                 Скорость, с которой двигатель вращается в режиме постоянной скорости
     *                              (это НЕ скорость с которой он перемещается в заданную позицию) [градусы/с]
     *
     *  @return                     Флаг успешности операции
     */
    bool SetModeRun(const Axis& axis, double pos /*deg*/, double vel /*deg/s*/);

    /*! @brief Добавляем в систему новый режим работы по параметрам.
     *
     * @param axis          Идентификатор двигателя
     * @param mode          Идентификатор режима работы, соответствует максимальному расстоянию в градусах
     *                      между текущей и запрашиваемой позицией.
     *                      Для режима AXIS_SCAN используется режим с максимальным идентификатором.
     * @param params        Выставляемые параметры двигателя
     */
    bool AddMoveMode(const Axis& axis, const MoveMode& mode, const AxisParams& params);

    /*! @brief Задает режим выставления параметров двигателя ()
     *
     * @param axis          Идентификатор двигателя
     * @param params_mode   Режим выставления параметров двигателя
     */
    bool SetParamsMode(const Axis& axis, const ParamsMode& params_mode);
    bool GetParamsMode(const Axis& axis, ParamsMode& params_mode) const;

    /*! @brief Выставляет параметры оси
     *
     * @param axis          Идентификатор двигателя
     * @param params        Выставляемые параметры двигателя
     */
    bool SetAxisParams(const Axis& axis, const AxisParams& params);

    /*! @brief Переключает систему управления одной оси в режим бездейсвтия.
     *
     *  @param  axis                Идентификатор двигателя
     *
     *  @return                     Флаг успешности операции
     *
     */
    bool SetModeIdle(const Axis& axis);

    /*! @brief Сбрасывает ошибку одной оси.
     *
     *  @param  axis                Идентификатор двигателя
     *
     *  @return                     Флаг успешности операции
     */
    bool ResetFault(const Axis& axis);

    /*! @brief Получаем текущее состояние системы управления (динамически изменяемые)
     *
     *  @return Структуру Status, заполненную актуальными данными.
     */
    const std::atomic<SystemStatus>& GetStatusRef() const;

    SystemStatus GetStatusCopy() const;

    /*! @brief Получаем статические параметры системы (не изменяющиеся с течением времени).
     *
     *  @return Структуру SystemInfo, заполненную актуальными данными.
     */
    const SystemInfo& GetSystemInfo() const;

    AxisParamIndexMap GetAvailableAxisParams(const Axis& axis) const;
    AxisParams GetCurAxisParams(const Axis& axis) const;

    const std::atomic<CycleTimeInfo>& GetCycleTimeInfoRef() const;
    CycleTimeInfo GetCycleTimeInfo() const;
    void ResetCycleTimeInfo();

    /*! @brief Запускаем статические тесты, печатаем результат
     */
    static void RunStaticTests();

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespaces
