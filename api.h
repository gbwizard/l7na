#ifdef __cplusplus
extern "C" {
#endif

/*! @brief API системы управления двигателями метеорологической антенны ДМРЛ-3
 *
 *  Описание работы с системой управления.
 *
 *
 *
 *  @attention Методы _НЕ_ потокобезопасны.
 */

//! @brief Возможные состояния системы управления.
enum State {
    STATE_OFF = 0,
    STATE_READY,
    STATE_SCAN_ON_ELEVATION,
    STATE_SCAN_ON_AZIMUTH,
    STATE_POSITION,
    STATE_ERROR,
    STATE_ERROR_FATAL
};

//! @brief Параметры, которые возможно получить из системы управления.
struct System {
    struct AxisParams {
        enum OperationMode {

        };

        double target_velocity;
        double cur_velocity;

        double tartet_
    };

    double azimuth_velocity;
    double azimuth_position;
    double elevation_velocity;
    double elevation_position;
};

//! @brief Инициализирует систему управления.
//!
//! @param   cfg_file_path  Путь к файлу с конфигурацией системы (абсолютный или относительно текущей рабочей директории)
//!
//! @return Одно из значений в перечислении State.
int Init(const char* cfg_file_path);

//! @brief Приводит систему управления в первоначальное состояние/выключает систему управления.
//! @return Одно из значений в перечисления State.
int Release();

int SetModeAzimuthRotation(double elevation_angle, double azimuth_velocity);
int SetModeElevationRotation(double azimuth_angle, double elevation_velocity);
int SetModePointPosition(double azimuth_angle, double elevation_angle);

//! @brief Получаем текущее состояние системы управления.
//! @return Одно из значений в перечислении State.
int GetState();

double GetAzimuthVelocity();
double GetAzimuthPosition();
double GetElevationVelocity();
double GetElevationPosition();

#ifdef __cplusplus
} // extern "C"
#endif
