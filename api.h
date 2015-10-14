#ifdef __cplusplus
extern "C" {
#endif

/*! @brief API системы управления двигателями метеорологической антенны ДМРЛ-3
 *
 *  @attention Методы _не- потокобезопасны.
 */

enum State {
    STATE_OFF = 0,
    STATE_READY,
    STATE_SCAN_ON_ELEVATION,
    STATE_SCAN_ON_AZIMUTH,
    STATE_POSITION,
    STATE_ERROR
};

//! @brief Инициализируем систему управления.
//! @return Одно из значений из перечисление State.
int Init();

//! @brief Получаем текущее состояние системы управления.
//! @return Одно из значений из перечисление State.
int GetState();

//! @brief Приводит систему управления в первоначальное состояние/выключает систему управления.
//! @return Одно из значений из перечисление State.
int Reset();

#ifdef __cplusplus
} // extern "C"
#endif
