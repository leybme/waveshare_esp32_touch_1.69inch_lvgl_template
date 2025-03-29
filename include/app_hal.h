#ifndef DRIVER_H
#define DRIVER_H
#include <stdint.h>

#if defined(ESPS3_1_69) || defined(ESPS3_1_28)
#define ENABLE_APP_QMI8658C
#define ENABLE_APP_ATTITUDE
#endif

#if defined(ELECROW_C3)
#define ENABLE_RTC
#endif

#ifdef __cplusplus
extern "C" {
#endif


void hal_setup(void);
void hal_loop(void);
void setBrightness(uint8_t brightness);
typedef void (*ButtonCallback)();  // Define a callback type
void setShortPressCallback(ButtonCallback cb);
void setDoublePressCallback(ButtonCallback cb);
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*DRIVER_H*/
