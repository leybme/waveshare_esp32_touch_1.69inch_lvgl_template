#ifndef DRIVER_H
#define DRIVER_H


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


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*DRIVER_H*/
