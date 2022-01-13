/*
 *  HTU21D Library component
 *
 * ESP-IDF library to communicate with HTU21D humidity and temperature sensor
 * 
 */

#ifndef __HTU21D_H__
#define __HTU21D_H__

#include "esp_err.h"
#include "driver/i2c.h"



// return values


typedef enum {
    HTU21D_OK = 0,
    HTU21D_ERR_NOINIT = 0x201,
    HTU21D_ERR_CONFIG = 0x202,
    HTU21D_ERR_INSTALL = 0x203,
    HTU21D_ERR_NOTFOUND = 0x204,
    HTU21D_ERR_INVALID_ARG = 0x205,
    HTU21D_ERR_FAIL = 0x206,
    HTU21D_ERR_INVALID_STATE = 0x207,
    HTU21D_ERR_TIMEOUT = 0x208,
    HTU21D_ERR_INVALID_VALUE = 0x209,
} htu21d_result_t;

typedef enum {
    htu21d_resolution_t14b_rh12b = 0,
    htu21d_resolution_t13b_rh10b,
    htu21d_resolution_t12b_rh8b,
    htu21d_resolution_t11b_rh11b,
} htu21d_resolution_t;

typedef enum {
    htu21d_heater_off = 0,
    htu21d_heater_on = 1
} htu21d_heater_status_t;

typedef enum {
    htu21d_humidity_non_compensated = 0,
    htu21d_humidity_compensated
} htu21d_humidity_type_t;



htu21d_result_t htu21dInit(i2c_port_t port, int sda_pin, int scl_pin, gpio_pullup_t sda_internal_pullup, gpio_pullup_t scl_internal_pullup);
htu21d_result_t htu21dCheckPresent(void);
htu21d_result_t htu21dSoftReset(void);
htu21d_result_t htu21dSetResolution(htu21d_resolution_t resolution);
htu21d_result_t htu21dGetResolution(htu21d_resolution_t *p_resolution);
htu21d_result_t htu21dGetHeaterStatus(htu21d_heater_status_t *p_heater);
htu21d_result_t htu21dTurnHeaterOn(void);
htu21d_result_t htu21dTurnHeaterOff(void);
htu21d_result_t htu21dReadTemperature(float *temperature);
htu21d_result_t htu21dReadHumidity(float *rh, htu21d_humidity_type_t compensation_mode, float temperature);
htu21d_result_t htu21dGetResolutionValuesFromType(htu21d_resolution_t sensor_resolution, uint8_t *t_res, uint8_t *rh_res);

#endif // __HTU21D_H__