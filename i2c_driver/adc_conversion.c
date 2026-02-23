#include <linux/types.h>
#include "adc_conversion.h"
//In here lives the adc constant translations from raw ADC reads to sensor values we can understand
int32_t t_fine;
struct bme280_calib_data calib;

/* Temperature compensation */
int32_t compensate_temp(int32_t adc_T)
{
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) *
            ((int32_t)calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
            ((int32_t)calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8; // 0.01°C
}

/* Pressure compensation */
uint32_t compensate_pressure(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = (int64_t)t_fine - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 += ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 += ((int64_t)calib.dig_P4 << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) +
           ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * calib.dig_P1 >> 33;

    if (var1 == 0)
        return 0;

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = (calib.dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)calib.dig_P7 << 4);
    return (uint32_t)p; // Pa * 256
}

/* Humidity compensation */
uint32_t compensate_humidity(int32_t adc_H)
{
    int32_t v_x1;
    v_x1 = t_fine - 76800;
    v_x1 = (((((adc_H << 14) - ((int32_t)calib.dig_H4 << 20) -
               ((int32_t)calib.dig_H5 * v_x1)) + 16384) >> 15) *
            (((((((v_x1 * (int32_t)calib.dig_H6) >> 10) *
                 (((v_x1 * (int32_t)calib.dig_H3) >> 11) + 32768)) >> 10) +
                 2097152) * (int32_t)calib.dig_H2 + 8192) >> 14));
    v_x1 -= (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * calib.dig_H1) >> 4);
    if (v_x1 < 0) v_x1 = 0;
    if (v_x1 > 419430400) v_x1 = 419430400;
    return (uint32_t)(v_x1 >> 12); // %RH * 1024
}
