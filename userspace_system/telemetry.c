#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include <string.h>
#include <curl/curl.h>

#define BME280_I2C_ADDR 0x77
#define I2C_DEV "/dev/i2c-1"

// BME280 registers
#define REG_CTRL_MEAS 0xF4
#define REG_CONFIG    0xF5
#define REG_DATA      0xF7

// Calibration struct
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_calib_data;

bme280_calib_data calib;
int fd;

// --- Helper functions ---
void i2c_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    write(fd, buf, 2);
}

void i2c_read(uint8_t reg, uint8_t *buf, int len) {
    write(fd, &reg, 1);
    read(fd, buf, len);
}

// Read calibration data from BME280
void read_calibration() {
    uint8_t buf[26];
    i2c_read(0x88, buf, 24);

    calib.dig_T1 = buf[0] | (buf[1]<<8);
    calib.dig_T2 = buf[2] | (buf[3]<<8);
    calib.dig_T3 = buf[4] | (buf[5]<<8);

    calib.dig_P1 = buf[6] | (buf[7]<<8);
    calib.dig_P2 = buf[8] | (buf[9]<<8);
    calib.dig_P3 = buf[10] | (buf[11]<<8);
    calib.dig_P4 = buf[12] | (buf[13]<<8);
    calib.dig_P5 = buf[14] | (buf[15]<<8);
    calib.dig_P6 = buf[16] | (buf[17]<<8);
    calib.dig_P7 = buf[18] | (buf[19]<<8);
    calib.dig_P8 = buf[20] | (buf[21]<<8);
    calib.dig_P9 = buf[22] | (buf[23]<<8);

    i2c_read(0xA1, &calib.dig_H1, 1);

    i2c_read(0xE1, buf, 7);
    calib.dig_H2 = buf[0] | (buf[1]<<8);
    calib.dig_H3 = buf[2];
    calib.dig_H4 = (buf[3]<<4) | (buf[4]&0xF);
    calib.dig_H5 = (buf[5]<<4) | ((buf[4]>>4)&0xF);
    calib.dig_H6 = buf[6];
}

// --- Compensation formulas from datasheet ---
int32_t t_fine;

float compensate_T(int32_t adc_T) {
    int32_t var1 = ((((adc_T>>3) - ((int32_t)calib.dig_T1 <<1))) * ((int32_t)calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T>>4) - ((int32_t)calib.dig_T1)) * ((adc_T>>4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    return ((float)((t_fine * 5 + 128) >> 8)) / 100.0;
}

float compensate_P(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1*(int64_t)calib.dig_P5)<<17);
    var2 = var2 + (((int64_t)calib.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3)>>8) + ((var1 * (int64_t)calib.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calib.dig_P1)>>33;
    if (var1 == 0) return 0; // avoid divide by zero
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125)/var1;
    var1 = (((int64_t)calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7)<<4);
    return (float)p/25600.0;
}

float compensate_H(int32_t adc_H) {
    int32_t v_x1_u32r = t_fine - 76800;
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) - (((int32_t)calib.dig_H5) * v_x1_u32r)) + 16384) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) + 32768)) >> 10) + 2097152) * ((int32_t)calib.dig_H2) + 8192) >> 14));
    v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib.dig_H1)) >> 4);
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (float)(v_x1_u32r>>12)/1024.0;
}

// --- Read sensor ---
void read_sensor(float *temperature, float *pressure, float *humidity) {
    uint8_t data[8];
    i2c_read(REG_DATA, data, 8);

    int32_t adc_P = (data[0]<<12) | (data[1]<<4) | (data[2]>>4);
    int32_t adc_T = (data[3]<<12) | (data[4]<<4) | (data[5]>>4);
    int32_t adc_H = (data[6]<<8) | data[7];

    *temperature = compensate_T(adc_T);
    *pressure    = compensate_P(adc_P);
    *humidity    = compensate_H(adc_H);
}

// --- Send HTTP POST ---

void send_http(float temperature, float pressure, float humidity) {
    CURL *curl = curl_easy_init();
    if(curl) {
        CURLcode res;

        // Build JSON
        char json[128];
        snprintf(json, sizeof(json),
                 "{\"temperature\":%.2f,\"pressure\":%.2f,\"humidity\":%.2f}",
                 temperature, pressure, humidity);

        // Set headers
        struct curl_slist *headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");

        curl_easy_setopt(curl, CURLOPT_URL, "https://webhook.site/101131b5-b101-43d4-87cb-9f75b75b6444");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        // Tell curl this is a POST with a body
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json);

        // Optional: timeout
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);

        // Perform the request
        res = curl_easy_perform(curl);
        if(res != CURLE_OK) {
            fprintf(stderr, "curl failed: %s\n", curl_easy_strerror(res));
        } else {
            printf("POST sent: %s\n", json);
        }

        // Clean up
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
    }
}

int main() {
    // Open I2C
    if ((fd = open(I2C_DEV, O_RDWR)) < 0) {
        perror("Open I2C device");
        return 1;
    }
    if (ioctl(fd, I2C_SLAVE, BME280_I2C_ADDR) < 0) {
        perror("Set I2C address");
        return 1;
    }

    // Read calibration
    read_calibration();

    // Configure sensor: normal mode, 1x oversampling for speed
    i2c_write(REG_CTRL_MEAS, 0x27);
    i2c_write(REG_CONFIG, 0xA0);

    struct timespec ts = {0, 1000000000L}; // 1 Hz → 1000ms
    while(1) {
        float t, p, h;
        read_sensor(&t, &p, &h);
        printf("T: %.2f C, P: %.2f hPa, H: %.2f %%\n", t, p, h);
        send_http(t, p, h);
        nanosleep(&ts, NULL);
    }

    return 0;
}

