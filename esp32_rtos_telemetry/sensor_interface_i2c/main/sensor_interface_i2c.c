#include <stdio.h>
#include <stdint.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "adc_conversion_32bit.h"

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_FREQ_HZ 100000

#define I2C_MASTER_PORT I2C_NUM_0

#define BME280_ADDR 0x77   // or 0x77 depending on wiring
#define BME280_CHIP_ID_REG 0xD0
#define BME280_CHIP_ID 0x60

struct bme280_sensor_packet{ //packet structure for transmission
    uint64_t timestamp_ns;
    int32_t temp_c;
    uint32_t humidity_percent;
    uint32_t pressure_pa;
    uint16_t crc;
} __attribute__((packed));

struct bme280_client{
    uint8_t dev_addr;
    uint8_t reg;
    uint8_t* data;
    size_t len;
};

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err;

    err = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (err != ESP_OK) return err;

    err = i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);

    return err;
}

static esp_err_t i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        I2C_MASTER_PORT,
        dev_addr,
        &reg,
        1,
        data,
        len,
        pdMS_TO_TICKS(1000)
    );
}

static esp_err_t i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};

    return i2c_master_write_to_device(
        I2C_MASTER_PORT,
        dev_addr,
        write_buf,
        sizeof(write_buf),
        pdMS_TO_TICKS(1000)
    );
}

static int16_t read_s16_data(int reg){
    uint8_t lower;
    i2c_read_reg(BME280_ADDR, reg, &lower, 1);
    uint8_t higher;
    i2c_read_reg(BME280_ADDR, reg+1, &higher, 1);

    return (int16_t)((higher << 8) | lower);
}

static uint16_t read_u16_data(int reg){
    uint8_t lower;
    i2c_read_reg(BME280_ADDR, reg, &lower, 1);
    uint8_t higher;
    i2c_read_reg(BME280_ADDR, reg+1, &higher, 1);

    return (uint16_t)((higher << 8) | lower);
}

static void read_calibration_data(){
    calib.dig_T1 = read_u16_data(0x88);
    calib.dig_T2 = read_s16_data(0x8A);
    calib.dig_T3 = read_s16_data(0x8C);

    calib.dig_P1 = read_u16_data(0x8E);
    calib.dig_P2 = read_u16_data(0x90);
    calib.dig_P3 = read_u16_data(0x92);
    calib.dig_P4 = read_u16_data(0x94);
    calib.dig_P5 = read_u16_data(0x96);
    calib.dig_P6 = read_u16_data(0x98);
    calib.dig_P7 = read_u16_data(0x9A);
    calib.dig_P8 = read_u16_data(0x9C);
    calib.dig_P9 = read_u16_data(0x9E);

    i2c_read_reg(BME280_ADDR, 0xA1, &(calib.dig_H1), 1);
    calib.dig_H2 = read_s16_data(0xE1);
    i2c_read_reg(BME280_ADDR, 0xE3, &(calib.dig_H3), 1);
    i2c_read_reg(BME280_ADDR, 0xE4, (uint8_t*)&(calib.dig_H4), 1);
    calib.dig_H4 <<= 4;
    uint8_t tmp;
    i2c_read_reg(BME280_ADDR, 0xE5, &(tmp), 1);
    calib.dig_H4 |= (tmp & 0x0F);

    i2c_read_reg(BME280_ADDR, 0xE6, (uint8_t*)&(calib.dig_H5), 1);
    calib.dig_H5 <<= 4;
    i2c_read_reg(BME280_ADDR, 0xE5, &(tmp), 1);
    calib.dig_H5 |= ((tmp >> 4) & 0x0F);

    i2c_read_reg(BME280_ADDR, 0xE7, (uint8_t*)&(calib.dig_H6), 1); //might have to be signed?
}

void bme280_verify_and_init()
{
    uint8_t chip_id;

    esp_err_t ret = i2c_read_reg(
        BME280_ADDR,
        BME280_CHIP_ID_REG,
        &chip_id,
        1
    );

    if (ret != ESP_OK)
    {
        ESP_LOGE("BME280", "I2C read failed");
        return;
    }

    if (chip_id == BME280_CHIP_ID)
    {
        ESP_LOGI("BME280", "Sensor detected! Chip ID: 0x%02X", chip_id);
    }
    else
    {
        ESP_LOGE("BME280", "Unexpected chip ID: 0x%02X", chip_id);
    }

    //init sensor readings from BME280
    if(i2c_write_reg(BME280_ADDR, 0xF2, 0x01) < 0){
        ESP_LOGE("BME280", "I2C Init Write Failed");
        return;   
    }
    if(i2c_write_reg(BME280_ADDR, 0xF4, 0x27) < 0){
        ESP_LOGE("BME280", "I2C Init Write Failed");
        return;   
    }

    //Read calibration data
}

static void bme280_read_all(int32_t* temp_c, uint32_t* press_pa, uint32_t* humid_rh, uint64_t* timestamp_ns)
{
    uint8_t buf[8];

    // timestamp (ESP timer is microseconds)
    *timestamp_ns = esp_timer_get_time() * 1000ULL;

    if (i2c_read_reg(BME280_ADDR, 0xF7, buf, 8) != ESP_OK) {
        ESP_LOGE("BME280", "Failed to read sensor data");
        return;
    }

    // Pressure raw
    int32_t press_raw =
        ((int32_t)buf[0] << 12) |
        ((int32_t)buf[1] << 4) |
        ((int32_t)buf[2] >> 4);

    // Temperature raw
    int32_t temp_raw =
        ((int32_t)buf[3] << 12) |
        ((int32_t)buf[4] << 4) |
        ((int32_t)buf[5] >> 4);

    // Humidity raw
    int32_t humid_raw =
        ((int32_t)buf[6] << 8) |
        (int32_t)buf[7];

    // Compensation (Bosch formulas)
    *temp_c = compensate_temp(temp_raw);
    *press_pa = compensate_pressure(press_raw) >> 8;
    *humid_rh = compensate_humidity(humid_raw) / 1024;

    ESP_LOGI("BME280",
        "Temp: %d.%02d C  Pressure: %u Pa  Humidity: %u %%",
        *temp_c / 100,
        *temp_c % 100,
        *press_pa,
        *humid_rh);
}
struct bme280_sensor_packet bme280_read(){
    struct bme280_sensor_packet pkt = {
        .timestamp_ns = 0,
        .temp_c = 0,
        .humidity_percent = 0,
        .pressure_pa = 0,
        .crc = 0,
    };
    return pkt;
}
    
void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());

    vTaskDelay(pdMS_TO_TICKS(100));

    bme280_verify_and_init();

    vTaskDelay(pdMS_TO_TICKS(100));

    read_calibration_data();

    int32_t temp_c;
    uint32_t press_pa;
    uint32_t humid_rh;
    uint64_t ts;
    while (1)
    {
        //sensor read task
        bme280_read_all(&temp_c, &press_pa, &humid_rh, &ts);

        ESP_LOGI("BME280", "Timestamp: %llu ns", ts);
        //UDP packet send to endpoint
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
