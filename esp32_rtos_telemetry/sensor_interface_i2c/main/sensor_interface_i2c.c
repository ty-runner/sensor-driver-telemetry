#include <stdio.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
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
void bme280_verify()
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
}

data_packet_struct bme280_read(){
    
void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());

    vTaskDelay(pdMS_TO_TICKS(100));

    bme280_verify_and_init();

    //bme280_read_calib_data();
    while (1)
    {
        //sensor read task
        //bme280_read();

        //UDP packet send to endpoint
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
