#include "secrets.h"
#include <stdio.h>
#include <stdint.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "adc_conversion_32bit.h"
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

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

static uint64_t htonll(uint64_t value)
{
    uint32_t high = htonl((uint32_t)(value >> 32));
    uint32_t low  = htonl((uint32_t)(value & 0xFFFFFFFFULL));
    return ((uint64_t)low << 32) | high;
}

static int32_t htonl_i32(int32_t value)
{
    return (int32_t)htonl((uint32_t)value);
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

static struct bme280_sensor_packet bme280_read(void)
{
    struct bme280_sensor_packet pkt;
    memset(&pkt, 0, sizeof(pkt));

    bme280_read_all(&pkt.temp_c,
                    &pkt.pressure_pa,
                    &pkt.humidity_percent,
                    &pkt.timestamp_ns);

    pkt.crc = 0;  // set after serialization
    return pkt;
}

static void serialize_bme280_packet(const struct bme280_sensor_packet *src, struct bme280_sensor_packet *dst)
{
    dst->timestamp_ns      = htonll(src->timestamp_ns);
    dst->temp_c            = htonl_i32(src->temp_c);
    dst->humidity_percent  = htonl(src->humidity_percent);
    dst->pressure_pa       = htonl(src->pressure_pa);
    dst->crc               = 0;

    //uint16_t crc = crc16_ccitt((const uint8_t *)dst, sizeof(struct bme280_sensor_packet) - sizeof(uint16_t));

    //dst->crc = htons(crc);
}
/* NETWORKING FUNCTIONS*/
static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    ESP_LOGI("WIFI", "wifi_init_sta finished");
}

static int udp_socket_create(struct sockaddr_in *dest_addr)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE("UDP", "Unable to create socket");
        return -1;
    }

    memset(dest_addr, 0, sizeof(*dest_addr));
    dest_addr->sin_family = AF_INET;
    dest_addr->sin_port = htons(UDP_DEST_PORT);
    dest_addr->sin_addr.s_addr = inet_addr(UDP_DEST_IP);

    ESP_LOGI("UDP", "UDP socket created, sending to %s:%d", UDP_DEST_IP, UDP_DEST_PORT);
    return sock;
}

static esp_err_t udp_send_packet(int sock, const struct sockaddr_in *dest_addr,
                                 const struct bme280_sensor_packet *pkt)
{
    int err = sendto(sock,
                     pkt,
                     sizeof(*pkt),
                     0,
                     (const struct sockaddr *)dest_addr,
                     sizeof(*dest_addr));

    if (err < 0) {
        ESP_LOGE("UDP", "sendto failed: errno=%d", errno);
        return ESP_FAIL;
    }

    ESP_LOGI("UDP", "Sent %d bytes", err);
    return ESP_OK;
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    vTaskDelay(pdMS_TO_TICKS(100));

    bme280_verify_and_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    read_calibration_data();

    wifi_init_sta();
    vTaskDelay(pdMS_TO_TICKS(3000)); // simple delay so Wi-Fi can associate

    struct sockaddr_in dest_addr;
    int sock = udp_socket_create(&dest_addr);
    if (sock < 0) {
        ESP_LOGE("APP", "Failed to create UDP socket");
        return;
    }
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    uint64_t prev_loop_start = 0;

    while (1) {
        // ---- LOOP START TIMESTAMP ----
        uint64_t loop_start = esp_timer_get_time(); // us

        // ---- JITTER (TRUE PERIOD) ----
        if (prev_loop_start != 0) {
            uint64_t loop_period = loop_start - prev_loop_start;
            ESP_LOGI("METRIC", "Loop Period (Jitter): %llu us", loop_period);
        }
        prev_loop_start = loop_start;

        // ---- END-TO-END LATENCY START ----
        uint64_t e2e_start = esp_timer_get_time();

        struct bme280_sensor_packet pkt_host = bme280_read();
        struct bme280_sensor_packet pkt_net;

        serialize_bme280_packet(&pkt_host, &pkt_net);

        udp_send_packet(sock, &dest_addr, &pkt_net);

        // ---- END-TO-END LATENCY END ----
        uint64_t e2e_end = esp_timer_get_time();
        ESP_LOGI("METRIC", "E2E Latency: %llu us", (e2e_end - e2e_start));

        // ---- LOOP EXECUTION TIME ----
        uint64_t loop_end = esp_timer_get_time();
        ESP_LOGI("METRIC", "Loop Exec Time: %llu us", (loop_end - loop_start));

        // Optional: keep your sensor print
        ESP_LOGI("BME280",
                 "TS=%llu ns Temp=%d.%02d C Press=%u Pa Hum=%u%%",
                 pkt_host.timestamp_ns,
                 pkt_host.temp_c / 100,
                 abs(pkt_host.temp_c % 100),
                 pkt_host.pressure_pa,
                 pkt_host.humidity_percent);

        // ---- DETERMINISTIC DELAY ----
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    close(sock);
    /*while (1) {
        struct bme280_sensor_packet pkt_host = bme280_read();
        struct bme280_sensor_packet pkt_net;

        serialize_bme280_packet(&pkt_host, &pkt_net);

        ESP_LOGI("BME280",
                 "TS=%llu ns Temp=%d.%02d C Press=%u Pa Hum=%u%%",
                 pkt_host.timestamp_ns,
                 pkt_host.temp_c / 100,
                 abs(pkt_host.temp_c % 100),
                 pkt_host.pressure_pa,
                 pkt_host.humidity_percent);

        udp_send_packet(sock, &dest_addr, &pkt_net);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    close(sock);*/
}
