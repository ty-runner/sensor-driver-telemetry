#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/timekeeping.h>


MODULE_LICENSE("GPL");

struct my_data {
    char name[32];
    int i;
};

static struct my_data a = {
    "Device A",
    42,
};

static struct my_data b = {
    "Device B",
    123,
};

/* BME280 calibration data */
struct bme280_calib_data {
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
};

static struct i2c_device_id my_ids[] = {
    {"bme280", (long unsigned int) &a},
    {"b-dev", (long unsigned int) &b},
    {},
};
MODULE_DEVICE_TABLE(i2c, my_ids);

static int16_t read_s16_data(struct i2c_client *client, int reg){
    int lower = i2c_smbus_read_byte_data(client, reg);
    int higher = i2c_smbus_read_byte_data(client, reg+1);

    return (int16_t)((higher << 8) | lower);
}

static uint16_t read_u16_data(struct i2c_client *client, int reg){
    int lower = i2c_smbus_read_byte_data(client, reg);
    int higher = i2c_smbus_read_byte_data(client, reg+1);

    return (uint16_t)((higher << 8) | lower);
}

static struct bme280_calib_data calib;
static int32_t t_fine;

static void read_calibration_data(struct i2c_client *client){
    calib.dig_T1 = read_u16_data(client, 0x88);
    calib.dig_T2 = read_s16_data(client, 0x8A);
    calib.dig_T3 = read_s16_data(client, 0x8C);

    calib.dig_P1 = read_u16_data(client, 0x8E);
    calib.dig_P2 = read_u16_data(client, 0x90);
    calib.dig_P3 = read_u16_data(client, 0x92);
    calib.dig_P4 = read_u16_data(client, 0x94);
    calib.dig_P5 = read_u16_data(client, 0x96);
    calib.dig_P6 = read_u16_data(client, 0x98);
    calib.dig_P7 = read_u16_data(client, 0x9A);
    calib.dig_P8 = read_u16_data(client, 0x9C);
    calib.dig_P9 = read_u16_data(client, 0x9E);

    calib.dig_H1 = i2c_smbus_read_byte_data(client, 0xA1);
    calib.dig_H2 = read_s16_data(client, 0xE1);
    calib.dig_H3 = i2c_smbus_read_byte_data(client, 0xE3);
    calib.dig_H4 = (i2c_smbus_read_byte_data(client, 0xE4) << 4) | ((i2c_smbus_read_byte_data(client, 0xE5)) & 0x0F) ;
    calib.dig_H5 = (i2c_smbus_read_byte_data(client, 0xE6) << 4) | ((i2c_smbus_read_byte_data(client, 0xE5) >> 4) & 0x0F) ;
    calib.dig_H6 = (int8_t)i2c_smbus_read_byte_data(client, 0xE7);
}


static int bme280_init(struct i2c_client *client){
    if (i2c_smbus_write_byte_data(client, 0xF2, 0x01) < 0)
        return -EIO;
    if (i2c_smbus_write_byte_data(client, 0xF4, 0x27) < 0)
        return -EIO;
    return 0;
}

static int read_bit_data(uint8_t* buffer, struct i2c_client *client, int offset, int bytes){
    int return_status = i2c_smbus_read_i2c_block_data(client, offset, bytes, buffer);

    if(return_status < 0){
        return return_status;
    }
    return 0;
}
/* Temperature compensation */
static int32_t compensate_temp(int32_t adc_T)
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
static uint32_t compensate_pressure(int32_t adc_P)
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
static uint32_t compensate_humidity(int32_t adc_H)
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

static int my_probe(struct i2c_client *client)
{
    int id = i2c_smbus_read_byte_data(client, 0xD0);
    if (id < 0) {
        pr_err("Chip ID read failed: %d\n", id);
        return id;
    }
    pr_info("BME280 Chip ID: 0x%x\n", id);

    struct my_data *data = (struct my_data *)i2c_get_match_data(client);
    if (!data)
        data = &a; // fallback
    int init_result = bme280_init(client); 
    msleep(50);
    struct timespec64 ts;
    //ktime_get_real_ts64(&ts); wall clock, used fro data logging and sensors
    ktime_get_ts64(&ts); //monotonic, kernel

    printk(KERN_INFO "my_i2c_driver - %s data->i=%d\n", data->name, data->i);

    read_calibration_data(client);

    uint8_t buf[3];

    printk(KERN_INFO
           "[%lld.%09ld] Timestamp and Read data code: %d\n",
           (long long)ts.tv_sec,
           ts.tv_nsec,
           read_bit_data(buf, client, 0xFA, 3));

    if (read_bit_data(buf, client, 0xFA, 3) == 0) { // temp
        int32_t temp_raw = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
        int32_t temp_c = compensate_temp(temp_raw);
        printk(KERN_INFO
               "[%lld.%09ld] Temp: %d.%02d C\n",
               (long long)ts.tv_sec,
               ts.tv_nsec,
               temp_c / 100,
               temp_c % 100);
    }

    if (read_bit_data(buf, client, 0xF7, 3) == 0) { // pressure
        int32_t press_raw = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
        uint32_t press_pa = compensate_pressure(press_raw) >> 8;

        printk(KERN_INFO
               "[%lld.%09ld] Pressure: %u Pa\n",
               (long long)ts.tv_sec,
               ts.tv_nsec,
               press_pa);
    }

    if (read_bit_data(buf, client, 0xFD, 2) == 0) { // humidity
        int32_t humid_raw = (buf[0] << 8) | buf[1];
        uint32_t humid_rh = compensate_humidity(humid_raw) / 1024;

        printk(KERN_INFO
               "[%lld.%09ld] Humidity: %u %%\n",
               (long long)ts.tv_sec,
               ts.tv_nsec,
               humid_rh);
    }

    printk("End of probe \n");
    return 0;
}
static void my_remove(struct i2c_client *client){
    printk("Removing device \n");
}

static struct i2c_driver my_driver= {
    .probe = my_probe,
    .remove = my_remove,
    .id_table = my_ids,
    .driver = {
        .name = "my-i2c-driver",
    }
};

module_i2c_driver(my_driver);
