#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/timekeeping.h>
#include <linux/of_device.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include "adc_conversion.h"

static struct task_struct *sensor_thread;
static bool thread_run = true;

MODULE_LICENSE("GPL");

struct my_data {
    char name[32];
    int i;
};

static struct my_data a = {
    "Device A",
    42,
};

struct bme280_sensor_packet{ //packet structure for transmission
    uint64_t timestamp_ns;
    int32_t temp_c;
    uint32_t humidity_percent;
    uint32_t pressure_pa;
    uint16_t crc;
} __attribute__((packed));

static struct i2c_device_id my_ids[] = {
    {"tyrunner_bme280", (long unsigned int) &a},
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

//Grouped data read
static void bme280_read_all(struct i2c_client *client, int32_t* temp_c, uint32_t* press_pa, uint32_t* humid_rh, struct timespec64 *ts){
    //ktime_get_real_ts64(ts); wall clock, used fro data logging and sensors
    ktime_get_ts64(ts); //monotonic, kernel
    uint8_t buf[3];

    printk(KERN_INFO
           "[%lld.%09ld] Timestamp and Read data code: %d\n",
           (long long)ts->tv_sec,
           ts->tv_nsec,
           read_bit_data(buf, client, 0xFA, 3));

    if (read_bit_data(buf, client, 0xFA, 3) == 0) { // temp
        int32_t temp_raw = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
        *temp_c = compensate_temp(temp_raw);
        printk(KERN_INFO
               "[%lld.%09ld] Temp: %d.%02d C\n",
               (long long)ts->tv_sec,
               ts->tv_nsec,
               *temp_c / 100,
               *temp_c % 100);
    }

    if (read_bit_data(buf, client, 0xF7, 3) == 0) { // pressure
        int32_t press_raw = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
        *press_pa = compensate_pressure(press_raw) >> 8;

        printk(KERN_INFO
               "[%lld.%09ld] Pressure: %u Pa\n",
               (long long)ts->tv_sec,
               ts->tv_nsec,
               *press_pa);
    }

    if (read_bit_data(buf, client, 0xFD, 2) == 0) { // humidity
        int32_t humid_raw = (buf[0] << 8) | buf[1];
        *humid_rh = compensate_humidity(humid_raw) / 1024;

        printk(KERN_INFO
               "[%lld.%09ld] Humidity: %u %%\n",
               (long long)ts->tv_sec,
               ts->tv_nsec,
               *humid_rh);
    }
}

static void send_data_packet(uint64_t timestamp_ns, int32_t temp_c, uint32_t press_pa, uint32_t humid_rh){
    struct bme280_sensor_packet pkt = {
        .timestamp_ns = timestamp_ns,
        .temp_c = temp_c,
        .humidity_percent = humid_rh,
        .pressure_pa = press_pa,
        .crc = 0
    };
    //transmit formed packet to given endpoint
}
static int sensor_thread_fn(void* client_ptr){
    struct timespec64 ts;
    struct i2c_client *client = client_ptr;
    int32_t temp_c;
    uint32_t press_pa;
    uint32_t humid_rh;
    uint64_t timestamp_ns;
    while(!kthread_should_stop()){

        bme280_read_all(client, &temp_c, &press_pa, &humid_rh, &ts);
        pr_info("THREAD READ -> Temp: %d.%02d C | Pressure: %u Pa | Humidity: %u %%\n",
                temp_c / 100,
                abs(temp_c % 100),
                press_pa,
                humid_rh);


        timestamp_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
        send_data_packet(timestamp_ns, temp_c, press_pa, humid_rh);
        msleep(1000);
    }

    return 0;
}
//Exposing the data to the userspace in the device tree
static ssize_t read_sensor_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    struct timespec64 ts;
    struct i2c_client *client = to_i2c_client(dev);
    int32_t temp_c;
    uint32_t press_pa;
    uint32_t humid_rh;
    bme280_read_all(client, &temp_c, &press_pa, &humid_rh, &ts);
    return sprintf(buf,
        "Temp: %d.%02d C\nPressure: %u Pa\nHumidity: %u %%\n",
        temp_c / 100, temp_c % 100,
        press_pa,
        humid_rh);

}

static DEVICE_ATTR_RO(read_sensor);

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

    printk(KERN_INFO "my_i2c_driver - %s data->i=%d\n", data->name, data->i);

    read_calibration_data(client);

    device_create_file(&client->dev, &dev_attr_read_sensor);
    sensor_thread = kthread_run(sensor_thread_fn,
                            client,
                            "bme280_thread");
    printk("End of probe \n");
    return 0;
}
static void my_remove(struct i2c_client *client){
    if (sensor_thread)
        kthread_stop(sensor_thread);
    device_remove_file(&client->dev, &dev_attr_read_sensor);
    printk("Removing device \n");
}

static const struct of_device_id my_of_match[] = {
    { .compatible = "tyrunner,bme280" },
    { }
};

static struct i2c_driver my_driver= {
    .probe = my_probe,
    .remove = my_remove,
    .id_table = my_ids,
    .driver = {
        .name = "my-i2c-driver",
        .of_match_table = my_of_match,
    }
};

module_i2c_driver(my_driver);
