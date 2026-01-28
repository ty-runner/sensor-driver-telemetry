#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>

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
static struct i2c_device_id my_ids[] = {
    {"a-dev", (long unsigned int) &a},
    {"b-dev", (long unsigned int) &b},
    {},
};
MODULE_DEVICE_TABLE(i2c, my_ids);

static void control_write(struct i2c_client *client){
    i2c_smbus_write_byte_data(client, 0xF2, 0x01);
    i2c_smbus_write_byte_data(client, 0xF4, 0x27);
}

static int read_bit_data(uint8_t* buffer, struct i2c_client *client, int offset, int bytes){
    control_write(client); //param here will be the oversampling
    msleep(10);
    int return_status = i2c_smbus_read_i2c_block_data(client, offset, bytes, buffer);

    if(return_status < bytes){
        return -1;
    }
    return 0;
}


static int my_probe(struct i2c_client *client)
{
    struct my_data *data;
    uint8_t buf[3];
    data = (struct my_data *)i2c_get_match_data(client);
    if (!data)
        return -ENODEV;

    printk(KERN_INFO "my_i2c_driver - %s data->i=%d\n",
           data->name, data->i);

    printk(KERN_INFO "my_i2c_driver - ID: 0x%x\n",
           i2c_smbus_read_byte_data(client, 0xd0));

    int ret = read_bit_data(buf, client, 0xf7, 3);
    if(ret == 0){
        uint32_t press_raw = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
        printk(KERN_INFO "Raw pressure: %u\n", press_raw);
    }

    ret = read_bit_data(buf, client, 0xfa, 3);
    if(ret == 0){
        uint32_t temp_raw = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
        printk(KERN_INFO "Raw temp: %u\n", temp_raw);
    }

    ret = read_bit_data(buf, client, 0xfd, 2);
    if(ret == 0){
        uint32_t humid_raw = (buf[0] << 8) | (buf[1]);
        printk(KERN_INFO "Raw humidity: %u\n", humid_raw);
    }
    //pressure - 0xF7 - 0xF9
    //temp - 0xFA - 0xFC
    //temp - 0xFA - 0xFC
    //humidity - 0xFD - 0xFE
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
