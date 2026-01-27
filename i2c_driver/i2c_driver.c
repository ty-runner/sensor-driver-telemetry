#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>

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

static int my_probe(struct i2c_client *client)
{
    struct my_data *data;

    data = (struct my_data *)i2c_get_match_data(client);
    if (!data)
        return -ENODEV;

    printk(KERN_INFO "my_i2c_driver - %s data->i=%d\n",
           data->name, data->i);

    printk(KERN_INFO "my_i2c_driver - ID: 0x%x\n",
           i2c_smbus_read_byte_data(client, 0xd0));

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
