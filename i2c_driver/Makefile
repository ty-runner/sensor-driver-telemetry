# This is the name of the final kernel module
obj-m := bme280_sensor_module.o

# These are the object files that get linked into the module
bme280_sensor_module-objs := i2c_driver.o adc_conversion.o

# Kernel build directory
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
