obj-m += ldd.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
# -C changes working directory
#  PWD present working directory
#  M= insert modules
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

test.text: test.c
	cat test.c
	cat test.c > test.text
	cat test.text
