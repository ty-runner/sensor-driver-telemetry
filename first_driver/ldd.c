//this is the first c module for our driver
#include <linux/init.h>
#include <linux/module.h>


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ty");
MODULE_DESCRIPTION("Dynamically loadable kernel module");

static int mod_init (void){
	printk("HI!\n");
	return 0;
}

static void mod_exit (void){
	printk("BYE!\n");
}

module_init(mod_init);
module_exit(mod_exit);
