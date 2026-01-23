//this is the first c module for our driver
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ty");
MODULE_DESCRIPTION("Dynamically loadable kernel module");

static struct proc_dir_entry *custom_proc_node;

static ssize_t mod_write(struct file* file_pointer,
                        const char *user_space_buffer,
                        size_t count,
                        loff_t* offset) {
    char proc_msg[128];

    if(copy_from_user(proc_msg, user_space_buffer, count))
        return -EFAULT;

    proc_msg[count] = '\0';

    printk("PROC WRITE: %s\n", proc_msg);

    return count;
}
static ssize_t mod_read(struct file* file_pointer,
                        char *user_space_buffer,
                        size_t count,
                        loff_t* offset) {
    char msg[] = "ACK!\n";
    size_t len = strlen(msg);

    int result = copy_to_user(user_space_buffer, msg, len);

    printk("READ\n");
    if(*offset >= len)
        return 0;
    *offset += len;

    return len;
}

struct proc_ops driver_proc_ops = {
    .proc_read = mod_read,
    .proc_write = mod_write
};
static int mod_init (void){
	printk("HI! - entry\n");
    custom_proc_node = proc_create("ty_driver",
                                   0,
                                   NULL,
                                   &driver_proc_ops);
    if(!custom_proc_node){
        return -ENOMEM;
    }
	printk("HI! - eexit\n");
	return 0;
}

static void mod_exit (void){
    proc_remove(custom_proc_node);
	printk("BYE!\n");
}

module_init(mod_init);
module_exit(mod_exit);
