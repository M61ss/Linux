#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mattia");
MODULE_DESCRIPTION("Test");

static struct proc_dir_entry *custom_proc_node;

static ssize_t mattia_read(struct file *pointer,
                    char __user *user_space_buffer, 
                    size_t count,
                    loff_t *offset) {
    char msg[] = "User space info\n";
    size_t msg_len = strlen(msg);
    int return_value;

    return_value = copy_to_user(user_space_buffer, msg, msg_len);

    printk("Read!\n");

    return msg_len;
}

struct proc_ops driver_proc_ops = {
    .proc_read = mattia_read
};

static int mattia_module_init (void) {
    printk("mattia_module_init: entry\n");

    custom_proc_node = proc_create("my_driver", 0, NULL, &driver_proc_ops);
    if (custom_proc_node == NULL) {
        printk("mattia_module_init: exit\n");
        return -1;
    }

    printk("mattia_module_init: exit\n");
    return 0;
}

static void mattia_module_exit (void) {
    printk("mattia_module_exit: entry\n");

    proc_remove(custom_proc_node);

    printk("mattia_module_exit: exit\n");
}

module_init(mattia_module_init);
module_exit(mattia_module_exit);