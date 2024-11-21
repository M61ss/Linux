# Linux drivers basics <!-- omit from toc -->

- [Linux SSH subsystem](#linux-ssh-subsystem)
  - [Multipass installation](#multipass-installation)
  - [Multipass instance configuration](#multipass-instance-configuration)
- [Setting up](#setting-up)
- [Compiling drivers](#compiling-drivers)
- [Using kernel modules](#using-kernel-modules)
  - [`insmod`](#insmod)
  - [`rmmod`](#rmmod)
- [`/proc/` filesystem](#proc-filesystem)
  - [Create a file entry](#create-a-file-entry)
  - [Remove a file entry](#remove-a-file-entry)
  - [Troubleshooting](#troubleshooting)
- [Implementing a read operation](#implementing-a-read-operation)
- [Passing data to the user](#passing-data-to-the-user)
- [Complete working module](#complete-working-module)
- [Conclusion](#conclusion)

## Linux SSH subsystem

### Multipass installation

Install multipass as hypervisor for a simple shell system on which we can safely do our tests:

```bash
sudo snap install multipass
```

Then, install your subsystem using:

```bash
multipass shell
```

> [!TIP] Check images
> 
> If you want to verify the list of installed images:
>
> ```bash
> multipass list
> ```

### Multipass instance configuration

Run the multipass shell, so, inside it, run:

```bash
sudo vim /etc/ssh/sshd_config
```

So switch to `yes` the value `KbdInteractiveAuthentication`.

Then, run:

```bash
sudo systemctl daemon-reload
```

```bash
sudo service ssh restart
```

Then, set a password for the multipass superuser:

```bash
sudo passwd ubuntu
```

> [!NOTE] 
>
> `ubuntu` is the username.

Now, get the IP address, so run:

```bash
hostname -I
```

Copy the IP address, then, inside the VSCode extention "Remote development", go to "Remotes (Tunnels/SSH)", then inside "SSH" add a new SSH and write `ubuntu@ip-address`. Finally, select the configuration file inside your `home` folder.

## Setting up 

Into the SSH system run:

```bash
sudo apt update
sudo apt upgrade
sudo apt install -y build-essential linux-headers-$(uname -r) kmod
```

## Compiling drivers

All you need to compile your kernel module is to create a `Makefile` like this:

```makefile
obj-m += ldd.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
```

Where `ldd.o` is the object file of your driver.

Then:

```bash
make
```

> [!TIP] Custom makefile
>
> If you want to assign a custom name to your makefile, instead of `Makefile`, you can create a file with the extention `.mk`. You can run that using:
>
> ```bash
> make -f mymakefile.mk 
> ```

## Using kernel modules

- Load a `module` into the running kernel:

   ```bash
   sudo insmod module.ko
   ```

- Remove a `module` from the running kernel:

   ```bash
   sudo rmmod module.ko
   ```

- See the output on the kernel log:

   ```bash
   sudo dmesg
   # or
   sudo dmesg -c # it clears the buffer after its running
   ```

- Obtain the list of loaded modules:

   ```bash
   lsmod
   ```

> [!TIP] Fetching info about a kernel module
>
> Run:
> 
> ```bash
> modinfo module.ko
> ```
>
> Informations are obtained from the `ldd.mod.c` file.

### `insmod`

`insmod` execute several **ordered** tasks when called:

1. Calls `init_module` to hit the kernel that a module insertion is attempted.
2. Transfers control to the kernel:
   1. The kernel execute `sys_init_module`.
   2. It verify permissions.
   3. `load_modules` function is called, then:
      1. It checks for the sanity of the `.ko` file.
      2. Allocates the proper memory.
      3. Copies code from user space to the kernel space.
      4. Resolves symbols.
      5. Returnes a reference to the kernel module just loaded.
   4. The kernel adds the reference to a linked list that contains all the loaded modules.
3. `module_init` is executed.

In case of failure of the last point, the kernel revert all changes, then `init_module` returns a failure feedback.

### `rmmod`

`rmmod` execute several **ordered** tasks when called:

1. `rmmod` calls `delete_module()`, which hints the kernel that a module has to be removed. This function transfers the control to the kernel:
   1. The kernel executes `sys_delete_module()`, then:
      1. Checks the permission of the one requesting.
      2. Checks if any other loaded module needs the current module.
      3. Checks if the module is actually loaded.
      4. Executes the function provided in `module_exit` macro.
   2. The kernel executes `free_module()`, then:
      1. Removes references and kernel object reference.
      2. Performs an other cleanup.
      3. Unloads the module.
      4. Changes the state in list.
      5. Removes it from the list and frees the memory.

## `/proc/` filesystem

The `/proc/` folder is a pseudo-filesystem used to access to process information provided by the kernel.
\
Files inside this folder are used to give the possibility to the user to use loaded drivers via softwares that use files inside `/proc/` folder. It works like a big interface.

> Other info at [Procfs - Wikipedia](https://en.wikipedia.org/wiki/Procfs)

> [!IMPORTANT] 
>
> It doesn't hold hard disk's memory because it is a virtual filesystem composed by virtual files, which are empty (Try to `cat` a file part of the `/proc/` folder).

All primitives are defined inside the `linux/include/linux/proc.fs.h` and it is possible to provide custom implementations because those are all function pointers.

> [!TIP] Gathering information about access
> 
> Remember that a kernel module keeps track of how many times, for example, has been requested to read from its file.

### Create a file entry

Since `/proc/` contains a virtual filesystem, the file that we will create is a virtual file, so it is an empty file, called "**file entry**".

*Example*:

If we have an application that take photos, then, when we click on the pic button, the software try to read from the corrispondent `proc` file entry. So, the kernel throught this file can reach the read function implementation of the corrispondent driver, then it reads information from the camera. Finally, the kernel puts informations into the kernel memory, then it passes to the user's software, that will process the photo.

> [!NOTE]
>
> The kernel doesn't keep any user data because it has to be safe and mostly unaffected by users. In fact, it is impossible for the user to modify the behaviour of a `/proc/` file entry because it is an empty file, so the user cannot write on that.

We need to associate `proc_ops` to our driver. `proc_ops` is a struct inside `linux/include/linux/proc.fs.h`.

```c
struct proc_dir_entry *proc_create(const char *name, 
                                    umode_t mode, 
                                    struct proc_dir_entry *parent, 
                                    const struct proc_ops *proc_ops);
```

Elements:

- `name`: the name that we want to assign to the file entry.
- `mode`: this is defined in `linux/include/linux/types.h`.
- `parent`: this is an eventual the parent entry.
- `proc_ops`: this struct contains all operations that we implemented into our driver. Available operations are listed into the definition of this data structure at `linux/include/linux/proc.fs.h`. **This parameter cannot be NULL.**

### Remove a file entry

```c
void proc_remove(struct proc_dir_entry *);
```

This function simply closes `proc_dir_entry` has to be the file entry returned by `proc_create`.

### Troubleshooting

It is frequent to fall in errors that cause half loading of a module. This means that we cannot remove our module with `rmmod` because it is detected as "in use", but at the same time the module has not been loaded correctly. 
\
In this case a solution could be rebooting the machine. This is why it is very important to do these tests on a virtual machine. 

## Implementing a read operation

Primitives' definitions are available at `linux/include/linux/proc.fs.h` inside the struct **`proc_ops`**.

Here, as an example, this is the provided definition of the read function:

```c
ssize_t (*proc_read)(struct file *, char __user *, size_t, loff_t *);
```

The interpretation is:

- `*proc_read` is the name of the `proc_ops`'s corrispondent variable. In this case, it requires a pointer to function.
- On the right side are listed function's parameters which don't have a name because it will be specified in the implementation of the function.

Here an example:

```c
// Implementation
static ssize_t mattia_read(struct file *pointer,
                    char __user *user_space_buffer, 
                    size_t count,
                    loff_t *offset) {
    printk("Read\n");

    return 0;
}

// Assignment of the implemented function to the proc_ops field
struct proc_ops driver_proc_ops = {
    .proc_read = mattia_read
};
```

> [!WARNING] static
> 
> All primitives implementations must be `static`. 

> [!NOTE] Who passes paramters to these functions?
>
> All these parameters are passed by the kernel.

After compiling and running, we can see that, if we try to `cat` the corrispondent file entry, then in the kernel log will appear the output of the `printk` inside the `mattia_read` function implementation. So, it means that the read operation works.
\
The `printk` writes on the kernel buffer, but it is possible to pass informations to the user, if we want.

## Passing data to the user

It is possible to pass data to the user, for example, in a read operation using its second parameter:

```c
ssize_t (*proc_read)(struct file *, char __user *, size_t, loff_t *);
```

`char __user *` is a buffer that contains information to be passed to the user space.

Then, we need an API to write data into that user space buffer:

```c
copy_to_user(dst, src, n_bytes);
```

When programs like `cat` try to read from a file entry, they use the return value of the driver's read function. So, a possible solution could be set the return value equals to the lenght of information to be read. Unfortunately, it cause a infinite read loop because `cat` cannot reach the `EOF`. The only way to limit it is to use the last parameter of the read function:

```c
ssize_t (*proc_read)(struct file *, char __user *, size_t, loff_t *);
```

`loff_t *` is an offset that specifies where to start to read data from the `user_space_buffer`. It is `0` by default. 
\
If we update the offset of the lenght of read data, then we can stop the infinite read loop with a simple `if`, which makes the function to return `0` instead of `msg_len`. So, `cat` stops read because it recieves `0`.

*Example*:

```c
static ssize_t mattia_read(struct file *pointer,
                           char __user *user_space_buffer, 
                           size_t count,
                           loff_t *offset) {
   char msg[] = "User space info\n";
   size_t msg_len = strlen(msg);

   // Here a variable to catch copy_to_user return value to avoid warning
   int return_value;

   // Check if the offset is equals or greater than the message that we want to read
   if (*offset >= msg_len) {
      // If true, then we return 0, so that the reader is informed that there is nothing to read
      return 0;
   }

   // Here we copy data to the user_space_buffer
   return_value = copy_to_user(user_space_buffer, msg, msg_len);

   // Here we update offset of the read message lenght 
   *offset += msg_len;

   printk("Read!\n");
   // We return the lenght of the message that the reader program can read from the user_space_buffer
   return msg_len;
}
```

## Complete working module

Here a [complete working module](./ldd/ldd.c) example.

## Conclusion

Now, we are able to write a device driver for a virtual device. So, for example, we can create a virtual system that manages information and works like a memory where to read or write information from any program on the system.
