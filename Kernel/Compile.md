# How to compile the Linux kernel

Install whatever source code you want.

> [!NOTE] Raspberry Pi
>
> [Here](https://www.raspberrypi.com/documentation/computers/linux_kernel.html#natively-build-a-kernel) the complete documentation.
> \
> For cross-compile see [this](https://www.raspberrypi.com/documentation/computers/linux_kernel.html#cross-compile-the-kernel).

## Dependencies

```shell
sudo apt-get install git fakeroot build-essential ncurses-dev xz-utils libssl-dev bc flex libelf-dev bison
```

## Configuration

Move to the root directory of the source code kernel folder.

Then, there are 2 options:

1. Copy `.config` file of your kernel:
   ```shell
   cp -v /boot/config-$(uname -r) .config
   ```

2. Create a new `.config` file:
   ```shell
   make menuconfig
   ```
   Then, select your preferences from the GUI.

3. Create a default `.config` file:
   ```shell
   make defconfig
   ```

## Compile

```shell
make -jn
```

Where `n` is the number of core of parallellization dedicated to compile.

## Clean

Remove compile output (also `.config` file):

```shell
make clean
```

For a depth clean:

```shell
make mrproper
```
