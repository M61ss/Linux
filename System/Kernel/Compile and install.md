# Compile the Linux kernel

Install whatever source code you want.

> [!NOTE] Raspberry Pi
>
> [Here](https://www.raspberrypi.com/documentation/computers/linux_kernel.html#natively-build-a-kernel) the complete documentation.
> \
> For cross-compile see [this](https://www.raspberrypi.com/documentation/computers/linux_kernel.html#cross-compile-the-kernel).

## Dependencies

```shell
sudo apt install git fakeroot build-essential ncurses-dev xz-utils libssl-dev bc flex libelf-dev bison pkg-config libdw-dev
```

## Configuration

Move to the root directory of the source code kernel folder.

Then, there are 3 options:

1. Copy `.config` file of your kernel:
   ```shell
   cp -v /boot/config-$(uname -r) .config
   make olddefconfig
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
make -j$(nproc)
```

## Clean

Remove compile output:

```shell
make clean
```

For a depth clean (it removes also `.config` file):

```shell
make mrproper
```

# Install the Linux kernel

```shell
sudo make modules_install install
```

Remeber to update grub (or whatever bootloader you use) in order to see the new boot entry at the next reboot:

```shell
sudo update-grub2
```
