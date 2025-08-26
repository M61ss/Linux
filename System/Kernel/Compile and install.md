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

### Certification issues

On many distros it is possible to recieve an error like this:

```shell
make[3]: *** No rule to make target 'debian/canonical-certs.pem', needed by 'certs/x509_certificate_list'.  Stop.
```

When developing, the simpler solution is to disable module signature verification using menuconfig. Edit the .config file using the dedicated script:

```shell
scripts/config --disable SYSTEM_TRUSTED_KEYS
scripts/config --disable SYSTEM_REVOCATION_KEYS
scripts/config --disable MODULE_SIG
```

Anyway, it is also possible to provide a fake certificate, maybe to test correct working of system, generating a PEM cert:

```shell
openssl req -new -nodes -utf8 -sha256 -days 36500 -batch -x509 -out certs/signing_key.pem -keyout certs/signing_key.pem
```

Then, in .config add:

```shell
CONFIG_SYSTEM_TRUSTED_KEYS="certs/signing_key.pem"
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
