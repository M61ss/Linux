# GRUB

## Configuration files and folders

Keep the [official documentation](https://www.gnu.org/software/grub/manual/grub/grub.html) as reference to edit GRUB's configuration files.

As general rule, to edit basic function of GRUB, you should edit `/etc/default/grub` file. Anyway, you could need to check or fix information in `/boot/grub/grub.cfg` or `/etc/grub.d` for advanced GRUB customizations.
\
Themes are stored in `/boot/grub/themes`. Every theme should be contained into a folder which should contain a file named `theme.txt`.

### Reload GRUB

After editing whatever GRUB configuration file, then you have to reload GRUB:

- **Ubuntu** & **Arch**:
  
  ```shell
  sudo update-grub
  ```

- **Fedora**:
  
  ```shell
  sudo grub2-mkconfig -o /boot/grub2/grub.cfg
  ```

For the most general case, use:

```shell
sudo grub-mkconfig
```

## Grub customizer

> [!WARNING]
>
> **DEPRECATED** by all distros. It works (april 2025) on Ubuntu. **NOT** recommended.

### Installation

1. ```bash
   sudo add-apt-repository ppa:danielrichter2007/grub-customizer
   ```

2. ```bash
   sudo apt update
   ```

3. ```bash
   sudo apt install grub-customizer
   ```

### Uninstallation

1. ```shell
   sudo apt remove grub-customizer
   ```

2. ```shell
   sudo apt-add-repository --remove ppa:danielrichter2007/grub-customizer
   ```