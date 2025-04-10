# Grub

## Configuration file

Keep the [official documentation](https://www.gnu.org/software/grub/manual/grub/grub.html) as reference to edit GRUB's configuration files.

### Reload GRUB

After editing whatever GRUB configuration file, then you have to reload GRUB:

- **Ubuntu**:
  
  ```shell
  sudo update-grub
  ```

- **Fedora**:
  
  ```shell
  grub2-mkconfig -o /boot/grub2/grub.cfg
  ```

## Grub customizer

> [!WARNING]
>
> **DEPRECATED** by all distros. It works (april 2025) on Ubuntu.

1. ```bash
   sudo add-apt-repository ppa:danielrichter2007/grub-customizer
   ```

2. ```bash
   sudo apt update
   ```

3. ```bash
   sudo apt install grub-customizer
   ```