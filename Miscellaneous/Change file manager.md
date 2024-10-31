# Change file manager <!-- omit from toc -->

- [Sources](#sources)
- [Identify and locate the file manager](#identify-and-locate-the-file-manager)
- [Set a new file manager as default](#set-a-new-file-manager-as-default)
- [Example: KDE on Ubuntu](#example-kde-on-ubuntu)
  - [Installation](#installation)
  - [Uninstallation](#uninstallation)

## Sources

- [Terminal file managers](https://www.tecmint.com/linux-terminal-file-managers/)
- [GUI file managers](https://geekflare.com/file-managers-for-linux/)

## Identify and locate the file manager

After the installation of whatever file manager, you may want to make it the default one. To do this, first determine the default file manager:

```shell
xdg-mime query default inode/directory
```

Then, you have to know what desktop entry file name is called. 
\
Desktop entry files are typically stored at a couple location on a Linux system:
- `~/.local/share/applications/` for user-specific applications.
- `/usr/share/applications/` or `/usr/local/share/applications/` for system-wide applications.

*Example*:

```shell
find /usr/share/applications/ -iname "*nautilus*"
```

It will search if there are desktop entries of Nautilus file manager at the specified path.

## Set a new file manager as default

To set a different file manager to substitute the current, run:

```shell
xdg-mime default desktop_entry_name inode/directory
```

*Example*:

```shell
xdg-mime default org.gnome.Nautilus.desktop inode/directory
```

It will set Nautilus as default file manager.

[You can check if changes has been applied.](#identify-and-locate-the-file-manager)

## Example: KDE on Ubuntu

### Installation

```shell
sudo apt install kde-full   # or whatever package you want
```

### Uninstallation

```shell
sudo apt remove kde-full
sudo apt autoremove
sudo apt remove libkf5*
sudo apt autoremove
```
