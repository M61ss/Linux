# Change file manager <!-- omit from toc -->

- [Identify and locate the file manager](#identify-and-locate-the-file-manager)
- [Set a new file manager as default](#set-a-new-file-manager-as-default)

## Identify and locate the file manager

Determine the default file manager:

```shell
xdg-mime query default inode/directory
```

In order to change the default manager, you have to know what desktop entry file name is called. 
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