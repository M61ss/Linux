# Some basic system utilities

## Keyboard layout

You can get available keyboard layout running:

```shell
localectl list-keymaps
```

You can set one of listed layout using:

```shell
loadkeys <layout>
```

## `timedatectl`

It prompts detailed informations about the current system clock:

```shell
timedatectl
```

## `setfont`

You can change the console font using:

```shell
setfont <font>
```

`/usr/share/consolefonts` contains all available console fonts. 

> [!WARNING]
> 
> Path to `consolefonts` could change between distribution: it could be a subfolder of some other folder inside `/usr/share`. 

To set the default font use:

```shell
setfont
```

## `lshw`

```shell
lshw
```

"List hardware" by default lists all the hardware devices connected to the system.

#### Options <!-- omit from toc -->

`-class <class>`: filter the list and display only `<class>` devices.
