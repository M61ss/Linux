# Some basic system utilities

## `localectl`

### Virtual console keymaps

You can get available virtual console keymaps running:

```shell
localectl list-keymaps
```

You can set one of listed keymap using:

```shell
loadkeys <keymap>
```

### Keyboard layout

You can get the current keyboard layout running:

```shell
localectl status
```

You can list available keyboard layouts running:

```shell
localectl list-locales
```

Then, you can change it using:

```shell
localectl set-locales <locale>
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
