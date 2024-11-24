# Some basic system utilities

- [Some basic system utilities](#some-basic-system-utilities)
  - [Keymaps and kbd layout: `localectl` and `loadkeys`](#keymaps-and-kbd-layout-localectl-and-loadkeys)
    - [Console keymaps](#console-keymaps)
    - [Keyboard layout](#keyboard-layout)
  - [Clock and timezone: `timedatectl`](#clock-and-timezone-timedatectl)
  - [Console font: `setfont`](#console-font-setfont)
  - [Hw information: `lshw`](#hw-information-lshw)
  - [Change root password: `passwd`](#change-root-password-passwd)

## Keymaps and kbd layout: `localectl` and `loadkeys`

Useful utilities to change keymaps and keyboard layout.

### Console keymaps

You can get available console keymaps running:

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

## Clock and timezone: `timedatectl`

It prompts detailed informations about the current system clock:

```shell
timedatectl
```

You can also use this as a simple cli utility to setup the system clock and timezone.

## Console font: `setfont`

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

## Hw information: `lshw`

```shell
lshw
```

"List hardware" by default lists all the hardware devices connected to the system. It is useful if combined with some of its options.
\
For instance:

```shell
lshw -class <class>
```

Filters the list and display only `<class>` devices.

## Change root password: `passwd`

You can set or change the root user password:

```shell
passwd
```

Then, follow prompted instructions.
