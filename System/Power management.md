# Power consumption analyzers

## powerstat

### Installation

```shell
sudo apt install powerstat
```

### Usage

If you need to quickly check system power consumption, you can simply run:

```shell
powerstat 60
```

For other usage, refer to its documentation.

# Power managers

## tlp (recommended)

> [!IMPORTANT]
> 
> This tutorial is for Ubuntu.

### Installation

```shell
sudo apt install tlp tlp-rdw
```

### Activation

You need to mask and stop the default power manager:

```shell
sudo systemctl mask power-profiles-daemon.service
sudo systemctl stop power-profiles-daemon.service
```

Then, you can activate tlp:

```shell
sudo systemctl enable tlp
sudo systemctl start tlp
```

Reboot the computer and all should work well.

## tuneD

### Installation

```bash
sudo apt install tuned
```

### Usage

```bash
sudo systemctl start tuned
```

If you want to start tuneD everytime the system boots:

```bash
sudo systemctl enable tuned
```

> [!WARNING] 
>
> If at boot time tuneD doesn't start, consider to disable default Ubuntu power profiles:
>
> ```bash
> sudo systemctl mask power-profiles-daemon
> ```

Select a profile:

```bash
tuned-adm profile profile_name
```

You can use recommended settings for your system:

```bash
tuned-adm recommend
```

To check if it works correctly, run the following command, making sure it matches the profile you selected:

```bash
tuned-adm active
```

View available profiles:

```bash
tuned-adm list
```

### GUI version

```bash
sudo apt install curl -y && curl https://raw.githubusercontent.com/FrameworkComputer/tuned-gui/main/intel-13thGen.sh | bash
```

Then, reboot the system.

Info: 

- [GitHub - tuneD](https://github.com/FrameworkComputer/tuned-gui/tree/main?tab=readme-ov-file#tuned-gui)