# Power saving

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