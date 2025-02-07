# Limit battery charge (ASUS)

## Prerequisites

To know the list of device's power supply run:

```bash
ls /sys/class/power_supply
```

Then, to know if your laptop supports battery charge limit run:

```bash
ls /sys/class/power_supply/BAT*/charge_control_end_threshold
```

## Service creation

Create a file `.service` with an arbitrary name in `/etc/systemd/system`.
\
Here an example:

```bash
sudo editor /etc/systemd/system/battery-charge-threshold.service
```

Then, paste inside it these lines:

```bash
[Unit]
Description=Set the battery charge threshold
After=multi-user.target

StartLimitBurst=0
[Service]
Type=oneshot
Restart=on-failure

ExecStart=/bin/bash -c 'echo CHARGE_STOP_THRESHOLD > /sys/class/power_supply/BATTERY_NAME/charge_control_end_threshold'
[Install]
WantedBy=multi-user.target
```

> [!IMPORTANT]
>
> Replace `CHARGE_STOP_THRESHOLD` with a value of your choice between 60, 80 and 100 and replace `BATTERY_NAME` with the name of your battery obtained running `ls /sys/class/power_supply` (e.g. BAT0).

Then, enable and start the service running:

```bash
sudo systemctl enable battery-charge-threshold.service
sudo systemctl start battery-charge-threshold.service
```

## Service activation

On every startup of your device you need to run:

```bash
sudo systemctl enable battery-charge-threshold.service
sudo systemctl start battery-charge-threshold.service
```

Alternatively, add those lines to `.bashrc` of your home directory.

## Working check

To check if the service works, plug your laptop to AC, wait many seconds and run:

```bash
cat /sys/class/power_supply/BATTERY_NAME/status
```

Replace `BATTERY_NAME` with the name of your battery. 
\
If the battery has already reached the charge threshold, the command should produce the following output:

```text
Not Charging
```
