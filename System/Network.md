# Network basic utilities

## `ip`

List of all network interfaces and some information about their state:

```shell
ip link
```

## `ping`

To know if the connection works, try to communicate with some secure (well known, like google.com) website:

```shell
ping <secure_website>
```

# Manually connect to internet

## Bare bones

It is possible that you are completely stucked because you haven't network manager or any utilities like iwctl, netplan, etc. installed on your system. There is a solution to obtain internet connection using systemd.

> [!NOTE]
>
> In this example we will use wlan0 as interface. Check name of your interface using `ip addr show`.

### Configure wpa_supplicant

Generate configuration file:

```shell
wpa_passphrase "<network-name>" "<password>" | sudo tee /etc/wpa_supplicant/wpa_supplicant.conf
```

Start connection:

```shell
sudo wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf
```

Check wpa_cli status:

```shell
sudo wpa_cli status
```

It should print `CTRL-EVENT-CONNECTED`.

### Configure systemd-networkd

Create configuration file for wlan0:

```shell
sudo vim /etc/systemd/network/10-wlan.network
```

And write this inside it:

```yaml
[Watch]
Name=wlan0

[Network]
DHCP=yes
```

Enable systemd for network and DNS:

```shell
sudo systemctl enable --now systemd-networkd
sudo systemctl enable --now systemd-resolved
```

### Check connection status

```shell
ping google.com
```

It should work.