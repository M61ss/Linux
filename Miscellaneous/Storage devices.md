# Find all storage devices

TODO: https://www.baeldung.com/linux/find-all-storage-devices

# Format an usb device

## Unmount the usb device

Search your usb device inside the list prompted by:

```shell
lsblk
```

> [!IMPORTANT]
>
> It is important to correctly read the output:
>
> ```shell
> sdb       # device
> └─sdb1    # partition
> ```
>
> Could be multiple partitions for the same disk.

Then, unmount all mounted partitions using this command:

```shell
umount /dev/<partition_name>
```

## Create a new partition

Open the device in `fdisk`:

```shell
sudo fdisk /dev/<device_name>
```

### Basic procedure

**Options could be a lot. Following instructions are only an example.**

1. Select partition table layout: type `g` for GPT.
2. Create a new partition: type `n` to create a new partition and accept all default settings.
3. Write changes on disk: type `w`.

## Format the device

### ext4

```shell
sudo mkfs.ext4 -L <label> -m <reserved_block_percentage> -b <block_size> /dev/<partition_name>
```

> [!TIP]
>
> By default, `<reserved_block_percentage> = 1` and `<block_size> = 4096`.

### NTFS

```shell
sudo mkfs.ntfs --fast --label <label> /dev/<partition_name>
```

`--fast` is useful to perform a quick format. 

### FAT nad VFAT

- **FAT**:
  
  ```shell
  sudo mkfs.fat -F 32 -n <label> /dev/<partition_name>
  ```

- **VFAT**:

  ```shell
  sudo mkfs.vfat -F 32 -n "<label>" /dev/<partition_name>
  ```

> [!IMPORTANT]
>
> If you want, you can create FAT12 or FAT16 replacing `-F 32` respectively with `-F 12` or `-F 16`.

### exFAT

```shell
sudo mkfs.exfat -n <label> /dev/<partition_name>
```

### Btrfs

```shell
sudo mkfs.btrfs -L <label> /dev/<partition_name>
```

### XFS

```shell
sudo mkfs.xfs -L <label> /dev/<partition_name>
```
