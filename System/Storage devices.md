# Index <!-- omit from toc -->

- [Find all storage devices](#find-all-storage-devices)
  - [Use `proc` filesystem](#use-proc-filesystem)
  - [`fdisk`](#fdisk)
  - [`lsblk`](#lsblk)
  - [`lshw`](#lshw)
  - [`parted`](#parted)
  - [`sfdisk`](#sfdisk)
- [Partition storage device](#partition-storage-device)
  - [Unmount the device](#unmount-the-device)
  - [Create a new partition](#create-a-new-partition)
    - [Basic procedure](#basic-procedure)
  - [Format the device](#format-the-device)
    - [ext4](#ext4)
    - [NTFS](#ntfs)
    - [FAT nad VFAT](#fat-nad-vfat)
    - [exFAT](#exfat)
    - [Btrfs](#btrfs)
    - [XFS](#xfs)

# Find all storage devices

## Use `proc` filesystem

The simplest way to list devices and their partitions is to run:

```shell
cat /proc/partitions
```

Learn more about `proc` [here](Kernel/Drivers/proc%20drivers.md).

## `fdisk`

```shell
sudo fdisk -l
```

It prompts the complete list of devices with **detailed** informations.

## `lsblk`

```shell
lsblk
```

"List of blocks" lists all the block storage devices attached to the system in a very clear **hierarchy**.

## `lshw`

```shell
lshw -class disk
```

"List hardware" by default lists all storage devices connected to the system.

## `parted`

```shell
sudo parted -l
```

Works like [`fdisk`](#fdisk).

## `sfdisk`

```shell
sudo sfdisk -l
```

It is an advanced version of [`fdisk`](#fdisk).

# Partition storage device

## Unmount the device

Search your device inside the list prompted by:

```shell
lsblk
```

> [!IMPORTANT]
>
> It is important to correctly read the output. For instance:
>
> ```shell
> sdb       # device
> └─sdb1    # partition 1
> └─sdb2    # partition 2
> ...
> └─sdb2    # partition n
> ```

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
   
   > [!WARNING]
   >
   > This step erases all device partitons. If you only want to create a new partition skip this.

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
