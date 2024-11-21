# PRIME (Ubuntu only)

The utility `prime-select` is available only for Ubuntu and it is useful to switch between integrated and discrete GPU.

### Current configuration

You can ask to prime what is the current configuration:

```bash
prime-select query
```

### Switch to integrated

```bash
sudo prime-select intel
```

### Switch to dedicated

```bash
sudo prime-select nvidia
```