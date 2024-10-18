# Terminal emulators

## Guake

```bash
sudo apt install guake
```

## Tilix

Powerful multisession terminal with well organized GUI.

```bash
sudo apt install tilix
```

### Issue when opening "preferences"

When Tilix is installed, an issue occurrs opening preferences. To fix this:

- Run:
  
  ```bash
  ln -s /etc/profile.d/vte-2.91.sh /etc/profile.d/vte.sh
  ```

- Open `~/.bashrc` and add:
  
  ```bash
  # fix for VTE (Virtual Terminal Emulator) to avoid errors in Tilix's features
  if [ $TILIX_ID ] || [ $VTE_VERSION ]; then
      source /etc/profile.d/vte.sh
  fi
  ```

## Cool retro terminal

```bash
sudo apt install cool-retro-term
```

## Konsole

```bash
sudo apt install konsole
```

## Gnome terminal

```bash
sudo apt install gnome-terminal
```

## Kitty

This terminal depends on GPU, so takes the load off the CPU when working on it.

```bash
sudo apt install kitty
```

## Xterm

```bash
sudo apt install xterm
```