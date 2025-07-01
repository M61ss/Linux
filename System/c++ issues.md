## Clangd vscode extention doesn't work with c++

This issue incomes usually on Ubuntu. Install gcc, then run:

```shell
ls /usr/lib/gcc/x86_64-linux-gnu
```

Output shows gcc installed versions. You need to install `libstdc++-XX-dev` where `XX` stands for gcc version:

```shell
sudo apt install libstdc++-XX-dev
```

The problem should be fixed.