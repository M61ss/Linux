# ROS2 package

## ROS2

In ROS2, a differenza di quanto succedeva in ROS1, non c'è un broker che raccolga tutti i valori pubblicati dai publisher e che li consegni ai vari subscriber.

## `package.xml`

Build tool depend: vengono definiti i requisiti per far buildare quel nodo (dipendenze). Questo è l'unico parametro di reale interesse di questo file.

## CMake

Basically, it is a `make` wrapper. It automatically creates `Makefile` following `CMakeLists.txt` instructions.

### Build

Bisogna sempre fare la build con `rosdep`, poi installare utilizzando `colcon`.
\
Infine, si fa il source al setup file prima di utilizzare i binaries.

## C++

A ROS2 package has a structure like this:

- `/include`: it contains all headers.
- `/src`: it contains all source code.

Then, in the folder which contains all packages, in addiction to package folders, should exists two folders:

- `/config`: it contains a `.yaml` file for every package. These files contain those parameters which do not change at runtime, called "static parameters". Every file in this folder should be named like this: `package_name_config.yaml`. Config files are useful to avoid to recompile all package code in case you only want to change static parameters (very common situation).
- `/launch`: it contains a `.py` file for every package. These scripts are necessary to be able to use the ROS2 utility `launch` and to load config files.

> [!IMPORTANT] main
>
> in every ROS2 package the main function should be placed in a short separeted file: `/src/main.cpp`.

> [!WARNING] Headers
>
> Beware to place any definition in source files. All of them should be placed in headers.

## Config e launch folder

Il file `config` di humble serve per evitare la rebuild da capo in caso di piccole modifiche
