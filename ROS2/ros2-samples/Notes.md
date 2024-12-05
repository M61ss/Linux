# ROS2 package

A ROS2 package has a structure like this:

- `/include`: it contains all headers.
- `/src`: it contains all source code.

Then, in the folder which contains all packages, in addiction to package folders, should exists two folders:

- `/config`: it contains a `.yaml` file for every package. These files contain those parameters which do not change at runtime, called "static parameters". 
\
Every file in this folder should be named like this: `package_name_config.yaml`. Config files are useful to avoid to recompile all package code in case you only want to change static parameters (very common situation).
- `/launch`: it contains a `.py` file for every package. These scripts are necessary to be able to use the ROS2 utility `launch` and to load config files.

> [!IMPORTANT] main
>
> in every ROS2 package the main function should be placed in a short separeted file: `/src/main.cpp`.

> [!WARNING] Headers
>
> Beware to place any definition in source files. All of them should be placed in headers.

The initial project tree should look like this:

```txt
root-folder
├── pkg1
|   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include
│   └── src
│       └── main.cpp
├── pkg2
|   └── ...
├── config
└── launch
```

## CMake

Basically, `CMake` it is a `make` wrapper. It automatically creates `Makefile` following `CMakeLists.txt` instructions.

## `package.xml`

It is important to focus on `<depend>` tags. They contain requirements to build the package.

## Build

1. Before to build, add necessary dependencies use `rosdep` (details [here](./../ROS2%20-%20Code.md#external-dependencies-rosdep)).
2. Then, build using `colcon` (details [here](./../ROS2%20-%20Code.md#build-with-colcon)).

> [!IMPORTANT] source
>
> Remember to source the setup file of your installation folder ([overlay](./../ROS2%20-%20Code.md#source-underlay-and-overlay)).

## ROS1 vs ROS2

### Broker

It is important to underline a big difference between ROS1 and ROS2.

Both ROS1 and ROS2 have publishers and subscribers. ROS1 has a third entity, which is broker. It works as mediator between publishers and subscribers. 

The broker collect data published and deliver them to subscribers. It was easy to synchronize subscribers thanks to it. Anyway, in ROS2 it no longer exists because broker is unefficient. There are other primitives which provides equivalent synchronization features, saving performance.
