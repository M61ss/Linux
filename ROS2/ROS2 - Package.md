# ROS2 - Package and builing <!-- omit from toc -->

- [Package](#package)
  - [Build tools](#build-tools)
    - [Install `colcon`](#install-colcon)
    - [External dependencies: `rosdep`](#external-dependencies-rosdep)
    - [Setup workspace](#setup-workspace)
    - [Build with `colcon`](#build-with-colcon)
    - [Source underlay and overlay](#source-underlay-and-overlay)
    - [Test](#test)
    - [Create a package based on template](#create-a-package-based-on-template)
    - [`colcon_cd`](#colcon_cd)
    - [colcon tab completion](#colcon-tab-completion)
  - [Create a new package (C++)](#create-a-new-package-c)
    - [Directory tree](#directory-tree)
    - [CMake](#cmake)
    - [`package.xml`](#packagexml)
    - [Build a package](#build-a-package)

# Package

## Build tools

### Install `colcon`

```shell
sudo apt install python3-colcon-common-extensions
```

### External dependencies: `rosdep`

> [!WARNING]
>
> Maybe you are using an environment which does not contain `rosdep`, ROS dependency manager. You can find info [here](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html).

From the root of your workspace, run the following command:

```shell
rosdep install -i --from-path src --rosdistro humble -y
```

### Setup workspace

Create a workspace and move inside it:

```shell
mkdir -p ~/root
cd ~/root
# root is an arbitrary name
```

Create the `src` folder, which will contain the source code:

```shell
mkdir src
```

### Build with `colcon`

When you are ready, in the root of the workspace, run:

```shell
colcon build
```

After the build is finished, we should see the `build`, `install`, and `log` directories.

#### Options <!-- omit from toc -->

- `--packages-up-to`: builds the package you want, plus all its dependencies, but not the whole workspace (saves time).
- `--symlink-install`: saves you from having to rebuild every time you tweak python scripts.
- `--event-handlers`: console_direct+ shows console output while building (can otherwise be found in the log directory).
- `--executor`: sequential processes the packages one by one instead of using parallelism.
- `--packages-select <package_name>`: allows to build only `<package_name>`.

### Source underlay and overlay

To source your overlay move to your root folder and run:

```shell
source install/local_setup.bash
```

> [!NOTE] 
>
> It is an overlay because it lays on top of the underlay, which is ROS. As always ROS must be [sourced](ROS2%20-%20CLI.md#source-setup-files). 

### Test

To run tests for the packages we just built, run the following:

```shell
colcon test
```

> [!TIP]
>
> If you do not want to build a specific package, then place an empty file named `COLCON_IGNORE` in the directory and it will not be indexed.

#### Options <!-- omit from toc -->

- `--cmake-args -DBUILD_TESTING=0`: avoid configuring and building tests in CMake packages.
- `--packages-select YOUR_PKG_NAME --ctest-args -R YOUR_TEST_IN_PKG`: run a single particular test from a package.

### Create a package based on template

To create a new package based on a template, run:

```shell
ros2 pkg create
```

### `colcon_cd`

Allows you to quickly change the current working directory of your shell to the directory of a package, instead of change shell sourcing every time.
\
See the [documentation](https://colcon.readthedocs.io/en/released/user/installation.html#quick-directory-changes).

### colcon tab completion

The colcon command supports command completion for bash and bash-like shells. The `colcon-argcomplete` package must be installed and [some setup](https://colcon.readthedocs.io/en/released/user/installation.html#enable-completion) may be required to make it work.

## Create a new package (C++)

```shell
ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>
```

### Directory tree

A ROS2 package has a structure like this:

- `/include`: it contains headers. Headers created from scratch for the package are placed in `/include/<package_name>`. 
- `/src`: it contains all source code.

Then, in the folder outside the package, which likely contains many packages, in addiction to packages, should exists two other folders:

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

### CMake

Basically, `CMake` it is a `make` wrapper. It automatically creates `Makefile` following `CMakeLists.txt` instructions.

### `package.xml`

It is important to focus on `<depend>` tags. They contain requirements to build the package.

### Build a package

1. Before to build, add necessary dependencies use `rosdep` (details [here](./ROS2%20-%20Code.md#external-dependencies-rosdep)).
2. Then, build using `colcon` (details [here](./ROS2%20-%20Code.md#build-with-colcon)).

> [!IMPORTANT] source
>
> Remember to source the setup file of your installation folder ([overlay](./ROS2%20-%20Code.md#source-underlay-and-overlay)).