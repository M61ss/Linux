# ROS2 - Coding modules <!-- omit from toc -->

- [Build](#build)
  - [Install `colcon`](#install-colcon)
  - [External dependencies: `rosdep`](#external-dependencies-rosdep)
  - [Setup workspace](#setup-workspace)
  - [Build with `colcon`](#build-with-colcon)
  - [Source underlay and overlay](#source-underlay-and-overlay)
  - [Test](#test)
  - [Create a package based on template](#create-a-package-based-on-template)
  - [`colcon_cd`](#colcon_cd)
  - [colcon tab completion](#colcon-tab-completion)
- [Package](#package)
  - [Content](#content)
  - [Create a new package](#create-a-new-package)

## Build

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

## Package

### Content

The minimum content of a ROS package is:

- **CMake**:
  
  - `CMakeLists.txt` file that describes how to build the code within the package.
  - `include/<package_name>` directory containing the public headers for the package.
  - `package.xml`: file containing meta information about the package.
  - `src`: directory containing the source code for the package.

- **Python**:
  
  - `package.xml`: file containing meta information about the package.
  - `resource/<package_name>`: marker file for the package.
  - `setup.cfg`: is required when a package has executables, so ros2 run can find them.
  - `setup.py`: containing instructions for how to install the package.
  - `<package_name>`: a directory with the same name as your package, used by ROS 2 tools to find your package, contains `__init__.py`.

### Create a new package

- **CMake**:
  
  ```shell
  ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>
  ```

- **Python**:
  
  ```shell
  ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>
  ```