# ROS2 - Coding modules <!-- omit from toc -->

- [`colcon`](#colcon)
  - [Build](#build)
  - [Source the environment](#source-the-environment)
  - [Test](#test)
  - [Create a package based on template](#create-a-package-based-on-template)
  - [`colcon_cd`](#colcon_cd)
  - [colcon tab completion](#colcon-tab-completion)

## `colcon`

```shell
sudo apt install python3-colcon-common-extensions
```

### Build

Create a workspace and move inside it:

```shell
mkdir -p ~/ros2_ws
cd ~/ros2_ws
```

Create the `src` folder, which will contain the source code:

```shell
mkdir src
```

When you are ready, in the root of the workspace, run:

```shell
colcon build
```

After the build is finished, we should see the `build`, `install`, and `log` directories.

#### Options <!-- omit from toc -->

- `--symlink-install`: allows the installed files to be changed by changing the files in the source space.

### Source the environment

Before you can use any of the installed executables or libraries, you will need to add them to your path and library paths:

```shell
source install/setup.bash
```

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
