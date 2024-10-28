# Commands <!-- omit from toc -->

- [Source setup files](#source-setup-files)
- [Obtaining list of running elements](#obtaining-list-of-running-elements)
- [Fetching informations](#fetching-informations)
- [rqt](#rqt)
- [Node](#node)
  - [Run node](#run-node)
  - [Run node with a different name](#run-node-with-a-different-name)
- [Topic](#topic)

## Source setup files

```shell
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
```

## Obtaining list of running elements

```shell
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

## Fetching informations

```shell
ros2 node info <node_name>
ros2 topic info <topic_name>
ros2 service info <service_name>
ros2 action info <action_name>
```

## rqt

To run rqt:

```shell
rqt
```

## Node

### Run node

```shell
ros2 run <package_name> <node_name>
```

### Run node with a different name

```shell
ros2 run <package_name> <node_name> --ros-args --remap __node:=my_turtle
```

## Topic