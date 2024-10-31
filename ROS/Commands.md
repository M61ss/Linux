# Commands <!-- omit from toc -->

- [Source setup files](#source-setup-files)
- [Obtaining list of running elements](#obtaining-list-of-running-elements)
    - [Options](#options)
- [Fetching informations](#fetching-informations)
- [rqt](#rqt)
  - [Run rqt](#run-rqt)
  - [rqt graph](#rqt-graph)
- [Node](#node)
  - [Run node](#run-node)
  - [Run node with a different name](#run-node-with-a-different-name)
- [Topic](#topic)
  - [See data published on a topic](#see-data-published-on-a-topic)
  - [Learn details about a specific msg](#learn-details-about-a-specific-msg)
  - [Publish data on a topic from command line](#publish-data-on-a-topic-from-command-line)
    - [Options](#options-1)
- [Documentation](#documentation)

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

#### Options

- `-t`: will return the same list of topics, this time with the topic type appended in brackets.

## Fetching informations

```shell
ros2 node info <node_name>
ros2 topic info <topic_name>
ros2 service info <service_name>
ros2 action info <action_name>
```

## rqt

### Run rqt

To run rqt:

```shell
rqt
```

### rqt graph

To visualize the changing nodes and topics and connections between them, run:

```shell
rqt_graph
```

Anyway, you can open rqt, then select from the menu: Plugins > Introspection > Node Graph.

## Node

### Run node

```shell
ros2 run <package_name> <node_name>
```

### Run node with a different name

```shell
ros2 run <package_name> <node_name> --ros-args --remap __node:=<new_node_name>
```

## Topic

### See data published on a topic

```shell
ros2 topic echo <topic_name>
```

It keeps running showing data published on that topic as soon as data are entered.

### Learn details about a specific msg

If we want to learn what structure of data the message expects and some other details:

```shell
ros2 interface show <msg_type>
```

`<msg_type>` is obtained [fetching informations](#fetching-informations) about a topic.

### Publish data on a topic from command line

There is the possibility to publish data on a topic directly from the command line:

```shell
ros2 topic pub <topic_name> <msg_type> "<args>"
```

> [!WARNING]
>
> It is implicit that data will be continuously published on the topic. To avoid this behaviour we need to use [--once](#options-1).

*Example*:

```shell
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

Inside double quotes there are data formatted according [msg details](#learn-details-about-a-specific-msg).

#### Options

- `--once`: data are published on the topic only once rather than continuously.
- `-w n`: is an optional argument meaning “wait for n matching subscriptions”. This is needed if we want to wait to publish data until there are `n` subscription to the topic. `n` must be a whole positive number.

## Documentation

[ROS 2 - Humble](https://docs.ros.org/en/humble/index.html)