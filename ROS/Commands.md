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

### Options

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
ros2 run <package_name> <node_name> --ros-args --remap __node:=my_turtle
```

## Topic

### See data published on a topic

```shell
ros2 topic echo <topic_name>
```

It keeps running showing data published on that topic as soon as data are entered.