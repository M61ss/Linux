# Commands <!-- omit from toc -->

- [Source setup files](#source-setup-files)
- [Obtaining list of running components](#obtaining-list-of-running-components)
    - [Options](#options)
- [Fetching informations](#fetching-informations)
    - [Options](#options-1)
- [rqt](#rqt)
  - [Run rqt](#run-rqt)
  - [rqt graph](#rqt-graph)
- [Node](#node)
  - [Run node](#run-node)
  - [Run node with a different name](#run-node-with-a-different-name)
- [Topic](#topic)
  - [See data published on a topic](#see-data-published-on-a-topic)
  - [Knowing structure of input](#knowing-structure-of-input)
  - [Publish data on a topic from command line](#publish-data-on-a-topic-from-command-line)
    - [Options](#options-2)
  - [Publication rate](#publication-rate)
  - [Bandwith used by a topic](#bandwith-used-by-a-topic)
  - [Find a topic of a given type](#find-a-topic-of-a-given-type)
- [Service](#service)
  - [Service type](#service-type)
  - [Find a service of a given type](#find-a-service-of-a-given-type)
  - [Knowing structure of input](#knowing-structure-of-input-1)
  - [Call a service](#call-a-service)
- [Documentation](#documentation)

## Source setup files

```shell
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
```

## Obtaining list of running components

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

#### Options

- `-v`: means "verbose". It provides more details.

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

### Knowing structure of input

If we want to learn what structure of data the message (input) expects and some other details:

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
- `-w n`: means “wait for n matching subscriptions”. This is needed if we want to wait to publish data until there are `n` subscription to the topic. `n` must be a whole positive number.
- `--rate n`: specifies the [publication rate](#publication-rate) of data on that topic. `n` represents the rate in Hz.

### Publication rate

You can also view the rate, measured in Hz, at which data is published using:

```shell
ros2 topic hz <topic_name>
```

### Bandwith used by a topic

The bandwidth used by a topic can be viewed using:

```shell
ros2 topic bw <topic_name>
```

It returns the bandwidth utilization and number of messages being published to the topic.

### Find a topic of a given type

To list a list of available topics of a given type use:

```shell
ros2 topic find <topic_type>
```

## Service

### Service type

To find out the type of a service, use the command:

```shell
ros2 service type <service_name>
```

### Find a service of a given type

If you want to find all the services of a specific type, you can use the command:

```shell
ros2 service find <type_name>
```

### Knowing structure of input

If we want to learn what is the expected input structure:

```shell
ros2 interface show <type_name>
```

### Call a service

You can call a service using:

```shell
ros2 service call <service_name> <service_type> <arguments>
```

## Documentation

[ROS 2 - Humble](https://docs.ros.org/en/humble/index.html)