# ROS2 Humble - CLI reference <!-- omit from toc -->

- [Documentation](#documentation)
- [Source setup files](#source-setup-files)
- [Listing running components: `list`](#listing-running-components-list)
- [Fetching informations: `info`](#fetching-informations-info)
- [Node](#node)
  - [Run node: `run`](#run-node-run)
  - [Run multiple nodes: `launch`](#run-multiple-nodes-launch)
- [Topic](#topic)
  - [See data published on a topic: `topic echo`](#see-data-published-on-a-topic-topic-echo)
  - [Knowing structure of input: `interface show`](#knowing-structure-of-input-interface-show)
  - [Publish data on a topic from command line: `topic pub`](#publish-data-on-a-topic-from-command-line-topic-pub)
  - [Publication rate: `topic hz`](#publication-rate-topic-hz)
  - [Bandwith used by a topic: `topic bw`](#bandwith-used-by-a-topic-topic-bw)
  - [Find a topic of a given: `topic find`](#find-a-topic-of-a-given-topic-find)
  - [Record data published on a topic: `bag`](#record-data-published-on-a-topic-bag)
- [Service](#service)
  - [Service type: `service type`](#service-type-service-type)
  - [Find a service of a given type: `service find`](#find-a-service-of-a-given-type-service-find)
  - [Knowing structure of input: `interface show`](#knowing-structure-of-input-interface-show-1)
  - [Call a service: `service call`](#call-a-service-service-call)
- [Parameter](#parameter)
  - [Type and value of a parameter: `param get`](#type-and-value-of-a-parameter-param-get)
  - [Change the value of a parameter: `param set`](#change-the-value-of-a-parameter-param-set)
  - [View node's parameters: `param dump`](#view-nodes-parameters-param-dump)
  - [Load parameters from a file: `param load`](#load-parameters-from-a-file-param-load)
- [Action](#action)
  - [Knowing structure of input: `interface show`](#knowing-structure-of-input-interface-show-2)
  - [Send action goal: `action send_goal`](#send-action-goal-action-send_goal)

## Documentation

[ROS 2 - Humble](https://docs.ros.org/en/humble/index.html)

## Source setup files

```shell
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
```

## Listing running components: `list`

```shell
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

#### Options <!-- omit from toc -->

- `-t` (NOT for node): will return not only the list, but also with the corresponding type appended in brackets.

## Fetching informations: `info`

```shell
ros2 node info <node_name>
ros2 topic info <topic_name>
ros2 service info <service_name>
ros2 action info <action_name>
```

#### Options <!-- omit from toc -->

- `-v`: means "verbose". It provides more details.

## Node

### Run node: `run`

```shell
ros2 run <package_name> <node_name>
```

#### Options <!-- omit from toc -->

- `--ros-args --params-file <file_name>`: start the node using [parameters from the specified file](#load-parameters-from-a-file).
- `--ros-args --remap __node:=<new_node_name>`: it runs the node with a different name.
- `--ros-args --log-level <LEVEL>`: sets the default [logger level](rqt.md#logger-levels).

### Run multiple nodes: `launch`

```shell
ros2 launch <package_name> <file>
```

Where `<file>` could be, for instance, a `.py` script.

## Topic

### See data published on a topic: `topic echo`

```shell
ros2 topic echo <topic_name>
```

It keeps running showing data published on that topic as soon as data are entered.

### Knowing structure of input: `interface show`

If we want to know what structure of data the message (input) expects and some other details:

```shell
ros2 interface show <msg_type>
```

`<msg_type>` is obtained [fetching informations](#fetching-informations) about a topic.

### Publish data on a topic from command line: `topic pub`

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

#### Options <!-- omit from toc -->

- `--once`: data are published on the topic only once rather than continuously.
- `-w n`: means “wait for n matching subscriptions”. This is needed if we want to wait to publish data until there are `n` subscription to the topic. `n` must be a whole positive number.
- `-r n`: specifies the [publication rate](#publication-rate) of data on that topic. `n` represents the rate in Hz.

### Publication rate: `topic hz`

You can also view the rate, measured in Hz, at which data is published using:

```shell
ros2 topic hz <topic_name>
```

### Bandwith used by a topic: `topic bw`

The bandwidth used by a topic can be viewed using:

```shell
ros2 topic bw <topic_name>
```

It returns the bandwidth utilization and number of messages being published to the topic.

### Find a topic of a given: `topic find`

To list a list of available topics of a given type use:

```shell
ros2 topic find <topic_type>
```

### Record data published on a topic: `bag`

To record the data published to one or more topics create a dedicated folder to store the output, move into it from the terminal, then use the command syntax:

```shell
ros2 bag record -o <destination_file> <topic1_name> <topic2_name> <topicN_name>
```

Where `-o` specifies the name of `<destination_file>` 

You can see details about your recording by running:

```shell
ros2 bag info <bag_file_name>
```

To use data stored in a bag use:

```shell
ros2 bag play <bag_file_name>
```

> [!WARNING]
>
> Before running the above command, be sure to stop all nodes which publish on the topic that recieve data from the `<bag_file_name>`. 

## Service

### Service type: `service type`

To find out the type of a service, use the command:

```shell
ros2 service type <service_name>
```

### Find a service of a given type: `service find`

If you want to find all the services of a specific type, you can use the command:

```shell
ros2 service find <type_name>
```

### Knowing structure of input: `interface show`

If we want to learn what is the expected input structure:

```shell
ros2 interface show <service_type>
```

### Call a service: `service call`

You can call a service using:

```shell
ros2 service call <service_name> <service_type> <arguments>
```

## Parameter

### Type and value of a parameter: `param get`

To display the type and current value of a parameter, use the command:

```shell
ros2 param get <node_name> <parameter_name>
```

### Change the value of a parameter: `param set`

To change a parameter's value at runtime, use the command:

```shell
ros2 param set <node_name> <parameter_name> <value>
```

> [!WARNING]
>
> `<value>` must be in YAML format.

### View node's parameters: `param dump`

You can view all of a node's current parameter values by using the command:

```shell
ros2 param dump <node_name>
```

### Load parameters from a file: `param load`

You can load parameters from a file to a currently running node using the command:

```shell
ros2 param load <node_name> <parameter_file>
```

> [!WARNING]
>
> `<parameter_file>` must contain values in YAML format.

## Action

### Knowing structure of input: `interface show`

If we want to learn what is the expected input structure:

```shell
ros2 interface show <action_type>
```

### Send action goal: `action send_goal`

Send an action goal from the command line with the following syntax:

```shell
ros2 action send_goal <action_name> <action_type> <values>
```

> [!WARNING]
>
> `<values>` must be in YAML format.

#### Options <!-- omit from toc -->

- `--feedback`: see the feedback of the action goal.
