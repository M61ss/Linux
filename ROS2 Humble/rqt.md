# ROS2 Humble - rqt

- [ROS2 Humble - rqt](#ros2-humble---rqt)
  - [Run rqt](#run-rqt)
  - [rqt graph](#rqt-graph)
  - [rqt console](#rqt-console)
    - [Logger levels](#logger-levels)

## Run rqt

To run rqt:

```shell
rqt
```

## rqt graph

To visualize the changing nodes and topics and connections between them, run:

```shell
rqt_graph
```

Anyway, you can open rqt, then select from the menu: Plugins > Introspection > Node Graph.

## rqt console

Start `rqt_console` in a new terminal with the following command:

```shell
ros2 run rqt_console rqt_console
```

### Logger levels

- `Fatal`: messages indicate the system is going to terminate to try to protect itself from detriment.
- `Error`: messages indicate significant issues that won't necessarily damage the system, but are preventing it from functioning properly.
- `Warn`: messages indicate unexpected activity or non-ideal results that might represent a deeper issue, but don't harm functionality outright.
- `Info`: messages indicate event and status updates that serve as a visual verification that the system is running as expected.
- `Debug`: messages detail the entire step-by-step process of the system execution.

The default level is `Info`, so only `Debug` is excluded from logging.
