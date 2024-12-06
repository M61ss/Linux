# Miscellaneous notes <!-- omit from toc -->

- [ROS1 vs ROS2](#ros1-vs-ros2)
  - [Broker](#broker)
- [Testing](#testing)

## ROS1 vs ROS2

### Broker

It is important to underline a big difference between ROS1 and ROS2.

Both ROS1 and ROS2 have publishers and subscribers. ROS1 has a third entity, which is broker. It works as mediator between publishers and subscribers. 

The broker collect data published and deliver them to subscribers. It was easy to synchronize subscribers thanks to it. Anyway, in ROS2 it no longer exists because broker is unefficient. There are other primitives which provides equivalent synchronization features, saving performance.

A nice example of syncronization between two subscriber in ROS2 is available [here](./packages-examples/adder-correct/src/sum.cpp).

## Testing
