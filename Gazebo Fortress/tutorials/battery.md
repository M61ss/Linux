\page battery Battery

The battery system keeps track of the battery charge on a robot model.

Currently, one battery per model is supported. When the battery drains
completely, all joints of the corresponding model are turned off, meaning joint
forces are set to 0.

All logic for battery consumption are encapsulated in a plugin.


## A perfect battery

An ideal battery has a constant voltage while discharging and no internal
resistance. Here's a minimum example of a perfect battery that can be added to a
model:

```{.xml}
<model>
  ...
  <plugin filename="libignition-gazebo-linearbatteryplugin-system.so"
          name="ignition::gazebo::systems::LinearBatteryPlugin">
    <battery_name>linear_battery</battery_name>
    <voltage>12.592</voltage>
    <open_circuit_voltage_constant_coef>12.592</open_circuit_voltage_constant_coef>
    <capacity>1.2009</capacity>
    <power_load>6.6</power_load>
    <fix_issue_225>false</fix_issue_225>
  </plugin>
  ...
</model>
```

Next, you can find a description of the SDF parameters used:

`<battery_name>`: The name of the battery.
`<voltage>`: Initial voltage of the battery (V).
`<open_circuit_voltage_constant_coef>`: Voltage at full charge (V).
`<capacity>`: Total charge that the battery can hold (Ah).
`<power_load>`: Power load on battery (W).
`<fix_issue_225>`: As reported [here](https://github.com/ignitionrobotics/ign-gazebo/issues/225),
there are some issues affecting batteries in Ignition Blueprint and Citadel.
This parameter fixes the issues. Feel free to omit the parameter if you have
legacy code and want to preserve the old behavior.

When setting the `<capacity>`, `<voltage>` of the battery and its `<power_load>`,
keep in mind the following formula:

`battery_runtime` (hours) = `<capacity>` * `<voltage>` / `<power_load>`

### Known limitations

If `<fix_issue_225>` is not set, the battery drains at a faster (100x) rate.
In this case, the battery runtime should be calculated as follows:

`battery_runtime` (hours) = `<capacity>` * `<voltage>` / (`<power_load>` * 100)


## Try a more realistic battery

If you need to model a more realistic battery, you can use the following
advanced SDF parameters:

`<open_circuit_voltage_linear_coef>`: Amount of voltage decrease when no charge (V).
`<initial_charge>`: Initial charge of the battery (Ah).
`<resistance>`: Internal resistance (Ohm)
`<smooth_current_tau>`: Coefficient for smoothing current.

Please, refer to the battery specification to set the advanced values.


## Charging

A battery can be charged if the SDF parameter `<enable_recharge>` is set to true.
Here are the relevant SDF parameters related with charging:

`<enable_recharge>`: As mentioned, it should be `true` to enable recharging.
`<charging_time>`: Hours taken to fully charge the battery. Keep in mind that
this value assumes no battery load while charging. If the battery is under load,
it will take a longer time to recharge.
`<recharge_by_topic>`: If true, the start/stop signals for recharging the
battery will also be available via topics. The regular Ignition services will
still be available.

By default, two Ignition Transport services are available for managing charging:

* `/model/<model_name>/battery/<battery_name>/recharge/start`: Enable recharging.
* `/model/<model_name>/battery/<battery_name>/recharge/stop`: Disable recharging.

Both services accept an `ignition::msgs::Boolean` parameter.

## Try out an example

A battery has been added to a demo world, which can be run using:

```
cd ign-gazebo/examples/worlds
ign gazebo -v 4 -r linear_battery_demo.sdf
```

The blue vehicle on the left has a battery, while the one on the right does not. When the battery drains, the corresponding vehicle stops moving. Please, see
`ign-gazebo/examples/worlds/linear_battery_demo.sdf`, where you can
find the commands to visualize the state of the battery, as well as commands to
start and stop the recharging.


To control the vehicles with keyboard, run

```
cd ign-gazebo/examples/standalone/keyboard/build
./keyboard ../keyboard.sdf
```
See more about the usage of the keyboard plugin in `examples/standalone/keyboard/README.md`.


## Known Issues

* The rate of consumption should be affected by torque. For example, going uphill should consume more power than going downhill.
