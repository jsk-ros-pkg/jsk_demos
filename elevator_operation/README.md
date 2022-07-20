# elevator_operation

This package provides elevator operation and recognition interface to robot.

## How to run

Connect spinal to PC and

```bash
roslaunch elevator_operation spinal_driver.launch
```

Start `elevator_state_publisher.launch`

```bash
roslaunch elevator_operation elevator_state_publisher.launch
```

And run elevator_operation node

```bash
roslaunch elevator_operation elevator_operation.launch
```

Then call action.

```bash
rostopic pub -1 TODO
```
