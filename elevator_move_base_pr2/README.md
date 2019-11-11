# elevator_move_base_pr2

A ROS package for moving around in the building with elevator by PR2 robot.

## Usage

### Go to subway, buy sandwich and come back

#### Step 1

Launch some common nodes on c1 machine to prepare for using elevator.

```bash
roslaunch elevator_move_base_pr2 elevator_move_base_eng2.launch
```

#### Step 2

Launch nodes for recognition on GPU machine.
This step will be meld into step 1 in the future.

```bash
roslaunch elevator_move_base_pr2 fcn_door_button_segmentation.launch
```

#### Step 3

Launch nodes for application on c1 machine to start demonstration.

```bash
roslaunch elevator_move_base_pr2 fetch_sandwich.launch
```
