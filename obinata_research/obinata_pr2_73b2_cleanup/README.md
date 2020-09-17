# cleanup robot project
## launch urdf.xacro world
```bash
roslaunch  empty_world.launch
# spawn xacro model
roslaunch obinata_pr2_73b2_cleanup/launch/gazebo_spawn_scene.launch scene:=door_with_wall
```

## launch 73b2 and fix initial pose
```bash
ROBOT_INITIAL_POSE="-x 0.75 -y 0 -z 0 -R 0 -P 0 -Y 0" roslaunch obinata_pr2_73b2_cleanup obinata-pr2-73b2world.launch 
```

## launch door + pr2 model
```bash
ROBOT_INITIAL_POSE="-x -1.5" roslaunch pr2_gazebo pr2_empty_world.launch
roslaunch obinata_pr2_73b2_cleanup gazebo_spawn_scene.launch scene:=door_with_wall
rviz -d config/pr2_gazebo.rviz
```

## about models
The door with a knob is room73b2-door-right. NOT LEFT

## rviz
When using rviz config file, use -d option.
```bash
rviz -d config.rviz
```

## pr2_gazebo initial pos
When you want to change initial position of pr2 in pr2_gazebo, please modify the param in `pr2_no_controllers.launch`.


## Where is the eusmodel of 73b2?
`/opt/ros/melodic/share/euslisp/jskeus/eus/models/room73b2-scene.l`
## When you want to fix eusmodel converted from euslib
https://github.com/jsk-ros-pkg/euslib/pull/148


## reference
https://github.com/jsk-ros-pkg/jsk_demos/pull/1293#commitcomment-39067634

## show coords
```lisp
(send (send *door* :get :knob-coords) :draw-on :flush t) 
```