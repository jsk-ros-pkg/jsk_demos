# cleanup robot project
## launch urdf.xacro world
```bash
roslaunch gazebo_ros empty_world.launch
# spawn xacro model
roslaunch obinata_pr2_73b2_cleanup/launch/gazebo_spawn_scene.launch scene:=door_with_wall
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