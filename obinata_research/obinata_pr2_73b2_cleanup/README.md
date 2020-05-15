# cleanup robot project
## launch urdf.xacro world

```bash
roslaunch gazebo_ros empty_world.launch
# spawn xacro model
roslaunch obinata_pr2_73b2_cleanup/launch/gazebo_spawn_scene.launch scene:=door_with_wall
```
