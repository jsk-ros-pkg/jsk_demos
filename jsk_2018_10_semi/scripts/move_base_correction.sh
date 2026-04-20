for a in /move_base/global_costmap/footprint /move_base/local_costmap/footprint /safe_teleop_base/local_costmap/footprint ; do                 
#/move_base/global_costmap/inflater/inflation_radius /move_base/global_costmap/inflater/robot_radius /move_base/global_costmap/robot_radius /move_base/local_costmap/inflater/inflation_radius /move_base/local_costmap/inflater/robot_radius /move_base/local_costmap/robot_radius /safe_teleop_base/local_costmap/inflater/inflation_radius /safe_teleop_base/local_costmap/inflater/robot_radius /safe_teleop_base/local_costmap/robot_radius  ; do
 echo $a
    rosparam get $a
    rosparam set $a '[[0.141315,-0.25],[0.092705,-0.285317],[-0.092705,-0.285317],[-0.242705,-0.176336],[-0.3,2.842171e-17],[-0.242705,0.176336],[-0.092705,0.285317],[0.092705,0.285317],[0.141315,0.25],[0.5,0.25],[0.5,-0.25],[0.141315,-0.25]]'
done
rosnode kill move_base
