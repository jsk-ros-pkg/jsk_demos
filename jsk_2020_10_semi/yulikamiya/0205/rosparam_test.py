import rospy
import rosparam

member_list = '[{"name":"yulikamiya", "temp":"green", "x":150, "y":170}, {"x": 140, "y": 120}]'

rosparam.set_param("/yulikamiya/member_list", member_list)
res = rosparam.get_param("/yulikamiya/member_list")
print(res)
