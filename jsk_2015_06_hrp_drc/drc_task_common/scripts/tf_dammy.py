#!/usr/bin/env python  
import roslib
roslib.load_manifest('drc_task_common')
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node("dammy_tf_broadcaster");
    br = tf.TransformBroadcaster()
    r = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            br.sendTransform((0, 0, 1.4),
                             (0, 0, 0, 1),
                             rospy.Time.now(),
                             "camera_link",
                             "odom_on_ground")
            r.sleep()
    except rospy.RISInterruptException: pass
