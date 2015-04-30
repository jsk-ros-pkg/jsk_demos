#! /usr/bin/env python

import roslib; roslib.load_manifest('gazebo_drive_simulator')
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion
import rospy
import numpy

class VehicleTrajectoryPlanner:
    def __init__ (self, node_name = "vehicle_controller", rate = 10):
        rospy.init_node(node_name)
        self.rate = rospy.Rate(rate)
        rospy.Subscriber("/polaris_interactive_marker/pose", PoseStamped, self.interactive_marker_pose_callback)
        self.trajectory_publisher = rospy.Publisher("~vehicle_trajectory", Path)

    def interactive_marker_pose_callback(self, msg):
        trajectory = self.calculate_path_bazier(msg)
        self.trajectory_publisher.publish(trajectory)

    def calculate_path_bazier(self, pose_stamped):
        trajectory = Path()

        # start_point
        x_s = 0.0
        y_s = 0.0
        # end point
        x_g = pose_stamped.pose.position.x
        y_g = pose_stamped.pose.position.y
        angles = euler_from_quaternion([pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w])
        theta_g = angles[2] # get orientation around z axis
        # control point (intersection point of tangentical lines of start and goal point)
        x_c = -y_g / numpy.tan(theta_g) + x_g
        y_c = 0.0

        # calculate trajectory using quadratic bazier function
        # cf) http://geom.web.fc2.com/geometry/bezier/quadratic.html
        step = 10
        for t in range(0, step):
            tmp_pose_stamped = PoseStamped()
            t = float(t) / float(step)
            x_t = t**2 * x_g + 2.0 * t * (1.0 - t) * x_c + (1.0 - t)**2 * x_s
            y_t = t**2 * y_g + 2.0 * t * (1.0 - t) * y_c + (1.0 - t)**2 * y_s
            dx_t = 2.0 * (t * (x_g - x_c) + (1.0 - t) * (x_c - x_s)) # dx/dt
            dy_t = 2.0 * (t * (y_g - y_c) + (1.0 - t) * (y_c - y_s)) # dy/dt
            theta_t = numpy.arctan2(dy_t, dx_t) # arctan(dy/dx) = arctan((dy/dt)/(dx/dt))
            tmp_pose_stamped.pose.position.x = x_t
            tmp_pose_stamped.pose.position.y = y_t
            tmp_pose_stamped.pose.position.z = 0.0
            tmp_pose_stamped.pose.orientation.x = 0.0
            tmp_pose_stamped.pose.orientation.y = 0.0
            tmp_pose_stamped.pose.orientation.z = theta_t
            tmp_pose_stamped.header = pose_stamped.header
            trajectory.poses.append(tmp_pose_stamped)
        trajectory.header = pose_stamped.header

        return trajectory
        
        
    def execute(self):
        self.rate.sleep()

if __name__ == "__main__":
    node = VehicleTrajectoryPlanner()
    while not rospy.is_shutdown():
        node.execute()
        
