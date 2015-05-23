#!/usr/bin/env python

import rospy
import math
import message_filters

import jsk_interactive_marker
from jsk_interactive_marker.srv import *
from jsk_interactive_marker.msg import *
from jsk_rviz_plugins.msg import TransformableMarkerOperate
from jsk_rviz_plugins.srv import RequestMarkerOperate
import os
import imp
imp.find_module('jsk_teleop_joy')
from jsk_teleop_joy.b_control_status import BControl2Status
from geometry_msgs.msg import *
from jsk_recognition_msgs.msg import BoundingBox
from sensor_msgs.msg import Joy, JoyFeedback, JoyFeedbackArray, PointCloud2
from std_msgs.msg import Float32, ColorRGBA, Bool
import tf
imp.find_module('std_srvs')
from std_srvs import srv
from std_msgs import msg
imp.find_module('drc_task_common')
from drc_task_common.srv import *
from drc_task_common.msg import *

def b_control_client_init():
    rospy.init_node('b_control_client')
    ns = rospy.get_param('~transformable_interactive_server_nodename', '/transformable_interactive_server')
    global robot_name
    robot_name = os.environ["ROBOT"]
    # midi device
    global prev_status, status
    prev_status=False
    status=False
    # object marker
    ## insert / erase
    global req_marker_operate_srv, set_color_pub
    global get_pose_srv, set_pose_pub, get_ik_arm_srv, get_ik_arm_pose_srv
    rospy.wait_for_service(ns+'/request_marker_operate')
    rospy.wait_for_service(ns+'/get_pose')
    rospy.wait_for_service(ns+'/set_dimensions')
    req_marker_operate_srv = rospy.ServiceProxy(ns+'/request_marker_operate', RequestMarkerOperate)
    set_color_pub = rospy.Publisher(ns+'/set_color', ColorRGBA)
    get_pose_srv = rospy.ServiceProxy(ns+'/get_pose', GetTransformableMarkerPose)
    set_pose_pub = rospy.Publisher(ns+'/set_pose', PoseStamped)
    get_ik_arm_srv = rospy.ServiceProxy('/get_ik_arm', GetIKArm)
    get_ik_arm_pose_srv = rospy.ServiceProxy('/get_ik_arm_pose', GetIKArmPose)
    global default_frame_id
    default_frame_id = rospy.get_param('~default_frame_id', 'odom')
    ## configuration
    ### pub
    global set_x_pub, set_y_pub, set_z_pub, set_r_pub, set_sr_pub
    set_x_pub = rospy.Publisher(ns+'/set_x', Float32)
    set_y_pub = rospy.Publisher(ns+'/set_y', Float32)
    set_z_pub = rospy.Publisher(ns+'/set_z', Float32)
    set_r_pub = rospy.Publisher(ns+'/set_radius', Float32)
    set_sr_pub = rospy.Publisher(ns+'/set_small_radius', Float32)
    ### param
    global x_max, y_max, z_max, r_max, sr_max, x_min, y_min, z_min, r_min, sr_min
    x_max = rospy.get_param('~x_max', 1.5)
    y_max = rospy.get_param('~y_max', 1.5)
    z_max = rospy.get_param('~z_max', 1.5)
    r_max = rospy.get_param('~r_max', 1.0)
    sr_max = rospy.get_param('~sr_max', 0.2)
    x_min = rospy.get_param('~x_min', 0.01)
    y_min = rospy.get_param('~y_min', 0.01)
    z_min = rospy.get_param('~z_min', 0.01)
    r_min = rospy.get_param('~r_min', 0.01)
    sr_min = rospy.get_param('~sr_min', 0.001)
    ## teleport
    global tf_listener, get_type_srv, set_pose_srv, set_dim_srv, midi_feedback_pub, auto_set_mode, send_icp_srv
    rospy.Service('enable_auto_set_mode', srv.Empty, enable_auto_set_mode)
    rospy.Service('disable_auto_set_mode', srv.Empty, disable_auto_set_mode)
    rospy.Service('/insert_drill_marker', srv.Empty, insert_drill_marker_cb)
    rospy.Service('/insert_plane_marker', srv.Empty, insert_plane_marker_cb)
    rospy.Service('/insert_wall_marker', srv.Empty, insert_wall_marker_cb)
    rospy.Service('/erase_all_marker', srv.Empty, erase_all_marker_cb)
    auto_set_mode = True
    ## box_sub = message_filters.Subscriber('bounding_box_marker/selected_box', BoundingBox)
    ## points_sub = message_filters.Subscriber('/selected_pointcloud', PointCloud2)
    ## ts = message_filters.TimeSynchronizer([box_sub, points_sub], 10)
    ## ts.registerCallback(selected_box_cb)
    #rospy.Subscriber('bounding_box_marker/selected_box', BoundingBox, selected_box_cb)
    rospy.Subscriber('selected_box', BoundingBox, selected_only_box_cb)
    rospy.Subscriber('/passed_selected_box', BoundingBox, selected_only_box_cb)
    rospy.Subscriber('t_marker_info', TMarkerInfo, marker_info_cb)
    
    tf_listener = tf.TransformListener()
    get_type_srv = rospy.ServiceProxy(ns+'/get_type', GetType)
    set_pose_srv = rospy.ServiceProxy(ns+'/set_pose', SetTransformableMarkerPose)
    send_icp_srv = rospy.ServiceProxy('/icp_apply', ICPService)
    set_dim_srv = rospy.ServiceProxy(ns+'/set_dimensions', SetMarkerDimensions)
    midi_feedback_pub = rospy.Publisher('midi_config_player/set_feedback', JoyFeedbackArray)
    ## handle
    global handle_variable_pos_pub, handle_variable_rot_pub, approach_variable_pub
    handle_variable_pos_pub = rospy.Publisher('/object_handle_variable_pos', Float32)
    handle_variable_rot_pub = rospy.Publisher('/object_handle_variable_rot', Float32)
    approach_variable_pub = rospy.Publisher('/object_approach_variable', Float32)
    # robot marker
    global set_robot_pose_pub, solve_ik_pub, send_angle_pub, reach_until_touch_pub, save_obj_pub, go_pos_pub, handle_reverse_srv, obj_menu_pub, robot_menu_pub, obj_mode_next_srv, ik_mode_next_srv
    set_robot_pose_pub = rospy.Publisher('urdf_control_marker/set_pose', PoseStamped)
    solve_ik_pub = rospy.Publisher('/solve_ik_command', msg.Empty)
    send_angle_pub = rospy.Publisher('/send_angle_command', msg.Empty)
    reach_until_touch_pub = rospy.Publisher('/reach_until_touch_command', msg.Empty)
    save_obj_pub = rospy.Publisher('/save_obj_command', msg.Empty)
    go_pos_pub = rospy.Publisher('/go_pos_command', msg.Empty)
    handle_reverse_srv = rospy.ServiceProxy('/set_handle_reverse', srv.Empty)
    obj_menu_pub = rospy.Publisher('/object_menu_command', msg.Empty)
    robot_menu_pub = rospy.Publisher('/robot_menu_command', msg.Empty)
    obj_mode_next_srv = rospy.ServiceProxy('/set_object_mode_next', srv.Empty)
    ik_mode_next_srv = rospy.ServiceProxy('/set_ik_mode_next', srv.Empty)
    # menu
    global menu_up_srv, menu_down_srv, menu_select_srv, menu_cancel_srv, menu_variable_pub, menu_bool_pub
    menu_up_srv = rospy.ServiceProxy('/rviz_menu_up', srv.Empty)
    menu_down_srv = rospy.ServiceProxy('/rviz_menu_up', srv.Empty)
    menu_select_srv = rospy.ServiceProxy('/rviz_menu_select', RvizMenuSelect)
    menu_cancel_srv = rospy.ServiceProxy('/rviz_menu_cancel', srv.Empty)
    menu_variable_pub = rospy.Publisher('/rviz_menu_variable', Float32)
    # menu_bool_pub = rospy.Publisher('/rviz_menu_bool', Bool)
    
    # midi device
    rospy.Subscriber('input_joy', Joy, b_control_joy_cb)
    rospy.sleep(1)


def b_control_client_main():
    rospy.spin()

def b_control_joy_cb(msg):
    global prev_status, status
    prev_status = status
    status = BControl2Status(msg)
    if not(prev_status):
        prev_status = status
    # erase marker
    if status.buttonL1 != prev_status.buttonL1:
        ik_mode_next_srv()
        return
    # insert marker
    insert_box_flag = (status.buttonU1 != prev_status.buttonU1)
    insert_cylinder_flag = (status.buttonU2 != prev_status.buttonU2)
    insert_torus_flag = (status.buttonU3 != prev_status.buttonU3)
    insert_hand_flag = (status.buttonU7 != prev_status.buttonU7)
    if insert_box_flag or insert_cylinder_flag or insert_torus_flag:
        color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.6)
        current_pose_stamped = get_pose_srv('').pose_stamped
        if current_pose_stamped.pose.orientation.x == 0 and current_pose_stamped.pose.orientation.y == 0 and current_pose_stamped.pose.orientation.z == 0 and current_pose_stamped.pose.orientation.w == 0:
            current_pose_stamped.pose.orientation.w = 1
        # midi_feedback_pub.publish([JoyFeedback(id=3, intensity=0.5)]) # reset handle variable in the generation timing
    ## insert box
    if insert_box_flag:
        erase_all_marker()
        insert_marker(shape_type=TransformableMarkerOperate.BOX, name='box1', description='')
    ## insert cylinder
    if insert_cylinder_flag:
        erase_all_marker()
        insert_marker(shape_type=TransformableMarkerOperate.CYLINDER, name='cylinder1', description='')
    ## insert torus
    if insert_torus_flag:
        erase_all_marker()
        insert_marker(shape_type=TransformableMarkerOperate.TORUS, name='torus1', description='')
        color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.6) # torus is triangle mesh
    if insert_hand_flag:
        menu_cancel_srv()
        rospy.sleep(0.1)
        erase_all_marker()
        try:
            ik_arm = get_ik_arm_srv().ik_arm
        except rospy.ServiceException, e:
            ik_arm = ":rarm"
        if (ik_arm==":rarm"):
            if robot_name=="STARO":
                mesh_resource_name="package://hrpsys_ros_bridge_tutorials/models/STARO_meshes/RARM_LINK7_mesh.dae"
                current_pose_stamped = PoseStamped(std_msgs.msg.Header(stamp=rospy.Time.now(), frame_id="jsk_model_marker_interface/robot/RARM_LINK7"), Pose(orientation=Quaternion(0, 0, 0, 1)))
            elif robot_name=="JAXON":
                mesh_resource_name="package://hrpsys_ros_bridge_tutorials/models/JAXON_meshes/RARM_LINK7_mesh.dae"
                current_pose_stamped = PoseStamped(std_msgs.msg.Header(stamp=rospy.Time.now(), frame_id="jsk_model_marker_interface/robot/RARM_LINK7"), Pose(orientation=Quaternion(0, 0, 0, 1)))
            else:
                mesh_resource_name="package://hrpsys_ros_bridge_tutorials/models/HRP3HAND_R_meshes/RARM_LINK6_mesh.dae"
                current_pose_stamped = PoseStamped(std_msgs.msg.Header(stamp=rospy.Time.now(), frame_id="jsk_model_marker_interface/robot/RARM_LINK6"), Pose(orientation=Quaternion(0, 0, 0, 1)))
        else:
            if robot_name=="STARO":
                mesh_resource_name="package://hrpsys_ros_bridge_tutorials/models/STARO_meshes/LARM_LINK7_mesh.dae"
                current_pose_stamped = PoseStamped(std_msgs.msg.Header(stamp=rospy.Time.now(), frame_id="jsk_model_marker_interface/robot/LARM_LINK7"), Pose(orientation=Quaternion(0, 0, 0, 1)))
            elif robot_name=="JAXON":
                mesh_resource_name="package://hrpsys_ros_bridge_tutorials/models/JAXON_meshes/LARM_LINK7_mesh.dae"
                current_pose_stamped = PoseStamped(std_msgs.msg.Header(stamp=rospy.Time.now(), frame_id="jsk_model_marker_interface/robot/LARM_LINK7"), Pose(orientation=Quaternion(0, 0, 0, 1)))
            else:
                mesh_resource_name="package://hrpsys_ros_bridge_tutorials/models/HRP3HAND_L_meshes/LARM_LINK6_mesh.dae"
                current_pose_stamped = PoseStamped(std_msgs.msg.Header(stamp=rospy.Time.now(), frame_id="jsk_model_marker_interface/robot/LARM_LINK6"), Pose(orientation=Quaternion(0, 0, 0, 1)))
        
        color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.6)
        # try:
        #     #current_pose_stamped = get_ik_arm_pose_srv('').pose_stamped
        #     current_pose_stamped = get_pose_srv('').pose_stamped
        # except rospy.ServiceException, e:
        #     current_pose_stamped = PoseStamped()
        if current_pose_stamped.pose.orientation.x == 0 and current_pose_stamped.pose.orientation.y == 0 and current_pose_stamped.pose.orientation.z == 0 and current_pose_stamped.pose.orientation.w == 0:
            current_pose_stamped.pose.orientation.w = 1
        insert_marker(shape_type=TransformableMarkerOperate.MESH_RESOURCE, name='hand1', description='', mesh_resource=mesh_resource_name, mesh_use_embedded_materials=True)
    if insert_box_flag or insert_cylinder_flag or insert_torus_flag or insert_hand_flag:
        set_color_pub.publish(color)
        set_pose_pub.publish(current_pose_stamped)
    # change marker configuration
    v1 = Float32()
    v2 = Float32()
    v3 = Float32()
    shape_type = get_type_srv()
    if shape_type.type == TransformableMarkerOperate.BOX:
        v1.data = (x_max - x_min) * status.slide1 + x_min
        v2.data = (y_max - y_min) * status.slide2 + y_min
        v3.data = (z_max - z_min) * status.slide3 + z_min
        set_x_pub.publish(v1)
        set_y_pub.publish(v2)
        set_z_pub.publish(v3)
    elif shape_type.type == TransformableMarkerOperate.CYLINDER:
        v1.data = (r_max - r_min) * status.slide1 + r_min
        v2.data = (z_max - z_min) * status.slide2 + z_min
        set_r_pub.publish(v1)
        set_z_pub.publish(v2)
    elif shape_type.type == TransformableMarkerOperate.TORUS:
        v1.data = (r_max - r_min) * status.slide1 + r_min
        v2.data = (sr_max - sr_min) * status.slide2 + sr_min
        set_r_pub.publish(v1)
        set_sr_pub.publish(v2)
    # set marker handle
    handle_variable_pos = Float32()
    handle_variable_pos.data = status.slide4
    handle_variable_pos_pub.publish(handle_variable_pos)
    handle_variable_rot = Float32()
    handle_variable_rot.data = status.slide5
    handle_variable_rot_pub.publish(handle_variable_rot)
    approach_variable = Float32()
    approach_variable.data = status.slide6
    approach_variable_pub.publish(approach_variable)
    # robot marker
    ## transport to obj
    if status.buttonU4 != prev_status.buttonU4:
        robot_pose = PoseStamped()
        robot_pose.header.stamp = rospy.Time.now()
        robot_pose.header.frame_id = default_frame_id
        robot_pose.pose = get_pose_srv('').pose_stamped.pose
        if shape_type.type == TransformableMarkerOperate.BOX:
            pass
        elif shape_type.type == TransformableMarkerOperate.CYLINDER:
            pass
        elif shape_type.type == TransformableMarkerOperate.TORUS:
            pass
        robot_pose.pose.position.x -= 0.5
        robot_pose.pose.position.z = 0
        robot_pose.pose.orientation.w = 1
        robot_pose.pose.orientation.x = 0
        robot_pose.pose.orientation.y = 0
        robot_pose.pose.orientation.z = 0
        set_robot_pose_pub.publish(robot_pose)
    ## command
    if status.buttonL4 != prev_status.buttonL4:
        solve_ik_pub.publish()
    if status.buttonU5 != prev_status.buttonU5:
        go_pos_pub.publish()
    if status.buttonL5 != prev_status.buttonL5:
        send_angle_pub.publish()
    if status.buttonL7 != prev_status.buttonL7:
        reach_until_touch_pub.publish()
    if status.buttonU6 != prev_status.buttonU6:
        save_obj_pub.publish()
    if status.buttonL3 != prev_status.buttonL3:
        obj_menu_pub.publish()
    if status.buttonL6 != prev_status.buttonL6:
        robot_menu_pub.publish()
    ## obj mode
    if status.buttonL2 != prev_status.buttonL2:
        obj_mode_next_srv()
    # menu
    if status.buttonU8 != prev_status.buttonU8:
        menu_down_srv()
    if status.buttonL8 != prev_status.buttonL8:
        menu_select_srv(variable = status.slide8)
    # if status.buttonU7 != prev_status.buttonU7:
    #     menu_bool_pub.publish(Bool(data=status.buttonU7))
    menu_v = Float32()
    menu_v.data = status.slide8
    menu_variable_pub.publish(menu_v)

def selected_box_cb(msg, points_msg):
    rospy.loginfo("selected_box_cb driven")
    if not auto_set_mode:
        return
    try:
        resp1 = send_icp_srv(points=points_msg, box=msg)
        erase_all_marker()
        x=resp1.dim.x
        y=resp1.dim.y
        z=resp1.dim.z
        r=resp1.dim.radius
        sr=resp1.dim.small_radius
        midi_feedback=[]
        if(resp1.dim.type==0):
            insert_marker(shape_type=TransformableMarkerOperate.BOX, name='box1', description='')
            midi_feedback.append(JoyFeedback(id=0, intensity=(x-x_min)/(x_max-x_min)))
            midi_feedback.append(JoyFeedback(id=1, intensity=(y-y_min)/(y_max-y_min)))
            midi_feedback.append(JoyFeedback(id=2, intensity=(z-z_min)/(z_max-z_min)))
        elif(resp1.dim.type==1):
            insert_marker(shape_type=TransformableMarkerOperate.CYLINDER, name='cylinder1', description='')
            midi_feedback.append(JoyFeedback(id=0, intensity=(r-r_min)/(r_max-r_min)))
            midi_feedback.append(JoyFeedback(id=1, intensity=(z-z_min)/(z_max-z_min)))
        else:
            insert_marker(shape_type=TransformableMarkerOperate.TORUS, name='torus1', description='')
            set_dim_srv(dimensions=MarkerDimensions(radius=r, small_radius=sr))
            midi_feedback.append(JoyFeedback(id=0, intensity=(r-r_min)/(r_max-r_min)))
            midi_feedback.append(JoyFeedback(id=1, intensity=(sr-sr_min)/(sr_max-sr_min)))
        midi_feedback_pub.publish(midi_feedback)
        color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.6)
        set_color_pub.publish(color)
        set_pose_srv(pose_stamped=resp1.pose_stamped)
        set_dim_srv(dimensions=resp1.dim)
        rospy.loginfo("icp succeeded")
        return
    except rospy.ServiceException, e:
        print "ICP Service call failed: %s"%e
    # pose
    selected_only_box_cb(msg)
def marker_info_cb(msg):
    erase_all_marker()
    x=msg.marker_dim.x
    y=msg.marker_dim.y
    z=msg.marker_dim.z
    r=msg.marker_dim.radius
    sr=msg.marker_dim.small_radius
    midi_feedback=[]
    if(msg.marker_dim.type==0):
        insert_marker(shape_type=TransformableMarkerOperate.BOX, name='box1', description='')
        midi_feedback.append(JoyFeedback(id=0, intensity=(x-x_min)/(x_max-x_min)))
        midi_feedback.append(JoyFeedback(id=1, intensity=(y-y_min)/(y_max-y_min)))
        midi_feedback.append(JoyFeedback(id=2, intensity=(z-z_min)/(z_max-z_min)))
    elif(msg.marker_dim.type==1):
        insert_marker(shape_type=TransformableMarkerOperate.CYLINDER, name='cylinder1', description='')
        midi_feedback.append(JoyFeedback(id=0, intensity=(r-r_min)/(r_max-r_min)))
        midi_feedback.append(JoyFeedback(id=1, intensity=(z-z_min)/(z_max-z_min)))
    else:
        insert_marker(shape_type=TransformableMarkerOperate.TORUS, name='torus1', description='')
        set_dim_srv(dimensions=MarkerDimensions(radius=r, small_radius=sr))
        midi_feedback.append(JoyFeedback(id=0, intensity=(r-r_min)/(r_max-r_min)))
        midi_feedback.append(JoyFeedback(id=1, intensity=(sr-sr_min)/(sr_max-sr_min)))
    midi_feedback_pub.publish(midi_feedback)
    color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.6)
    set_color_pub.publish(color)
    set_pose_srv(pose_stamped=msg.marker_pose_stamped)
    set_dim_srv(dimensions=msg.marker_dim)

def selected_only_box_cb(msg):
    pose_stamped_msg = PoseStamped()
    pose_stamped_msg.header = msg.header
    pose_stamped_msg.pose = msg.pose
    set_pose_pub.publish(pose_stamped_msg)
    # set_pose_srv(pose_stamped=pose_stamped_msg)
    # dimensions
    x = msg.dimensions.x
    y = msg.dimensions.y
    z = msg.dimensions.z
    shape_type = get_type_srv()
    midi_feedback=[]
    if shape_type.type == TransformableMarkerOperate.BOX:
        set_dim_srv(dimensions=MarkerDimensions(x=x, y=y, z=z))
        midi_feedback.append(JoyFeedback(id=0, intensity=(x-x_min)/(x_max-x_min)))
        midi_feedback.append(JoyFeedback(id=1, intensity=(y-y_min)/(y_max-y_min)))
        midi_feedback.append(JoyFeedback(id=2, intensity=(z-z_min)/(z_max-z_min)))
    elif shape_type.type == TransformableMarkerOperate.CYLINDER:
        r = min(x,y)
        set_dim_srv(dimensions=MarkerDimensions(radius=r, z=z))
        midi_feedback.append(JoyFeedback(id=0, intensity=(r-r_min)/(r_max-r_min)))
        midi_feedback.append(JoyFeedback(id=1, intensity=(z-z_min)/(z_max-z_min)))
    elif shape_type.type == TransformableMarkerOperate.TORUS:
        r = min(x,y)*0.5
        sr = min(x,y)*0.5*0.1
        set_dim_srv(dimensions=MarkerDimensions(radius=r, small_radius=sr))
        midi_feedback.append(JoyFeedback(id=0, intensity=(r-r_min)/(r_max-r_min)))
        midi_feedback.append(JoyFeedback(id=1, intensity=(sr-sr_min)/(sr_max-sr_min)))
    color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.6)
    set_color_pub.publish(color)
    midi_feedback_pub.publish(midi_feedback)

def enable_auto_set_mode(req):
    global auto_set_mode
    auto_set_mode = True
    return srv.EmptyResponse()

def disable_auto_set_mode(req):
    global auto_set_mode
    auto_set_mode = False
    return srv.EmptyResponse()

def insert_drill_marker_cb(req):
    insert_marker(shape_type=TransformableMarkerOperate.MESH_RESOURCE, name='drill', description='', mesh_resource="package://drc_task_common/models/takenoko_drill.dae", mesh_use_embedded_materials=True)
    return srv.EmptyResponse()

def insert_plane_marker_cb(req):
    insert_marker(shape_type=TransformableMarkerOperate.CYLINDER, name='cylinder1', description='', mesh_resource="", mesh_use_embedded_materials=True)
    return srv.EmptyResponse()

def insert_wall_marker_cb(req):
    insert_marker(shape_type=TransformableMarkerOperate.CYLINDER, name='drill_wall', description='', mesh_resource="", mesh_use_embedded_materials=True)
    return srv.EmptyResponse()
def erase_all_marker_cb(req):
    erase_all_marker()
    return srv.EmptyResponse()
def insert_marker(shape_type=TransformableMarkerOperate.BOX, name='default_name', description='default_description', mesh_resource='', mesh_use_embedded_materials=False):
    try:
        req_marker_operate_srv(TransformableMarkerOperate(type=shape_type, action=TransformableMarkerOperate.INSERT, frame_id=default_frame_id, name=name, description=description, mesh_resource=mesh_resource, mesh_use_embedded_materials=mesh_use_embedded_materials))
    except rospy.ServiceException, e:
        print 'insert_marker service call failed: %s'%e


def erase_all_marker():
    try:
        req_marker_operate_srv(TransformableMarkerOperate(type=TransformableMarkerOperate.BOX, action=TransformableMarkerOperate.ERASEALL))
    except rospy.ServiceException, e:
        print 'insert_marker service call failed: %s'%e

if __name__ == '__main__':
    b_control_client_init()
    b_control_client_main()

