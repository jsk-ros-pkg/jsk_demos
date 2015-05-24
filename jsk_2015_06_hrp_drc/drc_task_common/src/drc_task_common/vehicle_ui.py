# rqt plugin for vehicle task

from rqt_gui_py.plugin import Plugin
import python_qt_binding.QtGui as QtGui
from python_qt_binding.QtGui import (QAction, QIcon, QMenu, QWidget,
                                     QPainter, QColor, QFont, QBrush,
                                     QPen, QMessageBox, QSizePolicy,
                                     QListWidget, QLineEdit, QImage, QPixmap)
from python_qt_binding.QtCore import (Qt, QTimer, qWarning, Slot, QEvent, QSize,
                                      pyqtSignal,
                                      pyqtSlot)
from drc_task_common.srv import SetValue, StringRequest
from threading import Lock
import rospy
import python_qt_binding.QtCore as QtCore
from std_msgs.msg import Bool, Time
import sensor_msgs.msg
import math
import yaml
import os, sys
from std_srvs.srv import Empty
from jsk_rqt_plugins.image_view2_wrapper import ScaledLabel
import std_msgs.msg
import cv2, cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import geometry_msgs.msg


class VehicleUI(Plugin):
    """
    rqt plugin for vehicle ui
    """
    def __init__(self, context):
        super(VehicleUI, self).__init__(context)
        self.setObjectName("VehicleUI")
        self._widget = VehicleUIWidget()
        context.add_widget(self._widget)

class VehicleUIWidget(QWidget):
    """
    vehicle ui widget
    """
    def __init__(self):
        super(VehicleUIWidget, self).__init__()
        self.lock = Lock()
        root_hbox = QtGui.QGridLayout(self)
        left_vbox = QtGui.QVBoxLayout(self)
        center_vbox = QtGui.QVBoxLayout(self)
        right_vbox = QtGui.QVBoxLayout(self)
        
        # add widgets to layouts
        self.setUpLeftBox(left_vbox)
        self.setUpCenterBox(center_vbox)
        self.setUpRightBox(right_vbox)
        
        left_container = QtGui.QWidget()
        center_container = QtGui.QWidget()
        right_container = QtGui.QWidget()
        left_splitter = QtGui.QSplitter(self)
        right_splitter = QtGui.QSplitter(self)

        left_container.setLayout(left_vbox)
        center_container.setLayout(center_vbox)
        right_container.setLayout(right_vbox)
        right_splitter.addWidget(center_container)
        right_splitter.addWidget(right_container)
        right_splitter.setSizes((1000, 150))
        left_splitter.addWidget(left_container)
        left_splitter.addWidget(right_splitter)
        left_splitter.setSizes((150, 1000))
        root_hbox.addWidget(left_splitter, 0, 0)

        self.timerId = self.startTimer(1000)
        
        self.setLayout(root_hbox)
        self.setupSubscribers()
        self.show()
    def service(self, name):
        return lambda x: self.serviceEmptyImpl(name)
    def serviceEmptyImpl(self, name):
        srv = rospy.ServiceProxy(name, Empty)
        try:
            srv()
        except rospy.ServiceException, e:
            self.showError("Failed to call %s" % name)

    def handleModeClickedCallback(self, item):
        try:
            update_mode = rospy.ServiceProxy('drive/controller/set_handle_mode', StringRequest)
            update_mode(item.text())
        except rospy.ServiceException, e:
            self.showError("Failed to call drive/controller/set_handle_mode")
    def accelModeClickedCallback(self, item):
        try:
            update_mode = rospy.ServiceProxy('drive/controller/set_accel_mode', StringRequest)
            update_mode(item.text())
        except rospy.ServiceException, e:
            self.showError("Failed to call drive/controller/set_accel_mode")
    def neckModeClickedCallback(self, item):
        try:
            update_mode = rospy.ServiceProxy('drive/controller/set_neck_mode', StringRequest)
            update_mode(item.text())
        except rospy.ServiceException, e:
            self.showError("Failed to call drive/controller/set_neck_mode")
    
    def setUpLeftBox(self, left_vbox):

        execute_vbox = QtGui.QVBoxLayout(self)
        execute_group = QtGui.QGroupBox("", self)
        self.execute_button = QtGui.QPushButton("EXECUTE")
        self.execute_flag_publisher = rospy.Publisher("drive/execute_flag", Bool)
        self.execute_button.setCheckable(True)
        self.execute_button.clicked[bool].connect(self.setExecuteFlagCallback)
        self.execute_button.setStyleSheet("background-color: lightgreen")
        execute_vbox.addWidget(self.execute_button)
        execute_group.setLayout(execute_vbox)
        left_vbox.addWidget(execute_group)
        
        handle_mode_vbox = QtGui.QVBoxLayout(self)
        handle_mode_group = QtGui.QGroupBox("Handle Mode", self)
        self.handle_mode_list = QListWidget()
        self.handle_mode_list.addItem("Operation")
        self.handle_mode_list.addItem("Recognition")
        self.handle_mode_list.addItem("Stop")
        self.handle_mode_list.itemClicked.connect(self.handleModeClickedCallback)
        handle_mode_vbox.addWidget(self.handle_mode_list)
        handle_mode_group.setLayout(handle_mode_vbox)
        left_vbox.addWidget(handle_mode_group)

        accel_mode_vbox = QtGui.QVBoxLayout(self)
        accel_mode_group = QtGui.QGroupBox("Accel Mode", self)
        self.accel_mode_list = QListWidget()
        self.accel_mode_list.addItem("Operation")
        self.accel_mode_list.addItem("Recognition")
        self.accel_mode_list.addItem("Stop")
        self.accel_mode_list.itemClicked.connect(self.accelModeClickedCallback)
        accel_mode_vbox.addWidget(self.accel_mode_list)
        accel_mode_group.setLayout(accel_mode_vbox)
        left_vbox.addWidget(accel_mode_group)

        neck_mode_vbox = QtGui.QVBoxLayout(self)
        neck_mode_group = QtGui.QGroupBox("Neck Mode", self)
        self.neck_mode_list = QListWidget()
        self.neck_mode_list.addItem("Operation")
        self.neck_mode_list.addItem("Recognition")
        self.neck_mode_list.addItem("Stop")
        self.neck_mode_list.itemClicked.connect(self.neckModeClickedCallback)
        neck_mode_vbox.addWidget(self.neck_mode_list)
        neck_mode_group.setLayout(neck_mode_vbox)
        left_vbox.addWidget(neck_mode_group)
        
        action_vbox = QtGui.QVBoxLayout(self)
        action_group = QtGui.QGroupBox("action", self)
        self.initialize_button = QtGui.QPushButton("Initial Pose")
        self.grasp_button = QtGui.QPushButton("Grasp Handle")
        self.release_button = QtGui.QPushButton("Release Handle")
        self.correct_button = QtGui.QPushButton("Correct Handle Pose")
        self.resume_button = QtGui.QPushButton("Resume Handle Pose")
        self.initialize_button.clicked.connect(self.service("drive/controller/initialize"))
        self.grasp_button.clicked.connect(self.service("drive/controller/grasp"))
        self.release_button.clicked.connect(self.service("drive/controller/release"))
        self.correct_button.clicked.connect(self.service("drive/controller/correct"))
        self.resume_button.clicked.connect(self.service("drive/controller/resume"))
        action_vbox.addWidget(self.initialize_button)
        action_vbox.addWidget(self.grasp_button)
        action_vbox.addWidget(self.release_button)
        action_vbox.addWidget(self.correct_button)
        action_vbox.addWidget(self.resume_button)
        action_group.setLayout(action_vbox)
        left_vbox.addWidget(action_group)

        reach_vbox = QtGui.QVBoxLayout(self)
        reach_group = QtGui.QGroupBox("", self)
        self.reach_button = QtGui.QPushButton("Approach")
        menu = QtGui.QMenu()
        approachHandleArmAction = QAction("Approach Handle Arm", self)
        approachHandleArmAction.triggered.connect(self.service("drive/controller/approach_handle"))
        menu.addAction(approachHandleArmAction)
        approachAccelLegAction = QAction("Approach Accel Leg", self)
        approachAccelLegAction.triggered.connect(self.service("drive/controller/approach_accel"))
        menu.addAction(approachAccelLegAction)
        approachSupportArmAction = QAction("Approach Support Arm", self)
        approachSupportArmAction.triggered.connect(self.service("drive/controller/reach_arm"))
        menu.addAction(approachSupportArmAction)
        approachSupportLegAction = QAction("Approach Support Leg", self)
        approachSupportLegAction.triggered.connect(self.service("drive/controller/reach_leg"))
        menu.addAction(approachSupportLegAction)
        self.reach_button.setMenu(menu)
        reach_vbox.addWidget(self.reach_button)
        reach_group.setLayout(reach_vbox)
        left_vbox.addWidget(self.reach_button)

        # overwrite_vbox = QtGui.QVBoxLayout(self)
        overwrite_hbox = QtGui.QHBoxLayout(self)
        overwrite_group = QtGui.QGroupBox("", self)
        self.overwrite_button = QtGui.QPushButton("Overwrite")
        self.overwrite_button.clicked.connect(self.overwriteButtonCallback)
        self.overwrite_edit = QtGui.QLineEdit()
        self.overwrite_edit.setText("0.0")
        self.overwrite_edit.setValidator(QtGui.QDoubleValidator(-540, 540, 10))
        overwrite_hbox.addWidget(self.overwrite_button)
        overwrite_hbox.addWidget(self.overwrite_edit)
        overwrite_group.setLayout(overwrite_hbox)
        left_vbox.addWidget(overwrite_group)
        
        egress_vbox = QtGui.QVBoxLayout(self)
        egress_group = QtGui.QGroupBox("", self)
        self.egress_button = QtGui.QPushButton("Go to Egress")
        self.egress_button.clicked.connect(self.service("drive/controller/egress"))
        egress_vbox.addWidget(self.egress_button)
        egress_group.setLayout(egress_vbox)
        left_vbox.addWidget(egress_group)

        step_vbox = QtGui.QVBoxLayout(self)
        step_group = QtGui.QGroupBox("step", self)
        step_max_hbox = QtGui.QHBoxLayout(self)
        step_max_label = QtGui.QLabel("Max", self)
        step_max_vbox = QtGui.QVBoxLayout(self)
        self.step_max_value = -1
        self.step_max_up_button = QtGui.QPushButton()
        self.step_max_up_button.setIcon(QIcon.fromTheme("go-up"))
        self.step_max_up_button.clicked.connect(self.maxUpButtonCallback)
        self.step_max_down_button = QtGui.QPushButton()
        self.step_max_down_button.setIcon(QIcon.fromTheme("go-down"))
        self.step_max_down_button.clicked.connect(self.maxDownButtonCallback)
        self.step_max_edit = QtGui.QLineEdit()
        self.step_max_edit.setText(str(self.step_max_value))
        self.step_max_edit.returnPressed.connect(self.maxEditCallback)
        self.step_max_edit.setValidator(QtGui.QDoubleValidator(-200, 200, 10))
        step_max_hbox.addWidget(step_max_label)
        step_max_vbox.addWidget(self.step_max_up_button)
        step_max_vbox.addWidget(self.step_max_edit)
        step_max_vbox.addWidget(self.step_max_down_button)
        step_max_hbox.addLayout(step_max_vbox)
        step_vbox.addLayout(step_max_hbox)

        step_min_hbox = QtGui.QHBoxLayout(self)
        step_min_label = QtGui.QLabel("Min", self)
        step_min_vbox = QtGui.QVBoxLayout(self)
        self.step_min_value = -1
        self.step_min_up_button = QtGui.QPushButton()
        self.step_min_up_button.setIcon(QIcon.fromTheme("go-up"))
        self.step_min_up_button.clicked.connect(self.minUpButtonCallback)
        self.step_min_down_button = QtGui.QPushButton()
        self.step_min_down_button.setIcon(QIcon.fromTheme("go-down"))
        self.step_min_down_button.clicked.connect(self.minDownButtonCallback)
        self.step_min_edit = QtGui.QLineEdit()
        self.step_min_edit.setText(str(self.step_min_value))
        self.step_min_edit.returnPressed.connect(self.minEditCallback)
        self.step_min_edit.setValidator(QtGui.QDoubleValidator(-200, 200, 10))
        step_min_hbox.addWidget(step_min_label)
        step_min_vbox.addWidget(self.step_min_up_button)
        step_min_vbox.addWidget(self.step_min_edit)
        step_min_vbox.addWidget(self.step_min_down_button)
        step_min_hbox.addLayout(step_min_vbox)
        step_vbox.addLayout(step_min_hbox)
        
        step_group.setLayout(step_vbox)
        left_vbox.addWidget(step_group)

    def setUpCenterBox(self, center_box):
        self.multisense_widget = ROSImageWidget("/multisense/left/image_rect_color")
        # self.multisense_widget = ROSImageWidget("/ocs/communication/image_rect_color")
        center_box.addWidget(self.multisense_widget)
        left_splitter = QtGui.QSplitter()
        right_splitter = QtGui.QSplitter()
        left_container = QtGui.QWidget()
        right_container = QtGui.QWidget()
        lower_left_vbox = QtGui.QVBoxLayout()

        lhsensor_group = QtGui.QGroupBox("LARM Force")
        lhsensor_vbox = QtGui.QVBoxLayout()
        font = QtGui.QFont()
        font.setPointSize(16)
        self.lhsensor_label = QtGui.QLabel("-1")
        self.lhsensor_label.setFont(font)
        self.lhsensor_msg = None
        lhsensor_vbox.addWidget(self.lhsensor_label)
        lhsensor_group.setLayout(lhsensor_vbox)
        lower_left_vbox.addWidget(lhsensor_group)
        rhsensor_group = QtGui.QGroupBox("RARM Force")
        rhsensor_vbox = QtGui.QVBoxLayout()
        self.rhsensor_label = QtGui.QLabel("-1")
        self.rhsensor_label.setFont(font)
        self.rhsensor_msg = None
        rhsensor_vbox.addWidget(self.rhsensor_label)
        rhsensor_group.setLayout(rhsensor_vbox)
        lower_left_vbox.addWidget(rhsensor_group)
        lfsensor_group = QtGui.QGroupBox("LLEG Force")
        lfsensor_vbox = QtGui.QVBoxLayout()
        self.lfsensor_label = QtGui.QLabel("-1")
        self.lfsensor_label.setFont(font)
        self.lfsensor_msg = None
        lfsensor_vbox.addWidget(self.lfsensor_label)
        lfsensor_group.setLayout(lfsensor_vbox)
        lower_left_vbox.addWidget(lfsensor_group)
        rfsensor_group = QtGui.QGroupBox("RLEG Force")
        rfsensor_vbox = QtGui.QVBoxLayout()
        self.rfsensor_label = QtGui.QLabel("-1")
        self.rfsensor_label.setFont(font)
        self.rfsensor_msg = None
        rfsensor_vbox.addWidget(self.rfsensor_label)
        rfsensor_group.setLayout(rfsensor_vbox)
        lower_left_vbox.addWidget(rfsensor_group)
        lower_left_container = QtGui.QWidget()
        lower_left_container.setLayout(lower_left_vbox)
        left_splitter.addWidget(lower_left_container)
        
        self.fisheye_widget = ROSImageWidget("/chest_camera/image_color")
        # self.fisheye_widget = ROSImageWidget("/ocs/communication/panorama_image")
        right_splitter.addWidget(self.fisheye_widget)
       
        angle_vbox = QtGui.QVBoxLayout()
        angle_container = QtGui.QWidget()
        self.goal_angle = AngleWidget("drive/controller/goal_handle_angle")
        self.estimated_angle = AngleWidget("drive/controller/estimated_handle_angle")
        angle_vbox.addWidget(self.goal_angle, 5)
        angle_vbox.addWidget(self.estimated_angle, 5)
        handle_diff_av_hbox = QtGui.QHBoxLayout()
        handle_diff_av_container = QtGui.QWidget()
        self.handle_diff_av_value = 0.0
        handle_diff_av_label = QtGui.QLabel("Steering Diff Angle Vector:", self)
        handle_diff_av_label.setFont(font)
        self.handle_diff_av_value_label = QtGui.QLabel(str(self.handle_diff_av_value))
        self.handle_diff_av_value_label.setFont(font)
        handle_diff_av_hbox.addWidget(handle_diff_av_label)
        handle_diff_av_hbox.addWidget(self.handle_diff_av_value_label)
        handle_diff_av_container.setLayout(handle_diff_av_hbox)
        angle_vbox.addWidget(handle_diff_av_container, 1)
        angle_container.setLayout(angle_vbox)

        obstacle_length_hbox = QtGui.QHBoxLayout()
        obstacle_length_container = QtGui.QWidget()
        self.obstacle_length_value = 0.0
        obstacle_length_label = QtGui.QLabel("Obstale Length:", self)
        obstacle_length_label.setFont(font)
        self.obstacle_length_value_label = QtGui.QLabel(str(self.obstacle_length_value))
        self.obstacle_length_value_label.setFont(font)
        obstacle_length_hbox.addWidget(obstacle_length_label)
        obstacle_length_hbox.addWidget(self.obstacle_length_value_label)
        obstacle_length_container.setLayout(obstacle_length_hbox)
        angle_vbox.addWidget(obstacle_length_container, 1)
        angle_container.setLayout(angle_vbox)

        right_splitter.addWidget(angle_container)
        left_splitter.addWidget(right_splitter)
        right_splitter.setSizes((500, 250))
        left_splitter.setSizes((200, 1000))
        center_box.addWidget(left_splitter)
    def setUpRightBox(self, right_vbox):
        self.step_gage_label = QtGui.QLabel("-1.0") # Initial value
        self.step_gage_label.setFixedHeight(50)
        
        right_vbox.addWidget(self.step_gage_label)
        self.step_gage = StepGageWidget("drive/controller/step",
                                        "drive/controller/max_step",
                                        "drive/controller/min_step")
        right_vbox.addWidget(self.step_gage)
        self.set_current_step_as_max_button = QtGui.QPushButton("Set Current Step as Max")
        self.set_current_step_as_max_button.clicked.connect(self.setCurrentStepAsMaxButtonCallback)
        self.set_current_step_as_min_button = QtGui.QPushButton("Set Current Step as Min")
        self.set_current_step_as_min_button.clicked.connect(self.setCurrentStepAsMinButtonCallback)
        right_vbox.addWidget(self.set_current_step_as_max_button)
        right_vbox.addWidget(self.set_current_step_as_min_button)
    # Message callback
    def setupSubscribers(self):
        self.step_gage_value_sub = rospy.Subscriber(
            "drive/controller/step", std_msgs.msg.Float32, self.stepGageValueCallback)
        self.min_step_value_sub = rospy.Subscriber(
            "drive/controller/min_step", std_msgs.msg.Float32, self.minStepGageValueCallback)
        self.max_step_value_sub = rospy.Subscriber(
            "drive/controller/max_step", std_msgs.msg.Float32, self.maxStepGageValueCallback)
        self.lleg_force_sub = rospy.Subscriber(
            "/lhsensor", geometry_msgs.msg.WrenchStamped, self.lhsensorCallback)
        self.rleg_force_sub = rospy.Subscriber(
            "/rhsensor", geometry_msgs.msg.WrenchStamped, self.rhsensorCallback)
        self.lleg_force_sub = rospy.Subscriber(
            "/lfsensor", geometry_msgs.msg.WrenchStamped, self.lfsensorCallback)
        self.rleg_force_sub = rospy.Subscriber(
            "/rfsensor", geometry_msgs.msg.WrenchStamped, self.rfsensorCallback)
        self.handle_mode_sub = rospy.Subscriber(
            "drive/controller/handle_mode", std_msgs.msg.String, self.handleModeCallback)
        self.accel_mode_sub = rospy.Subscriber(
            "drive/controller/accel_mode", std_msgs.msg.String, self.accelModeCallback)
        self.neck_mode_sub = rospy.Subscriber(
            "drive/controller/neck_mode", std_msgs.msg.String, self.neckModeCallback)
        self.handle_angle_vector_diff_sub = rospy.Subscriber(
            "drive/controller/steering_diff_angle_vector", std_msgs.msg.Float32, self.steeringDiffAngleVectorCallback)
        self.obstacle_length_sub = rospy.Subscriber(
            "drive/recognition/obstacle_length/indicator", std_msgs.msg.Float32, self.obstacleLengthCallback)
    def lhsensorCallback(self, msg):
        self.lhsensor_msg = msg
    def rhsensorCallback(self, msg):
        self.rhsensor_msg = msg
    def lfsensorCallback(self, msg):
        self.lfsensor_msg = msg
    def rfsensorCallback(self, msg):
        self.rfsensor_msg = msg
    def updateForceSensor(self, label, msg):
        max_force = None
        max_direction = None
        index_to_direction = {0:":x", 1:":y", 2:":z"}
        for idx, value in enumerate([str(msg.wrench.force.x), str(msg.wrench.force.y), str(msg.wrench.force.z)]):
            if max_force == None or abs(float(value)) > abs(max_force):
                max_force = round(float(value), 3)
                max_direction = index_to_direction[idx]
        label.setText(max_direction + " " + str(max_force))
        if abs(max_force) > 150.0: # threshould
            label.setAutoFillBackground(True);
            palette = QtGui.QPalette()
            palette.setColor(QtGui.QPalette.Background,QtCore.Qt.red)
            label.setPalette(palette)
        elif abs(max_force) > 100.0: # threshould
            label.setAutoFillBackground(True);
            palette = QtGui.QPalette()
            palette.setColor(QtGui.QPalette.Background,QtCore.Qt.yellow)
            label.setPalette(palette)
        elif abs(max_force) > 50.0: # threshould
            label.setAutoFillBackground(True);
            palette = QtGui.QPalette()
            palette.setColor(QtGui.QPalette.Background,QtCore.Qt.green)
            label.setPalette(palette)
        else:
            label.setAutoFillBackground(False);
    def setExecuteFlagCallback(self, pressed):
        pub_msg = Bool()
        if pressed:
            pub_msg.data = True
            self.execute_button.setStyleSheet("background-color: red")
        else:
            self.execute_button.setStyleSheet("background-color: lightgreen")
        self.execute_flag_publisher.publish(pub_msg)
            
    def stepGageValueCallback(self, msg):
        with self.lock:
            self.step_gage_label.setText(str(msg.data))
    def minStepGageValueCallback(self, msg):
        with self.lock:
            if self.step_min_value != msg.data:
                self.step_min_value = msg.data
                self.step_min_edit.setText(str(self.step_min_value))
    def maxStepGageValueCallback(self, msg):
        with self.lock:
            if self.step_max_value != msg.data:
                self.step_max_value = msg.data
                self.step_max_edit.setText(str(self.step_max_value))
    def handleModeCallback(self, msg):
        with self.lock:
            for i in range(self.handle_mode_list.count()):
                item = self.handle_mode_list.item(i)
                item.setSelected(False)
                if item.text() == msg.data.capitalize():
                    item.setBackground(QtGui.QColor("#18FFFF"))
                else:
                    item.setBackground(Qt.white)
    def accelModeCallback(self, msg):
        with self.lock:
            for i in range(self.accel_mode_list.count()):
                item = self.accel_mode_list.item(i)
                item.setSelected(False)
                if item.text() == msg.data.capitalize():
                    item.setBackground(QtGui.QColor("#18FFFF"))
                else:
                    item.setBackground(QtCore.Qt.white)
    def neckModeCallback(self, msg):
        with self.lock:
            for i in range(self.neck_mode_list.count()):
                item = self.neck_mode_list.item(i)
                item.setSelected(False)
                if item.text() == msg.data.capitalize():
                    item.setBackground(QtGui.QColor("#18FFFF"))
                else:
                    item.setBackground(QtCore.Qt.white)
    def steeringDiffAngleVectorCallback(self, msg):
        with self.lock:
            if self.handle_diff_av_value != msg.data:
                self.handle_diff_av_value = msg.data
                self.handle_diff_av_value_label.setText(str(int(msg.data)))
                if abs(float(self.handle_diff_av_value)) > 40.0: # threshould
                     self.handle_diff_av_value_label.setAutoFillBackground(True);
                     palette = QtGui.QPalette()
                     palette.setColor(QtGui.QPalette.Background,QtCore.Qt.red)
                     self.handle_diff_av_value_label.setPalette(palette)
                elif abs(float(self.handle_diff_av_value)) > 15.0: # threshould
                     self.handle_diff_av_value_label.setAutoFillBackground(True);
                     palette = QtGui.QPalette()
                     palette.setColor(QtGui.QPalette.Background,QtCore.Qt.yellow)
                     self.handle_diff_av_value_label.setPalette(palette)
                elif abs(float(self.handle_diff_av_value)) > 5.0: # threshould
                     self.handle_diff_av_value_label.setAutoFillBackground(True);
                     palette = QtGui.QPalette()
                     palette.setColor(QtGui.QPalette.Background,QtCore.Qt.green)
                     self.handle_diff_av_value_label.setPalette(palette)
                else:
                    self.handle_diff_av_value_label.setAutoFillBackground(False);
    def obstacleLengthCallback(self, msg):
        with self.lock:
            self.obstacle_length_value = msg.data
            self.obstacle_length_value_label.setText(str(int(msg.data)))
    # Event callback
    def minUpButtonCallback(self, event):
        current_value = float(self.step_min_edit.text())
        current_value = current_value + 1.0
        next_value = self.callSetValueService('drive/controller/set_min_step', current_value)
        if next_value != None:
            with self.lock:
                self.step_min_edit.setText(str(next_value))
    def minDownButtonCallback(self, event):
        current_value = float(self.step_min_edit.text())
        current_value = current_value - 1.0
        next_value = self.callSetValueService('drive/controller/set_min_step', current_value)
        if next_value != None:
            with self.lock:
                self.step_min_edit.setText(str(next_value))
    def minEditCallback(self):
        current_value = float(self.step_min_edit.text())
        next_value = self.callSetValueService('drive/controller/set_min_step', current_value)
        if next_value != None:
            with self.lock:
                self.step_min_edit.setText(str(next_value))
        else:
            with self.lock:
                self.step_min_edit.setText(str(self.step_min_value))

    def maxUpButtonCallback(self, event):
        current_value = float(self.step_max_edit.text())
        current_value = current_value + 1.0
        next_value = self.callSetValueService('drive/controller/set_max_step', current_value)
        if next_value != None:
            with self.lock:
                self.step_max_edit.setText(str(next_value))
    def maxDownButtonCallback(self, event):
        current_value = float(self.step_max_edit.text())
        current_value = current_value - 1.0
        next_value = self.callSetValueService('drive/controller/set_max_step', current_value)
        if next_value != None:
            with self.lock:
                self.step_max_edit.setText(str(next_value))
    def maxEditCallback(self):
        current_value = float(self.step_max_edit.text())
        next_value = self.callSetValueService('drive/controller/set_max_step', current_value)
        if next_value != None:
            with self.lock:
                self.step_max_edit.setText(str(next_value))
        else:
            with self.lock:
                self.step_max_edit.setText(str(self.step_max_value))

    def overwriteButtonCallback(self, event):
        current_handle = float(self.overwrite_edit.text())
        next_value = self.callSetValueService('drive/controller/overwrite_handle_angle', current_handle)
        if next_value != None:
            with self.lock:
                self.overwrite_edit.setText(str(next_value))

    def setCurrentStepAsMaxButtonCallback(self, event):
        current_step = float(self.step_gage_label.text())
        next_value = self.callSetValueService('drive/controller/set_max_step', current_step)
        if next_value != None:
            with self.lock:
                self.step_max_edit.setText(str(next_value))
    def setCurrentStepAsMinButtonCallback(self, event):
        current_step = float(self.step_gage_label.text())
        next_value = self.callSetValueService('drive/controller/set_min_step', current_step)
        if next_value != None:
            with self.lock:
                self.step_min_edit.setText(str(next_value))
        
    def callSetValueService(self, service_name, value):
        try:
            update_value = rospy.ServiceProxy(service_name, SetValue)
            next_value = update_value(value)
            return next_value.set_value
        except rospy.ServiceException, e:
            self.showError("Failed to call " + service_name)
            return None
    def timerEvent(self, event):
        label_list = [self.lhsensor_label, self.rhsensor_label, self.lfsensor_label, self.rfsensor_label]
        msg_list = [self.lhsensor_msg, self.rhsensor_msg, self.lfsensor_msg, self.rfsensor_msg]
        with self.lock:
            for (label, msg) in zip(label_list, msg_list):
                if msg != None:
                    self.updateForceSensor(label, msg)
        
            
    def showError(self, message):
        QMessageBox.about(self, "ERROR", message)

class ROSImageWidget(QWidget):
    """
    Qt widget to show sensor_msgs.msg.Image
    """
    repaint_trigger = pyqtSignal()
    def __init__(self, topic_name):
        super(ROSImageWidget, self).__init__()
        self.cv_image = None
        self.pixmap = QtGui.QImage((0, 0))

        self.repaint_trigger.connect(self.redraw)
        self.lock = Lock()
        self.need_to_rewrite = False
        self.bridge = CvBridge()
        self.image_sub = None
        self.label = ScaledLabel()
        self.sub = rospy.Subscriber(topic_name, sensor_msgs.msg.Image,
                                    self.imageCallback)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setSizePolicy(QSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored))
        vbox = QtGui.QVBoxLayout(self)
        vbox.addWidget(self.label)
        self.setLayout(vbox)
        self.show()
    def imageCallback(self, msg):
        with self.lock:
            if msg.width == 0 or msg.height == 0:
                rospy.logdebug("Looks input images is invalid")
                return
            cv_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            if msg.encoding == "bgr8":
                self.cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            elif msg.encoding == "rgb8":
                self.cv_image = cv_image
            self.numpy_image = np.asarray(self.cv_image)
            self.need_to_rewrite = True
            self.repaint_trigger.emit()
    @pyqtSlot()
    def redraw(self):
        with self.lock:
            if not self.need_to_rewrite:
                return
            if self.cv_image != None:
                size = self.cv_image.shape
                img = QImage(self.cv_image.data,
                             size[1], size[0], size[2] * size[1],
                             QImage.Format_RGB888)
                # convert to QPixmap
                self.pixmap = QPixmap(size[1], size[0])
                self.pixmap.convertFromImage(img)
                self.label.setPixmap(self.pixmap.scaled(
                    self.label.width(), self.label.height(),
                    QtCore.Qt.KeepAspectRatio))
                #self.label.setPixmap(self.pixmap)
                
class AngleWidget(QWidget):
    """
    QWidget to visualize angle of stearin handle
    """
    def __init__(self, topic_name):
        super(AngleWidget, self).__init__()
        self.lock = Lock()
        self.value = 0.0
        self.value_sub = rospy.Subscriber(
            topic_name,
            std_msgs.msg.Float32, self.valueCallback)
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.redraw)
        self._update_plot_timer.start(1000 / 15) # 15 fpsw
        return
    def valueCallback(self, msg):
        with self.lock:
            self.value = msg.data
    def redraw(self):
        self.update()
    def paintEvent(self, event):
        with self.lock:
            line_width = 10
            w = self.width() - line_width
            h = self.height() - line_width
            r = min(w, h) / 2.0
            margin_x = (w - 2 * r) / 2.0
            margin_y = (h - 2 * r) / 2.0
            center = (margin_x + r, margin_y + r)
            qp = QPainter()
            qp.begin(self)
            qp.setFont(QFont('Helvetica', 20))
            qp.setPen(QPen(QBrush(QColor(0, 0, 0)), line_width))
            
            rect = QtCore.QRect(int(margin_x), int(margin_y), int(r) * 2, int(r) * 2)
            qp.drawArc(rect, 90 * 16, 360 * 16)
            qp.setPen(QPen(QBrush(QColor(255, 0, 0)), line_width))
            qp.drawLine(int(center[0]), 0, int(center[0]), h)
            
            qp.setPen(QPen(QBrush(QColor(0, 0, 0)), line_width))
            theta = - math.fmod(self.value / 180 * math.pi, 2 * math.pi) # degree? radian?
            A = (r * math.cos(theta) + center[0], r * math.sin(theta) + center[1])
            B = (r * math.cos(theta + math.pi) + center[0], r * math.sin(theta + math.pi) + center[1])
            C = (r * math.cos(theta + math.pi / 2.0) + center[0], r * math.sin(theta + math.pi / 2.0) + center[1])
            qp.drawLine(int(A[0]), int(A[1]), int(B[0]), int(B[1]))
            qp.drawLine(int(C[0]), int(C[1]), int(center[0]), int(center[1]))
            qp.fillRect(QtCore.QRect(center[0] - 40, center[1] - 20, 40 * 2, 40), QColor(255, 255, 255))
            
            qp.drawText(rect, QtCore.Qt.AlignCenter, str(int(self.value)))

            qp.end()
    
class StepGageWidget(QWidget):
    """
    QWidget to visualize step gage
    """
    def __init__(self, value_topic, max_value_topic, min_value_topic):
        super(StepGageWidget, self).__init__()
        self.lock = Lock()
        self.value = 0.5
        self.max_value = 1.0
        self.min_value = 0.0
        self.value_sub = rospy.Subscriber(
            value_topic,
            std_msgs.msg.Float32, self.valueCallback)
        self.min_value_sub = rospy.Subscriber(
            min_value_topic,
            std_msgs.msg.Float32, self.minValueCallback)
        self.max_value_sub = rospy.Subscriber(
            max_value_topic,
            std_msgs.msg.Float32, self.maxValueCallback)
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.redraw)
        self._update_plot_timer.start(1000 / 15) # 15 fpsw

    def valueCallback(self, msg):
        with self.lock:
            self.value = msg.data
    def maxValueCallback(self, msg):
        with self.lock:
            self.max_value = msg.data
    def minValueCallback(self, msg):
        with self.lock:
            self.min_value = msg.data
    def paintEvent(self, event):
        with self.lock:
            w = self.width() - 1
            h = self.height() - 1
            if w <= 0 or h <= 0:
                rospy.logdebug("[StepGageWidget] too small region to draw")
                return
            rect = QtCore.QRect(0, 0, w, h)
            painter = QtGui.QPainter()
            painter.begin(self)
            painter.setOpacity(0.8)
            painter.setPen(QtCore.Qt.black)
            painter.drawRect(rect)
            if (self.max_value - self.min_value) * h != 0:
                step_h = float(self.value - self.min_value) / (self.max_value - self.min_value) * h
            else:
                step_h = 0.0
            fill_rect = QtCore.QRect(1, (h - step_h), w - 1, step_h)
            painter.fillRect(fill_rect, QtGui.QColor("#18FFFF"))
            painter.end()
    def redraw(self):
        self.update()
