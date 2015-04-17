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
from drc_task_common.srv import SetValue
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
    def setUpLeftBox(self, left_vbox):
        self.mode_list = QListWidget()
        self.mode_list.addItem("Manual")
        self.mode_list.addItem("Auto")
        left_vbox.addWidget(self.mode_list)

        action_vbox = QtGui.QVBoxLayout(self)
        action_group = QtGui.QGroupBox("action", self)
        self.initialize_button = QtGui.QPushButton("Initialize Pose")
        self.grasp_button = QtGui.QPushButton("Grasp")
        self.release_button = QtGui.QPushButton("Release")
        self.correct_button = QtGui.QPushButton("Correct")
        self.initialize_button.clicked.connect(self.service("/drive/controller/initialize"))
        self.grasp_button.clicked.connect(self.service("/drive/controller/grasp"))
        self.release_button.clicked.connect(self.service("/drive/controller/release"))
        self.correct_button.clicked.connect(self.service("/drive/controller/correct"))
        action_vbox.addWidget(self.initialize_button)
        action_vbox.addWidget(self.grasp_button)
        action_vbox.addWidget(self.release_button)
        action_vbox.addWidget(self.correct_button)
        action_group.setLayout(action_vbox)
        left_vbox.addWidget(action_group)

        overwrite_vbox = QtGui.QVBoxLayout(self)
        overwrite_group = QtGui.QGroupBox("", self)
        self.overwrite_button = QtGui.QPushButton("Overwrite")
        overwrite_vbox.addWidget(self.overwrite_button)
        overwrite_group.setLayout(overwrite_vbox)
        left_vbox.addWidget(overwrite_group)

        step_vbox = QtGui.QVBoxLayout(self)
        step_group = QtGui.QGroupBox("step", self)
        step_max_hbox = QtGui.QHBoxLayout(self)
        step_max_label = QtGui.QLabel("Max", self)
        step_max_vbox = QtGui.QVBoxLayout(self)
        self.step_max_up_button = QtGui.QPushButton()
        self.step_max_up_button.setIcon(QIcon.fromTheme("go-up"))
        self.step_max_up_button.clicked.connect(self.maxUpButtonCallback)
        self.step_max_down_button = QtGui.QPushButton()
        self.step_max_down_button.setIcon(QIcon.fromTheme("go-down"))
        self.step_max_down_button.clicked.connect(self.maxDownButtonCallback)
        self.step_max_edit = QtGui.QLineEdit()
        self.step_max_edit.setText("-1")
        self.step_max_edit.returnPressed.connect(self.maxEditCallback)
        self.step_max_edit.setValidator(QtGui.QDoubleValidator(0, 100, 10))
        step_max_hbox.addWidget(step_max_label)
        step_max_vbox.addWidget(self.step_max_up_button)
        step_max_vbox.addWidget(self.step_max_edit)
        step_max_vbox.addWidget(self.step_max_down_button)
        step_max_hbox.addLayout(step_max_vbox)
        step_vbox.addLayout(step_max_hbox)

        step_min_hbox = QtGui.QHBoxLayout(self)
        step_min_label = QtGui.QLabel("Min", self)
        step_min_vbox = QtGui.QVBoxLayout(self)
        self.step_min_up_button = QtGui.QPushButton()
        self.step_min_up_button.setIcon(QIcon.fromTheme("go-up"))
        self.step_min_up_button.clicked.connect(self.minUpButtonCallback)
        self.step_min_down_button = QtGui.QPushButton()
        self.step_min_down_button.setIcon(QIcon.fromTheme("go-down"))
        self.step_min_down_button.clicked.connect(self.minDownButtonCallback)
        self.step_min_edit = QtGui.QLineEdit()
        self.step_min_edit.setText("-1")
        self.step_min_edit.returnPressed.connect(self.minEditCallback)
        self.step_min_edit.setValidator(QtGui.QDoubleValidator(0, 100, 10))
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
        # self.multisense_widget = ROSImageWidget("/cv_camera/image_raw") # for debugging
        center_box.addWidget(self.multisense_widget)
        left_splitter = QtGui.QSplitter()
        right_splitter = QtGui.QSplitter()
        left_container = QtGui.QWidget()
        right_container = QtGui.QWidget()
        lower_left_vbox = QtGui.QVBoxLayout()
        left_force_group = QtGui.QGroupBox("LLEG Force")
        left_force_vbox = QtGui.QVBoxLayout()
        self.left_force_label = QtGui.QLabel("-1")
        left_force_vbox.addWidget(self.left_force_label)
        left_force_group.setLayout(left_force_vbox)
        lower_left_vbox.addWidget(left_force_group)

        right_force_group = QtGui.QGroupBox("RLEG Force")
        right_force_vbox = QtGui.QVBoxLayout()
        self.right_force_label = QtGui.QLabel("-1")
        right_force_vbox.addWidget(self.right_force_label)
        right_force_group.setLayout(right_force_vbox)
        lower_left_vbox.addWidget(right_force_group)
        lower_left_container = QtGui.QWidget()
        lower_left_container.setLayout(lower_left_vbox)
        left_splitter.addWidget(lower_left_container)
        
        # self.fisheye_widget = ROSImageWidget("/cv_camera/image_raw") # for debugging
        self.fisheye_widget = ROSImageWidget("/chest_camera/image_rect_color")
        right_splitter.addWidget(self.fisheye_widget)

        angle_vbox = QtGui.QVBoxLayout()
        angle_container = QtGui.QWidget()
        self.goal_angle = AngleWidget("/drive/controller/goal_handle_angle")
        self.estimated_angle = AngleWidget("/drive/controller/estimated_handle_angle")
        angle_vbox.addWidget(self.goal_angle)
        angle_vbox.addWidget(self.estimated_angle)
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
        self.step_gage = StepGageWidget("/drive/step",
                                        "/drive/max_step",
                                        "/drive/min_step")
        right_vbox.addWidget(self.step_gage)
    # Message callback
    def setupSubscribers(self):
        self.step_gage_value_sub = rospy.Subscriber(
            "/drive/step", std_msgs.msg.Float32, self.stepGageValueCallback)
        self.min_step_value_sub = rospy.Subscriber(
            "/drive/min_step", std_msgs.msg.Float32, self.minStepGageValueCallback)
        self.max_step_value_sub = rospy.Subscriber(
            "/drive/max_step", std_msgs.msg.Float32, self.maxStepGageValueCallback)
        self.lleg_force_sub = rospy.Subscriber(
            "/lfsensor", geometry_msgs.msg.WrenchStamped, self.lfsensorCallback)
        self.rleg_force_sub = rospy.Subscriber(
            "/rfsensor", geometry_msgs.msg.WrenchStamped, self.rfsensorCallback)
    def lfsensorCallback(self, msg):
        self.left_force_label.setText(str(msg.wrench.force.z))
    def rfsensorCallback(self, msg):
        self.right_force_label.setText(str(msg.wrench.force.z))
    def stepGageValueCallback(self, msg):
        self.step_gage_label.setText(str(msg.data))
    def minStepGageValueCallback(self, msg):
        self.step_min_edit.setText(str(msg.data))
    def maxStepGageValueCallback(self, msg):
        self.step_max_edit.setText(str(msg.data))
    # Event callback
    def minUpButtonCallback(self, event):
        current_value = float(self.step_min_edit.text())
        current_value = current_value + 1.0
        try:
            update_value = rospy.ServiceProxy('/drive/controller/set_min_step', SetValue)
            next_value = update_value(current_value)
            self.step_min_edit.setText(str(next_value.set_value))
        except rospy.ServiceException, e:
            self.showError("Failed to call /drive/controller/set_min_step")
    def minDownButtonCallback(self, event):
        current_value = float(self.step_min_edit.text())
        current_value = current_value - 1.0
        try:
            update_value = rospy.ServiceProxy('/drive/controller/set_min_step', SetValue)
            next_value = update_value(current_value)
            self.step_min_edit.setText(str(next_value.set_value))
        except rospy.ServiceException, e:
            self.showError("Failed to call /drive/controller/set_min_step")
    def maxEditCallback(self):
        current_value = float(self.step_max_edit.text())
        try:
            update_value = rospy.ServiceProxy('/drive/controller/set_max_step', SetValue)
            self.step_max_edit.setText(str(current_value))
        except rospy.ServiceException, e:
            self.showError("Failed to call /drive/controller/set_max_step")
    def minEditCallback(self):
        current_value = float(self.step_min_edit.text())
        try:
            update_value = rospy.ServiceProxy('/drive/controller/set_min_step', SetValue)
            self.step_min_edit.setText(str(current_value))
        except rospy.ServiceException, e:
            self.showError("Failed to call /drive/controller/set_min_step")
    def maxUpButtonCallback(self, event):
        current_value = float(self.step_max_edit.text())
        current_value = current_value + 1.0
        try:
            update_value = rospy.ServiceProxy('/drive/controller/set_max_step', SetValue)
            next_value = update_value(current_value)
            self.step_max_edit.setText(str(next_value.set_value))
        except rospy.ServiceException, e:
            self.showError("Failed to call /drive/controller/set_max_step")
    def maxDownButtonCallback(self, event):
        current_value = float(self.step_max_edit.text())
        current_value = current_value - 1.0
        try:
            update_value = rospy.ServiceProxy('/drive/controller/set_max_step', SetValue)
            next_value = update_value(current_value)
            self.step_max_edit.setText(str(next_value.set_value))
        except rospy.ServiceException, e:
            self.showError("Failed to call /drive/controller/set_max_step")
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
    QWidget to visualize angle of stearing handle
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
            line_width = 4
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
            step_h = float(self.value - self.min_value) / (self.max_value - self.min_value) * h
            fill_rect = QtCore.QRect(1, (h - step_h), w - 1, step_h)
            painter.fillRect(fill_rect, QtGui.QColor("#18FFFF"))
            painter.end()
    def redraw(self):
        self.update()
