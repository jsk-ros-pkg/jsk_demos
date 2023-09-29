# robot_head_ui.py

from jsk_rqt_plugins.image_view2_wrapper import ImageView2Widget
import rospy
from rqt_gui_py.plugin import Plugin
from python_qt_binding import QtGui, QtCore
from urdf_parser_py.urdf import URDF
import math
from sensor_msgs.msg import JointState
from threading import Lock
from drc_task_common.srv import SetValue

def rad2deg(v):
  return v / math.pi * 180.0

def deg2rad(v):
  return v / 180.0 * math.pi

class RobotHeadPlugin(Plugin):
  def __init__(self, context):
    super(RobotHeadPlugin, self).__init__(context)
    self.setObjectName("ImageView2Plugin")
    self._widget = RobotHeadWidget()
    context.add_widget(self._widget)
  def save_settings(self, plugin_settings, instance_settings):
    self._widget.save_settings(plugin_settings, instance_settings)
  def restore_settings(self, plugin_settings, instance_settings):
    self._widget.restore_settings(plugin_settings, instance_settings)
  def trigger_configuration(self):
    self._widget.trigger_configuration()


class RobotHeadWidget(QtGui.QWidget):
  def __init__(self):
    super(RobotHeadWidget, self).__init__()
    self.lock = Lock()
    try:
      robot_model = URDF.from_xml_string(rospy.get_param("/robot_description"))
    except Exception as e:
      self.showError("Failed to load /robot_description: " + e.message)
      return
    # Parse urdf
    try:
      yaw_joint = [j for j in robot_model.joints if j.name == "HEAD_JOINT0"][0]
      pitch_joint = [j for j in robot_model.joints if j.name == "HEAD_JOINT1"][0]
      self.yaw_upper = rad2deg(yaw_joint.limit.upper)
      self.yaw_lower = rad2deg(yaw_joint.limit.lower)
      self.pitch_upper = rad2deg(pitch_joint.limit.upper)
      self.pitch_lower = rad2deg(pitch_joint.limit.lower)
    except Exception as e:
      self.showError("Failed to load correct min/max joint angles: " + e.message)
      return
    vsplitter = QtGui.QSplitter(QtCore.Qt.Vertical)
    hsplitter = QtGui.QSplitter(QtCore.Qt.Horizontal)
    self.image_widget = ImageView2Widget()
    self.vslider = QtGui.QSlider(QtCore.Qt.Vertical)
    self.hslider = QtGui.QSlider(QtCore.Qt.Horizontal)
    self.hslider.setMinimum(self.yaw_lower)
    self.hslider.setMaximum(self.yaw_upper)
    self.vslider.setMinimum(self.pitch_lower)
    self.vslider.setMaximum(self.pitch_upper)
    # self.hslider.setInvertedControls(False)
    # self.vslider.setInvertedControls(False)
    self.hslider.setInvertedAppearance(True)
    self.vslider.setInvertedAppearance(True)
    self.movingp = False
    self.vslider.sliderPressed.connect(self.moveLock)
    self.hslider.sliderPressed.connect(self.moveLock)
    self.vslider.sliderMoved.connect(self.moveLock)
    self.hslider.sliderMoved.connect(self.moveLock)
    self.vslider.sliderReleased.connect(self.vsliderReleased)
    self.hslider.sliderReleased.connect(self.hsliderReleased)
    hsplitter.addWidget(self.image_widget)    
    hsplitter.addWidget(self.vslider)
    vsplitter.addWidget(hsplitter)
    vsplitter.addWidget(self.hslider)
    hsplitter.setSizes((300, 1))
    vsplitter.setSizes((300, 1))
    box = QtGui.QVBoxLayout()
    box.addWidget(vsplitter)
    self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.jointCallback)
    self.setLayout(box)
  def vsliderReleased(self):
    with self.lock:
      self.movingp = False
      try:
        set_pitch_angle = rospy.ServiceProxy("/set_pitch_angle", SetValue)
        set_pitch_angle(deg2rad(self.vslider.value()))
      except:
        self.showError("Failed to call /set_pitch_angle")
  def hsliderReleased(self):
    with self.lock:
      self.movingp = False
      try:
        set_yaw_angle = rospy.ServiceProxy("/set_yaw_angle", SetValue)
        set_yaw_angle(deg2rad(self.hslider.value()))
      except:
        self.showError("Failed to call /set_yaw_angle")
  def moveLock(self):
    with self.lock:
      self.movingp = True
  def showError(self, message):
    QtGui.QMessageBox.about(self, "ERROR", message)
  def save_settings(self, plugin_settings, instance_settings):
    self.image_widget.save_settings(plugin_settings, instance_settings)
  def restore_settings(self, plugin_settings, instance_settings):
    self.image_widget.restore_settings(plugin_settings, instance_settings)
  def trigger_configuration(self):
    self.image_widget.trigger_configuration()
  def jointCallback(self, msg):
    with self.lock:
      if self.movingp:
        # Now it's moving
        return
      if "HEAD_JOINT0" in msg.name:
        self.hslider.setValue(rad2deg(msg.position[msg.name.index("HEAD_JOINT0")]))
      if "HEAD_JOINT1" in msg.name:
        self.vslider.setValue(rad2deg(msg.position[msg.name.index("HEAD_JOINT1")]))
