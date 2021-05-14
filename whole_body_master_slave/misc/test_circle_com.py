#!/usr/bin/python
# -*- coding: utf-8 -*-
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import sys
import threading
import time
import signal
import rospy
import tf
import math
from std_msgs.msg import *
from geometry_msgs.msg import *

pub_val_com = PoseStamped()
pub_val_rf = PoseStamped()
pub_val_lf = PoseStamped()
pub_val_rh = PoseStamped()
pub_val_lh = PoseStamped()
pub_val_rfw = WrenchStamped()
pub_val_lfw = WrenchStamped()
pub_val_list = [pub_val_com, pub_val_rf, pub_val_lf, pub_val_rh, pub_val_lh, pub_val_rfw, pub_val_lfw]

key_list = ["com", "rf", "lf", "rh", "lh", "rfw", "lfw"]
label_list = ["X","Y","Z","R","P","Y"]
topic_d = {"com":PoseStamped, "rf":PoseStamped, "lf":PoseStamped, "rh":PoseStamped, "lh":PoseStamped, "rfw":WrenchStamped, "lfw":WrenchStamped}
POSMINMAX = 0.5/100.0
ROTMINMAX = 45.0/180.0*math.pi/100.0
testrad = 0

class TestThread(threading.Thread):

  def __init__(self):
    super(TestThread, self).__init__()
    self.setDaemon(True)

  def run(self):
    print "enter ROS pub thread"
    loop = 0
    while not rospy.is_shutdown():
      loop += 1
      for i in range(len(pub_list)):
        
        pub_val_list[0].pose.position.x = testrad * math.sin(loop/100.0 * 2*math.pi /10)
        pub_val_list[0].pose.position.y = testrad * math.cos(loop/100.0 * 2*math.pi /10)
        
        pub_list[i].publish(pub_val_list[i])
      r.sleep()

class App(QMainWindow):
  
  def main(self):
    self.w = QWidget()
    self.w.resize(800, 1000)
    self.w.setWindowTitle('Test Value Publisher')
    
    self.label_list = []
    self.slider_list = []
    
    allpubbox = QVBoxLayout()
    
    for i in range(len(key_list)):
      self.label_list.append(QLabel('Initial label '+key_list[i]))
      pubbox = QHBoxLayout()
      pubbox.setAlignment(self.label_list[i], Qt.AlignVCenter)
      vbox_L  = QVBoxLayout()
      vbox_R = QVBoxLayout()
      pubbox.addLayout(vbox_L)
      pubbox.addLayout(vbox_R)
      for j in range( len(label_list) ):
        self.slider_list.append([])
        self.slider_list[i].append(QSlider(Qt.Horizontal))
        self.slider_list[i][j].setRange(-100, 100)  # スライダの範囲
        self.slider_list[i][j].setValue(0)  # 初期値
        self.slider_list[i][j].setTickPosition(QSlider.TicksBothSides) #スライダの目盛りを両方に出す
        self.connect(self.slider_list[i][j], SIGNAL('valueChanged(int)'), self.on_draw)
        hbox= QHBoxLayout()
        hbox.addWidget(QLabel(label_list[j]))
        hbox.addWidget(self.slider_list[i][j])
        if j<3:
          vbox_L.addLayout(hbox)
          vbox_L.setAlignment(self.slider_list[i][j], Qt.AlignVCenter)
        else:
          vbox_R.addLayout(hbox)
          vbox_R.setAlignment(self.slider_list[i][j], Qt.AlignVCenter)
      allpubbox.addWidget(self.label_list[i])
      allpubbox.addLayout(pubbox)

    self.w.setLayout(allpubbox)
    self.w.show()
  
  def on_draw(self):
    global pub_val_list, testrad
    
    for i in range(0, len(key_list)-2):
      pub_val_list[i].pose.position.x = self.slider_list[i][0].value() *POSMINMAX
      pub_val_list[i].pose.position.y = self.slider_list[i][1].value() *POSMINMAX
      pub_val_list[i].pose.position.z = self.slider_list[i][2].value() *POSMINMAX
      quaternion = tf.transformations.quaternion_from_euler(self.slider_list[i][3].value() *ROTMINMAX, self.slider_list[i][4].value() *ROTMINMAX, self.slider_list[i][5].value() *ROTMINMAX)
      pub_val_list[i].pose.orientation.x = quaternion[0]
      pub_val_list[i].pose.orientation.y = quaternion[1]
      pub_val_list[i].pose.orientation.z = quaternion[2]
      pub_val_list[i].pose.orientation.w = quaternion[3]
      
      self.label_list[i].setText( key_list[i] \
                                  + ": %+4.3f"%(self.slider_list[i][0].value() *POSMINMAX) \
                                  + " %+4.3f"%(self.slider_list[i][1].value() *POSMINMAX) \
                                  + " %+4.3f"%(self.slider_list[i][2].value() *POSMINMAX) \
                                  + " %+4.3f"%(self.slider_list[i][3].value() *ROTMINMAX *180 /math.pi) \
                                  + " %+4.3f"%(self.slider_list[i][4].value() *ROTMINMAX *180 /math.pi) \
                                  + " %+4.3f"%(self.slider_list[i][5].value() *ROTMINMAX *180 /math.pi) )
      
    for i in range(len(key_list)-2, len(key_list)):
      pub_val_list[i].wrench.force.x = self.slider_list[i][0].value() * 10.0
      pub_val_list[i].wrench.force.y = self.slider_list[i][1].value() * 10.0
      pub_val_list[i].wrench.force.z = self.slider_list[i][2].value() * 10.0
      self.label_list[i].setText( key_list[i] + ": %+4.3f"%pub_val_list[i].wrench.force.x + " %+4.3f"%pub_val_list[i].wrench.force.y + " %+4.3f"%pub_val_list[i].wrench.force.z)

    testrad = pub_val_list[0].pose.position.y

if __name__ == '__main__':
  signal.signal(signal.SIGINT, signal.SIG_DFL)
  pub_com = rospy.Publisher('/human_tracker_com_ref', PoseStamped, queue_size=10)
  pub_rf = rospy.Publisher('/human_tracker_rf_ref', PoseStamped, queue_size=10)
  pub_lf = rospy.Publisher('/human_tracker_lf_ref', PoseStamped, queue_size=10)
  pub_rh = rospy.Publisher('/human_tracker_rh_ref', PoseStamped, queue_size=10)
  pub_lh = rospy.Publisher('/human_tracker_lh_ref', PoseStamped, queue_size=10)
  pub_rfw = rospy.Publisher('/human_tracker_rfw_ref', WrenchStamped, queue_size=10)
  pub_lfw = rospy.Publisher('/human_tracker_lfw_ref', WrenchStamped, queue_size=10)
  pub_list = [pub_com,pub_rf,pub_lf,pub_rh,pub_lh,pub_rfw,pub_lfw]
  
  rospy.init_node('humansync_test_publisher', anonymous=True)
  r = rospy.Rate(100)
  
  th_cl = TestThread()
  th_cl.start()
  
  
  app = QApplication(sys.argv)
  mainApp = App()
  mainApp.main()
  app.exec_()