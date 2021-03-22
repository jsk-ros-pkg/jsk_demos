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

keys = ["com", "lleg", "rleg", "larm", "rarm", "head", "lhand", "rhand"]
ids = ["X","Y","Z","r","p","y"]
vals = [PoseStamped() for k in keys]

POSMINMAX = 1.0/100.0
ROTMINMAX = 60.0/180.0*math.pi/100.0

class TestThread(threading.Thread):

  def __init__(self):
    super(TestThread, self).__init__()
    self.setDaemon(True)

  def run(self):
    global vals
    print "enter ROS pub thread"
 
    for v in vals:
      v.pose.orientation.w = 1 # quatanion 0,0,0,0は色々バグる
        
    while not rospy.is_shutdown():
      for v, p in zip(vals, pubs):
        v.header.stamp = rospy.Time.now()
        p.publish(v)
      r.sleep()

class App(QMainWindow):
  
  def main(self):
    self.w = QWidget()
    self.w.resize(800, 1000)
    self.w.setWindowTitle('Test Value Publisher')
    self.labels = [QLabel('Initial label '+k) for k in keys]
    self.sliders = [[QSlider(Qt.Horizontal) for i in ids] for k in keys]
    allpubbox = QVBoxLayout()
    
    for k,l,ss in zip(keys, self.labels, self.sliders):
      pubbox = QHBoxLayout()
      pubbox.setAlignment(l, Qt.AlignVCenter)
      vbox_L = QVBoxLayout()
      vbox_R = QVBoxLayout()
      pubbox.addLayout(vbox_L)
      pubbox.addLayout(vbox_R)
      for i, s in zip(ids, ss):
        s.setRange(-100, 100)  # スライダの範囲
        s.setValue(0)  # 初期値
        #s.setTickPosition(QSlider.TicksBothSides) #スライダの目盛りを両方に出す
        s.setTickPosition(QSlider.TicksBelow)
        self.connect(s, SIGNAL('valueChanged(int)'), self.on_draw)
        hbox= QHBoxLayout()
        hbox.addWidget(QLabel(i))
        hbox.addWidget(s)
        if i in ["X","Y","Z"]:
          vbox_L.addLayout(hbox)
          vbox_L.setAlignment(s, Qt.AlignVCenter)
        else:
          vbox_R.addLayout(hbox)
          vbox_R.setAlignment(s, Qt.AlignVCenter)
      allpubbox.addWidget(l)
      allpubbox.addLayout(pubbox)

    self.w.setLayout(allpubbox)
    self.w.show()
  
  def on_draw(self):
    global vals
    for k, v, s, l in zip(keys, vals, self.sliders, self.labels):
      v.header.frame_id = "test_"+k+"_frame"
      v.pose.position.x = s[0].value()*POSMINMAX
      v.pose.position.y = s[1].value()*POSMINMAX
      v.pose.position.z = s[2].value()*POSMINMAX
      quaternion = tf.transformations.quaternion_from_euler(s[3].value()*ROTMINMAX, s[4].value()*ROTMINMAX, s[5].value()*ROTMINMAX)
      v.pose.orientation.x = quaternion[0]
      v.pose.orientation.y = quaternion[1]
      v.pose.orientation.z = quaternion[2]
      v.pose.orientation.w = quaternion[3]
      l.setText( k \
                 + ": %+4.3f"%(s[0].value() *POSMINMAX) \
                 + " %+4.3f"%(s[1].value() *POSMINMAX) \
                 + " %+4.3f"%(s[2].value() *POSMINMAX) \
                 + " %+4.3f"%(s[3].value() *ROTMINMAX *180 /math.pi) \
                 + " %+4.3f"%(s[4].value() *ROTMINMAX *180 /math.pi) \
                 + " %+4.3f"%(s[5].value() *ROTMINMAX *180 /math.pi) )
      
if __name__ == '__main__':
  signal.signal(signal.SIGINT, signal.SIG_DFL)
  pubs = [rospy.Publisher('/master_'+k+'_pose', PoseStamped, queue_size=10) for k in keys]
  
  rospy.init_node('humansync_test_publisher', anonymous=True)
  r = rospy.Rate(100)
  
  th_cl = TestThread()
  th_cl.start()
  
  app = QApplication(sys.argv)
  signal.signal(signal.SIGINT, signal.SIG_DFL)
  mainApp = App()
  mainApp.main()
  app.exec_()
