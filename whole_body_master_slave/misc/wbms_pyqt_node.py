#!/usr/bin/env python
# coding=utf-8
import numpy as np
import scipy as sp
import scipy.spatial
import sys
import time as tm
import threading
import signal
import rospy
import math
import tf
from std_msgs.msg import *
from geometry_msgs.msg import *
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph.opengl as gl

sub_val_h_com = PoseStamped()
sub_val_h_rf = PoseStamped()
sub_val_h_lf = PoseStamped()
sub_val_h_rh = PoseStamped()
sub_val_h_lh = PoseStamped()
sub_val_h_head = PoseStamped()
sub_val_r_com = PoseStamped()
sub_val_r_rf = PoseStamped()
sub_val_r_lf = PoseStamped()
sub_val_r_rh = PoseStamped()
sub_val_r_lh = PoseStamped()
sub_val_r_head = PoseStamped()
sub_val_r_dcp = PointStamped()
sub_val_r_acp = PointStamped()
sub_val_r_zmp = PointStamped()
args = sys.argv
MODE = ""

if len(args) == 1 :
  MODE = "ROS"
  print "start subscription as a ROS node"
elif len(args) == 2 :
  MODE = "FILE"
  LOGNAME = args[1]
  STARTTIME = 0.0
  print "start plotting "+LOGNAME+" from "+str(STARTTIME)+" [s]"
elif len(args) == 3 :
  MODE = "FILE"
  LOGNAME = args[1]
  STARTTIME = float(args[2])
  print "start plotting "+LOGNAME+" from "+str(STARTTIME)+" [s]"
else:
  print "\033[91m"+"Invalid command line args (rosrun [pkg_name] *.py) or (rosrun [pkg_name] *.py [hrpsys logname] [start time])"+"\033[0m"
  sys.exit()

HZ = 500
PLOTHZ = 100
_X = 0
_Y = 1
_Z = 2
_XYZ = 3

if MODE == "FILE":
  hcomf = open(LOGNAME+".abc_htcom_dbgOut", 'r')
  hrff = open(LOGNAME+".abc_htrf_dbgOut", 'r')
  hlff = open(LOGNAME+".abc_htlf_dbgOut", 'r')
  rdcpf = open(LOGNAME+".abc_rpdcp_dbgOut", 'r')
  racpf = open(LOGNAME+".abc_rpacp_dbgOut", 'r')
  rzmpf = open(LOGNAME+".abc_rpzmp_dbgOut", 'r')
  rcomf = open(LOGNAME+".abc_rpcom_dbgOut", 'r')
  rrff = open(LOGNAME+".abc_rprf_dbgOut", 'r')
  rlff = open(LOGNAME+".abc_rplf_dbgOut", 'r')
  hcomlines = hcomf.readlines()
  hrflines = hrff.readlines()
  hlflines = hlff.readlines()
  rdcplines = rdcpf.readlines()
  racplines = racpf.readlines()
  rzmplines = rzmpf.readlines()
  rcomlines = rcomf.readlines()
  rrflines = rrff.readlines()
  rlflines = rlff.readlines()
  LOGFRAMES = len(hcomlines)
  LOGTIMELENGTH = float(LOGFRAMES)/HZ

logtime = []
hcom = [[] for i in range(_XYZ)]
rcom = [[] for i in range(_XYZ)]
rdcp = [[] for i in range(_XYZ)]
racp = [[] for i in range(_XYZ)]
rzmp = [[] for i in range(_XYZ)]
hrf = [[] for i in range(_XYZ)]
hlf = [[] for i in range(_XYZ)]
rrf = [[] for i in range(_XYZ)]
rlf = [[] for i in range(_XYZ)]
rsr = [[[],[]] for i in range(20)]
  
time_old = tm.time()

def h_com_cb(msg):  global sub_val_h_com;  sub_val_h_com = msg
def h_rf_cb(msg):  global sub_val_h_rf;  sub_val_h_rf = msg; sub_val_h_rf.pose.position.x -= 0.15; sub_val_h_rf.pose.position.y += 0.25 ### ViveとXtion, あるいはスリッパ位置の補正
def h_lf_cb(msg):  global sub_val_h_lf;  sub_val_h_lf = msg; sub_val_h_lf.pose.position.x -= 0.15; sub_val_h_lf.pose.position.y += 0.25
def h_rh_cb(msg):  global sub_val_h_rh;  sub_val_h_rh = msg
def h_lh_cb(msg):  global sub_val_h_lh;  sub_val_h_lh = msg
def h_head_cb(msg):  global sub_val_h_head;  sub_val_h_head = msg
def r_com_cb(msg):  global sub_val_r_com;  sub_val_r_com = msg
def r_rf_cb(msg):  global sub_val_r_rf;  sub_val_r_rf = msg
def r_lf_cb(msg):  global sub_val_r_lf;  sub_val_r_lf = msg
def r_rh_cb(msg):  global sub_val_r_rh;  sub_val_r_rh = msg
def r_lh_cb(msg):  global sub_val_r_lh;  sub_val_r_lh = msg
def r_head_cb(msg):  global sub_val_r_head;  sub_val_r_head = msg
def r_dcp_cb(msg):  global sub_val_r_dcp;  sub_val_r_dcp = msg
def r_acp_cb(msg):  global sub_val_r_acp;  sub_val_r_acp = msg
def r_zmp_cb(msg):  global sub_val_r_zmp;  sub_val_r_zmp = msg

if __name__ == '__main__':
  signal.signal(signal.SIGINT, signal.SIG_DFL)
  if MODE == "ROS":
    rospy.init_node('wbms_matplotlib_node', anonymous=True)
#     rospy.Subscriber("/human_tracker_com_ref", PoseStamped, com_cb)
#     rospy.Subscriber("/human_tracker_rf_ref", PoseStamped, rf_cb)
#     rospy.Subscriber("/human_tracker_lf_ref", PoseStamped, lf_cb)
#     rospy.Subscriber("/human_tracker_rh_ref", PoseStamped, rh_cb)
#     rospy.Subscriber("/human_tracker_lh_ref", PoseStamped, lh_cb)
#     rospy.Subscriber("/human_tracker_rfw_ref", WrenchStamped, rfw_cb)
#     rospy.Subscriber("/human_tracker_lfw_ref", WrenchStamped, lfw_cb)
    rospy.Subscriber("/wbms/feedback/h_com", PoseStamped, h_com_cb)
    rospy.Subscriber("/wbms/feedback/h_rf", PoseStamped, h_rf_cb)
    rospy.Subscriber("/wbms/feedback/h_lf", PoseStamped, h_lf_cb)
    rospy.Subscriber("/wbms/feedback/h_rh", PoseStamped, h_rh_cb)
    rospy.Subscriber("/wbms/feedback/h_lh", PoseStamped, h_lh_cb)
    rospy.Subscriber("/wbms/feedback/h_head", PoseStamped, h_head_cb)
    rospy.Subscriber("/wbms/feedback/r_com", PoseStamped, r_com_cb)
    rospy.Subscriber("/wbms/feedback/r_rf", PoseStamped, r_rf_cb)
    rospy.Subscriber("/wbms/feedback/r_lf", PoseStamped, r_lf_cb)
    rospy.Subscriber("/wbms/feedback/r_rh", PoseStamped, r_rh_cb)
    rospy.Subscriber("/wbms/feedback/r_lh", PoseStamped, r_lh_cb)
    rospy.Subscriber("/wbms/feedback/r_head", PoseStamped, r_head_cb)
    rospy.Subscriber("/wbms/feedback/r_dcp", PointStamped, r_dcp_cb)
    rospy.Subscriber("/wbms/feedback/r_acp", PointStamped, r_acp_cb)
    rospy.Subscriber("/wbms/feedback/r_zmp", PointStamped, r_zmp_cb)
  ### アプリケーション作成
  app = QtGui.QApplication([])
  app.quitOnLastWindowClosed()
  ### メインウィンドウ
  mainWindow = QtGui.QMainWindow()
  mainWindow.setWindowTitle("COM-SupportRegion") # Title
  mainWindow.resize(1600, 1000) # Size
  ### settei
  pg.setConfigOptions(antialias=True)
  pg.useOpenGL = True
  ### キャンパス
  centralWid = QtGui.QWidget()
  mainWindow.setCentralWidget(centralWid)
  lay_h = QtGui.QHBoxLayout()
  centralWid.setLayout(lay_h)
  
  ### GL
  all_wid = gl.GLViewWidget()
  all_wid.setFixedSize(800, 1000)
  p=np.array([[],[],[]])
  p=p.transpose()
  all_PltItms = [gl.GLScatterPlotItem() for i in range(6)]
  for i in range(len(all_PltItms)): all_wid.addItem(all_PltItms[i])
  gz = gl.GLGridItem()
  gz.translate(0, 0, 0)
  all_wid.addItem(gz)
  lay_h.addWidget(all_wid)
  all_wid.show()
  
  ### migi 3
  lay_v = QtGui.QVBoxLayout()
  rightWid = QtGui.QWidget()
  rightWid.setLayout(lay_v)
  rightWid.setFixedSize(800, 1000)
  lay_h.addWidget(rightWid)
  
  com_wid = pg.PlotWidget(name="spectrum")
  com_PltItm = com_wid.getPlotItem()
#   com_PltItm.setLabel('left', "Y-pos[m]")
#   com_PltItm.setLabel('bottom', "X-pos[m]")
  com_PltItm.setLabel('left', "Back - X-pos[m] - Forward")
  com_PltItm.setLabel('bottom', "Left - Y-pos[m] - Right")
  com_PltItm.setAspectLocked(lock=True, ratio=1)
  com_PltItm.invertX(True)
  lay_v.addWidget(com_wid)
  
  rf_wid = pg.PlotWidget(name="rf")
  rf_wid.setFixedSize(800, 200)
  rf_PltItm = rf_wid.getPlotItem()
  rf_PltItm.setLabel("left", "RF height [m]")
  rf_PltItm.setLabel("bottom", "time [s]")
  lay_v.addWidget(rf_wid)
  
  lf_wid = pg.PlotWidget(name="lf")
  lf_wid.setFixedSize(800, 200)
  lf_PltItm = lf_wid.getPlotItem()
  lf_PltItm.setLabel("left", "LF height [m]")
  lf_PltItm.setLabel("bottom", "time [s]")
  lay_v.addWidget(lf_wid)
  
  mainWindow.show() ### ウィンドウ表示
  
  lines_2d = [com_PltItm.plot() for i in range(6)]
  hulls_2d = [com_PltItm.plot() for i in range(len(rsr))]
#   hull_h_2d = com_PltItm.plot()
  lines_rf = [rf_PltItm.plot() for i in range(2)]
  lines_lf = [lf_PltItm.plot() for i in range(2)]
  texts_2d = [pg.TextItem(text="Default text", anchor=(0,0)) for i in range(5)]
  for i in range(len(texts_2d)): com_PltItm.addItem(texts_2d[i])
  texts_rf = [pg.TextItem(text="Default text", anchor=(0,0)) for i in range(2)]
  for i in range(len(texts_rf)): rf_PltItm.addItem(texts_rf[i])
  texts_lf = [pg.TextItem(text="Default text", anchor=(0,0)) for i in range(2)]
  for i in range(len(texts_lf)): lf_PltItm.addItem(texts_lf[i])
  
  
  loop=0
  cur_s = 0.0
  fps_filtered = 0.0
  while 1 :
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    time_now = tm.time()
    elapsed_time = time_now - time_old
    fps_filtered = fps_filtered * 0.99 + (1.0/elapsed_time) * 0.01
    time_old = time_now
      
    if MODE == "ROS":
      cur_s = sub_val_r_com.header.stamp.to_sec()
      logtime.append(cur_s)
      
      
      
# 
#       from geometry_msgs.msg import Vector3
#       
#       e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
#       return Vector3(x=e[0], y=e[1], z=e[2])

      
      for i in range(_XYZ):
        hcom[i].append([sub_val_h_com.pose.position.x,sub_val_h_com.pose.position.y,sub_val_h_com.pose.position.z][i])
        rcom[i].append([sub_val_r_com.pose.position.x,sub_val_r_com.pose.position.y,sub_val_r_com.pose.position.z][i])
        hrf[i].append([sub_val_h_rf.pose.position.x,sub_val_h_rf.pose.position.y,sub_val_h_rf.pose.position.z][i])
        hlf[i].append([sub_val_h_lf.pose.position.x,sub_val_h_lf.pose.position.y,sub_val_h_lf.pose.position.z][i])
        rrf[i].append([sub_val_r_rf.pose.position.x,sub_val_r_rf.pose.position.y,sub_val_r_rf.pose.position.z][i])
        rlf[i].append([sub_val_r_lf.pose.position.x,sub_val_r_lf.pose.position.y,sub_val_r_lf.pose.position.z][i])
        rdcp[i].append([sub_val_r_dcp.point.x,sub_val_r_dcp.point.y,sub_val_r_dcp.point.z][i])
        racp[i].append([sub_val_r_acp.point.x,sub_val_r_acp.point.y,sub_val_r_acp.point.z][i])
        rzmp[i].append([sub_val_r_zmp.point.x,sub_val_r_zmp.point.y,sub_val_r_zmp.point.z][i])
      
    elif MODE == "FILE":  
      PLOTHZ = fps_filtered
      cur_s = float(loop)/PLOTHZ + STARTTIME
      cur_f = int(cur_s * HZ)
      if cur_f >= LOGFRAMES :
        loop = 0
        cur_s = float(loop)/PLOTHZ + STARTTIME
        cur_f = int(cur_s * HZ)
        logtime = []
        for i in range(_XYZ): hcom[i]=[]; rcom[i]=[]; rdcp[i]=[]; racp[i]=[]; rzmp[i]=[]; hrf[i]=[]; hlf[i]=[]; rrf[i]=[]; rlf[i]=[]; 
        
      hcomvals = hcomlines[cur_f].replace('\n', '').split(' ')
      for i in range(len(hcomvals)-2):        hcomvals[i] = float(hcomvals[i+1])
      rcomvals = rcomlines[cur_f].replace('\n', '').split(' ')
      for i in range(len(rcomvals)-2):        rcomvals[i] = float(rcomvals[i+1])
      rdcpvals = rdcplines[cur_f].replace('\n', '').split(' ')
      for i in range(len(rdcpvals)-2):        rdcpvals[i] = float(rdcpvals[i+1])
      racpvals = racplines[cur_f].replace('\n', '').split(' ')
      for i in range(len(racpvals)-2):        racpvals[i] = float(racpvals[i+1])
      rzmpvals = rzmplines[cur_f].replace('\n', '').split(' ')
      for i in range(len(rzmpvals)-2):        rzmpvals[i] = float(rzmpvals[i+1])
      hrfvals = hrflines[cur_f].replace('\n', '').split(' ')
      for i in range(len(hrfvals)-2):        hrfvals[i] = float(hrfvals[i+1])
      hlfvals = hlflines[cur_f].replace('\n', '').split(' ')
      for i in range(len(hlfvals)-2):        hlfvals[i] = float(hlfvals[i+1])
      rrfvals = rrflines[cur_f].replace('\n', '').split(' ')
      for i in range(len(rrfvals)-2):        rrfvals[i] = float(rrfvals[i+1])
      rlfvals = rlflines[cur_f].replace('\n', '').split(' ')
      for i in range(len(rlfvals)-2):        rlfvals[i] = float(rlfvals[i+1])
      logtime.append(cur_s)
      for i in range(_XYZ):
        hcom[i].append(hcomvals[i])
        rcom[i].append(rcomvals[i])
        rdcp[i].append(rdcpvals[i])
        racp[i].append(racpvals[i])
        rzmp[i].append(rzmpvals[i])
        hrf[i].append(hrfvals[i])
        hlf[i].append(hlfvals[i])
        rrf[i].append(rrfvals[i])
        rlf[i].append(rlfvals[i])
      
    if len(logtime) > 10 * fps_filtered and len(logtime) > 1:
      logtime.pop(0)
      for i in range(_XYZ):
        hcom[i].pop(0)
        rcom[i].pop(0)
        rdcp[i].pop(0)
        racp[i].pop(0)
        rzmp[i].pop(0)
        hrf[i].pop(0)
        hlf[i].pop(0)
        rrf[i].pop(0)
        rlf[i].pop(0)
    if len(rdcp[i]) > 50:
      for i in range(_XYZ):
        rdcp[i].pop(0)
        racp[i].pop(0)
        rzmp[i].pop(0)
      
    points = np.array([
                 [rrf[_X][-1]+0.02,rrf[_Y][-1]+0.02],
                 [rrf[_X][-1]+0.02,rrf[_Y][-1]+0.01],
                 [rrf[_X][-1]-0.01,rrf[_Y][-1]+0.02],
                 [rrf[_X][-1]-0.01,rrf[_Y][-1]+0.01],
                 [rlf[_X][-1]+0.02,rlf[_Y][-1]-0.02],
                 [rlf[_X][-1]+0.02,rlf[_Y][-1]-0.01],
                 [rlf[_X][-1]-0.01,rlf[_Y][-1]-0.02],
                 [rlf[_X][-1]-0.01,rlf[_Y][-1]-0.01],
                 ])
    hull = sp.spatial.ConvexHull(points)
    hull_ans = [[],[]]
    for i in range(len(hull.vertices)):
      hull_ans[_X].append(points[hull.vertices[i]][_X])
      hull_ans[_Y].append(points[hull.vertices[i]][_Y])
    hull_ans[_X].append(points[hull.vertices[0]][_X])
    hull_ans[_Y].append(points[hull.vertices[0]][_Y])
    rsr[-1] = hull_ans
    if loop % 10 == 0:
      rsr.append(hull_ans)
      rsr.pop(0)
      
    points = np.array([
                 [hrf[_X][-1]+0.02,hrf[_Y][-1]+0.02],
                 [hrf[_X][-1]+0.02,hrf[_Y][-1]+0.01],
                 [hrf[_X][-1]-0.01,hrf[_Y][-1]+0.02],
                 [hrf[_X][-1]-0.01,hrf[_Y][-1]+0.01],
                 [hlf[_X][-1]+0.02,hlf[_Y][-1]-0.02],
                 [hlf[_X][-1]+0.02,hlf[_Y][-1]-0.01],
                 [hlf[_X][-1]-0.01,hlf[_Y][-1]-0.02],
                 [hlf[_X][-1]-0.01,hlf[_Y][-1]-0.01],
                 ])
    hull = sp.spatial.ConvexHull(points)
    hsr = [[],[]]
    for i in range(len(hull.vertices)):
      hsr[_X].append(points[hull.vertices[i]][_X])
      hsr[_Y].append(points[hull.vertices[i]][_Y])
    hsr[_X].append(points[hull.vertices[0]][_X])
    hsr[_Y].append(points[hull.vertices[0]][_Y])
      
    if loop % 30 == 0:
      com_PltItm.enableAutoRange()
      com_PltItm.disableAutoRange()
      rf_PltItm.enableAutoRange()
      rf_PltItm.disableAutoRange()
      lf_PltItm.enableAutoRange()
      lf_PltItm.disableAutoRange()
    
    com_PltItm.setTitle("COM-SR plot @ "+str(int(fps_filtered))+" [fps]")
    
    scale_3d = 5
    
    for i in range(len(hcom)):
      brightness = int(255.0*i/len(hcom))
    all_PltItms[0].setData(pos=(np.array(hcom)*scale_3d).transpose(), size=5)
    all_PltItms[1].setData(pos=(np.array(rcom)*scale_3d).transpose(), size=5)
    all_PltItms[2].setData(pos=(np.array(hrf)*scale_3d).transpose(), size=5)
    all_PltItms[3].setData(pos=(np.array(hlf)*scale_3d).transpose(), size=5)
    all_PltItms[4].setData(pos=(np.array(rrf)*scale_3d).transpose(), size=5)
    all_PltItms[5].setData(pos=(np.array(rlf)*scale_3d).transpose(), size=5)
    
    lines_2d[0].setData(hcom[_Y],hcom[_X])
    lines_2d[0].setPen(pg.mkPen((200,200,200), style=QtCore.Qt.DotLine,width=2) )
    texts_2d[0].setText("HumanCOM")
    texts_2d[0].setPos(hcom[_Y][-1],hcom[_X][-1])
    
    lines_2d[1].setData(rzmp[_Y],rzmp[_X])
    lines_2d[1].setPen(pg.mkPen((255,255,0),width=3) )
    texts_2d[1].setText("ZMP")
    texts_2d[1].setPos(rzmp[_Y][-1],rzmp[_X][-1])
    
    lines_2d[2].setData(rdcp[_Y],rdcp[_X])
    lines_2d[2].setPen(pg.mkPen("r",width=3) )
    texts_2d[2].setText("DCP")
    texts_2d[2].setPos(rdcp[_Y][-1],rdcp[_X][-1])
    
    lines_2d[3].setData(racp[_Y],racp[_X])
    lines_2d[3].setPen(pg.mkPen("b",width=3) )
    texts_2d[3].setText("ACP")
    texts_2d[3].setPos(racp[_Y][-1],racp[_X][-1])
    
    lines_2d[4].setData(rcom[_Y],rcom[_X])
    lines_2d[4].setPen(pg.mkPen("g",width=4) )
    texts_2d[4].setText("RobotCOM")
    texts_2d[4].setPos(rcom[_Y][-1],rcom[_X][-1])
    
#     hull_h_2d.setData(hsr[_Y],hsr[_X])### Human SR
#     hull_h_2d.setPen(pg.mkPen("r", style=QtCore.Qt.DotLine,width=2) )
    
    for i in range(len(rsr)):
      hulls_2d[i].setData(rsr[i][_Y],rsr[i][_X])
      brightness = int(255.0*i/len(rsr))
      hulls_2d[i].setPen(pg.mkPen((brightness,brightness,brightness),width=1,style=QtCore.Qt.DashLine))
    
    lines_rf[0].setData(logtime,hrf[_Z],width=2)
    lines_rf[0].setPen(pg.mkPen("r", style=QtCore.Qt.DotLine) )
    texts_rf[0].setText("HumanRF")
    texts_rf[0].setPos(logtime[-1],hrf[_Z][-1])

    lines_rf[1].setData(logtime,rrf[_Z])
    lines_rf[1].setPen(pg.mkPen("g",width=3) )
    texts_rf[1].setText("RobotRF")
    texts_rf[1].setPos(logtime[-1],rrf[_Z][-1])
     
    lines_lf[0].setData(logtime,hlf[_Z],width=2)
    lines_lf[0].setPen(pg.mkPen("r", style=QtCore.Qt.DotLine) )
    texts_lf[0].setText("HumanLF")
    texts_lf[0].setPos(logtime[-1],hlf[_Z][-1])
     
    lines_lf[1].setData(logtime,rlf[_Z])
    lines_lf[1].setPen(pg.mkPen("g",width=3) )
    texts_lf[1].setText("RobotLF")
    texts_lf[1].setPos(logtime[-1],rlf[_Z][-1])
    
    QtGui.QApplication.processEvents()
    
    loop+=1
