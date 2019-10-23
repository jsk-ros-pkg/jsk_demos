mech-user@KI00112:~$ source ~/catkin_ws/devel/setup.bash 
mech-user@KI00112:~$ source ~/semi_ws/devel/setup.bash 
mech-user@KI00112:~$ rossetip
set ROS_IP and ROS_HOSTNAME to 133.11.216.180
[http://133.11.216.115:11311][133.11.216.180] ]0;mech-user@KI00112: ~mech-user@KI00112:~$ rossetmaster
set ROS_MASTER_URI to http://pr1040:11311
[http://pr1040:11311][133.11.216.180] ]0;mech-user@KI00112: ~mech-user@KI00112:~$ rossetmaster 133.11.216.115
set ROS_MASTER_URI to http://133.11.216.115:11311
[http://133.11.216.115:11311][133.11.216.180] ]0;mech-user@KI00112: ~mech-user@KI00112:~$ roseus
configuring by "/opt/ros/melodic/share/euslisp/jskeus/eus//lib/eusrt.l"
;; readmacro ;; object ;; packsym ;; common ;; constants ;; stream ;; string ;; loader ;; pprint ;; process ;; hashtab ;; array ;; mathtran ;; eusdebug ;; eusforeign ;; coordinates ;; tty ;; history ;; toplevel ;; trans ;; comp ;; builtins ;; par ;; intersection ;; geoclasses ;; geopack ;; geobody ;; primt ;; compose ;; polygon ;; viewing ;; viewport ;; viewsurface ;; hid ;; shadow ;; bodyrel ;; dda ;; helpsub ;; eushelp ;; xforeign ;; Xdecl ;; Xgraphics ;; Xcolor ;; Xeus ;; Xevent ;; Xpanel ;; Xitem ;; Xtext ;; Xmenu ;; Xscroll ;; Xcanvas ;; Xtop ;; Xapplwin 
connected to Xserver DISPLAY=:0
X events are being asynchronously monitored.
;; pixword ;; RGBHLS ;; convolve ;; piximage ;; pbmfile ;; image_correlation ;; oglforeign ;; gldecl ;; glconst ;; glforeign ;; gluconst ;; gluforeign ;; glxconst ;; glxforeign ;; eglforeign ;; eglfunc ;; glutil ;; gltexture ;; glprim ;; gleus ;; glview ;; toiv-undefined ;; fstringdouble irtmath irtutil irtc irtgeoc irtgraph ___time ___pgsql irtgeo euspqp pqp irtscene irtmodel irtdyna irtrobot irtsensor irtbvh irtcollada irtpointcloud irtx eusjpeg euspng png irtimage irtglrgb 
;; extending gcstack 0x5639f8fd4690[16374] --> 0x5639f94501a0[32748] top=3d61
irtgl irtglc irtviewer 
EusLisp 9.26( 1.2.1) for Linux64 created on ip-172-30-1-203(Fri May 31 16:58:14 PST 2019)
roseus ;; loading roseus("1.7.4") on euslisp((9.26 ip-172-30-1-203 Fri May 31 16:58:14 PST 2019  1.2.1))
eustf roseus_c_util 1.irteusgl(load "package://naoeus/nao-interface.l")

;; extending gcstack 0x5639f94501a0[32738] --> 0x5639fa90d8a0[65476] top=7f4e
t
2.irteusgl$ (nao-init)
PQP Error! EndModel() called on model with no triangles
PQP Error! EndModel() called on model with no triangles
PQP Error! EndModel() called on model with no triangles
(:robot #<metaclass #X5639fa88e0e0 nao-robot> :group-namespace "" :naoqi-namespace "nao_robot" :dcm-namespace "nao_dcm" :group-namespace "")
PQP Error! EndModel() called on model with no triangles
PQP Error! EndModel() called on model with no triangles
PQP Error! EndModel() called on model with no triangles
nil
3.irteusgl$ (objects (list *nao*))
;; (make-irtviewer) executed
(#<naoh25v50-robot #X5639f94624f0 NaoH25V50  0.0 0.0 0.0 / 0.0 0.0 0.0>)
4.irteusgl$ (send *nao* :head :neck-y :joint-angle 30)
30
5.irteusgl$ (send *ri* :angle-vector (send *nao* :angle-vector) 5000)
#f(112.147 12.3024 -90.7067 -39.2852 1.49175 109.252 -12.483 91.7566 37.8837 3.16169 0.002404 0.002404 -29.2655 60.2913 -29.9735 0.002404 0.002404 0.002404 -29.8856 60.2082 -30.0566 0.002404 30.0 -0.793436)
6.irteusgl$ (send *ri* :speak "hello")
t
7.irteusgl$ (send *ri* :go-pos 0.1 0 0)
t
8.irteusgl$ 