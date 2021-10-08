#!/usr/bin/env roseus

(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(345 280 1330) :rpy (float-vector -1.57 1.57 0))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :larm :move-end-pos #f(0 0 200))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :larm :move-end-pos #f(30 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :larm)
(unix::sleep 1)

#| state of this position
#f(-45.7282 -24.4921 60.716 -105.043 21.0379 92.2515 147.288 -24.7494 -8.22244 -5.22154 -104.346 -0.698772 95.9557 -74.1599)
|#

(send *robot* :larm :move-end-pos #f(-100 -40 0))
(send *robot* :rarm :move-end-pos #f(-100 40 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :larm :move-end-pos #f(-40 30 0))
(send *robot* :rarm :move-end-pos #f(-40 -30 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :larm :move-end-pos #f(-30 0 0))
(send *robot* :rarm :move-end-pos #f(-30 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :larm :move-end-pos #f(0 50 0))
(send *robot* :rarm :move-end-pos #f(0 -50 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :rarm)
(send *ri* :stop-grasp :larm)
(unix::sleep 2)

(send *robot* :larm :move-end-pos #f(-20 -50 -50))
(send *robot* :rarm :move-end-pos #f(-20 50 -50))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(unix::sleep 10)

(send *robot* :reset-pose)
(send *robot* :larm :move-end-pos #f(-500 0 0))
(send *robot* :rarm :move-end-pos #f(-500 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)
