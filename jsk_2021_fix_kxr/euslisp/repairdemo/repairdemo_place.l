#!/usr/bin/env roseus

;;before grasp position
(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(345 -246.5 1330) :rpy (float-vector 0.0 1.57 -1.57))) :translation-axis t :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(345 -84.8 1330) :rpy (float-vector 1.57 1.57 -3.14))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :larm :move-end-pos #f(30 0 0))
(send *robot* :rarm :move-end-pos #f(30 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(send *ri* :start-grasp :larm)
(unix::sleep 2)

;lift KXR
(send *robot* :larm :move-end-pos #f(-15 0 0))
(send *robot* :rarm :move-end-pos #f(-15 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;place KXR to cable_insert
(send *robot* :larm :move-end-pos #f(0 0 -150))
(send *robot* :rarm :move-end-pos #f(0 0 150))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :larm :move-end-pos #f(15 0 0))
(send *robot* :rarm :move-end-pos #f(15 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :larm)
(unix::sleep 2)

(send *robot* :larm :move-end-pos #f(-100 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :larm :move-end-pos #f(0 0 -200))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)
