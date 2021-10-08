#!/usr/bin/env roseus

(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(345 260 1290) :rpy (float-vector -1.57 0 0))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(345 210 1290) :rpy (float-vector -1.57 0 0))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *ri* :start-grasp :larm)
(unix::sleep 1)

(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(345 210 1290) :rpy (float-vector -1.57 0 -1.57))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :rarm)
(unix::sleep 1)

(send *robot* :rarm :move-end-pos #f(-120 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :move-end-pos #f(0 0 150))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :move-end-pos #f(50 0 50))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(unix::sleep 1)

;start to insertion
(send *robot* :rarm :move-end-pos #f(20 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :rarm)
(unix::sleep 1)

(send *robot* :rarm :move-end-pos #f(-20 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)
;end insertion

(send *robot* :rarm :move-end-pos #f(-50 0 -50))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :move-end-pos #f(0 0 -150))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :move-end-pos #f(120 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(unix::sleep 1)

(send *ri* :stop-grasp :larm)
(unix::sleep 1)

(send *robot* :larm :move-end-pos #f(-70 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;;check if successed cable-insertion

;;end checking

