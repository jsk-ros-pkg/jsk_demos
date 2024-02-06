#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")

(dual_panda-init)

(objects (list *robot*))

;setting
(send *ri* :stop-grasp :rarm)
(send *ri* :stop-grasp :larm)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(444 -96.5 1301.6) :rpy (float-vector 0.0 1.57 -1.57))) :translation-axis t :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(432.17 224.157 1400) :rpy (float-vector -1.57 0.84 1.60))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)

;grasp
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(433 170.8 1340.32) :rpy (float-vector -1.57 0.84 1.60))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :larm)
(unix::sleep 1)

(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(433 185.8 1310.22) :rpy (float-vector -1.533 0.56 1.63))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(433 185.8 1280.22) :rpy (float-vector -1.533 0.0 1.63))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :larm)

(send *robot* :larm :move-end-pos #f(-100 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)
