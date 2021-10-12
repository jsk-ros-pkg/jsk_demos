#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")

(dual_panda-init)

(objects (list *robot*))


(send *ri* :stop-grasp :rarm)
(send *ri* :stop-grasp :larm)
(unix::sleep 2)

(send *robot* :reset-pose)
(send *robot* :larm :move-end-pos #f(-500 0 0))
(send *robot* :rarm :move-end-pos #f(-500 0 0))

(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

#| wordkbench init
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(412 100 1300) :rpy (float-vector -3.14 1.57 1.57))) :translation-axis t :rotation-axis t)|#

#| check KXR position
(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(345 -246.5 1330) :rpy (float-vector 0.0 1.57 -1.57))) :translation-axis t :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(345 -84.8 1330) :rpy (float-vector 1.57 1.57 -3.14))) :translation-axis t :rotation-axis t)
(send *robot* :larm :move-end-pos #f(-50 0 0))
(send *robot* :rarm :move-end-pos #f(-50 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation) |#