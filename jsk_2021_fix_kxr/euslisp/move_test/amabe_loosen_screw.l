#!/usr/bin/env roseus

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(434 -14.541 1181.738) :rpy (float-vector 0.0 1.57 3.14))) :rotation-axis t)

(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(434 -14.541 1166.738) :rpy (float-vector 0.0 1.57 3.14))) :rotation-axis t)

(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(setq tem_d (- (send *robot* :rarm_joint7 :joint-angle) 180))
(send *robot* :rarm_joint7 :joint-angle tem_d))
 
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(434 -14.541 1181.738) :rpy (float-vector 0.0 1.57 0))) :rotation-axis t)

(send *ri* :angle-vector (send *robot* :angle-vector) 1500)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(434 -14.541 1181.738) :rpy (float-vector 0.0 1.57 3.14))) :rotation-axis t)

(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(434 -14.541 1166.738) :rpy (float-vector 0.0 1.57 3.14))) :rotation-axis t)

(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(setq tem_d (- (send *robot* :rarm_joint7 :joint-angle) 180))
(send *robot* :rarm_joint7 :joint-angle tem_d))
 
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(434 -14.541 1181.738) :rpy (float-vector 0.0 1.57 0))) :rotation-axis t)

(send *ri* :angle-vector (send *robot* :angle-vector) 1500)
(send *ri* :wait-interpolation)
#|
(send *ri* :start-grasp :rarm)
(unix::sleep 2)
(send *ri* :stop-grasp :rarm)
(send *ri* :start-grasp :rarm)
(unix::sleep 1)
(send *ri* :start-grasp :rarm)
(unix::sleep 1)
(send *ri* :start-grasp :rarm)
(unix::sleep 1)
|#
