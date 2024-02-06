#!/usr/bin/env roseus

(defun panda-return-rarm ()
  (ros::ros-info "Returning to neutral position")

  (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(300.472 -75.155 1210.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
  (send *robot* :rarm :move-end-pos #f(-100 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(300.472 -75.155 1210.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :rarm)
  (unix:sleep 1)

  (larm-release-rarm-elbow-p-servo-from-below)

  (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(297.605 169.619 1214.223) :rpy (float-vector -1.224 1.5 0.362))) :translation-axis t :rotation-axis t)
  (send *robot* :larm :move-end-pos #f(-100 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(297.605 169.619 1214.223) :rpy (float-vector -1.224 1.5 0.362))) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm)
  (unix:sleep 1)

  (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(308.472 -259.081 1220.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
  (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(305.157 -14.307 1220.274) :rpy (float-vector -1.224 1.5 0.362))) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (ros::ros-info "End returning")
  )
