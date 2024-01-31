#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(unless (boundp '*ri*)
  (pr2-init))
(objects (list *pr2*))

(send *pr2* :init-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector))
(send *ri* :wait-interpolation)

(setq left-hand-cds (make-coords :pos #f(500 0 800) :rpy (float-vector -pi/2 0 0)))
(setq right-hand-cds (make-coords :pos #f(500 0 1000) :rpy (float-vector pi/2 0 0)))
(send *ri* :angle-vector-sequence (list (send *pr2* :rarm :inverse-kinematics right-hand-cds :rotation-axis t) (send *pr2* :rarm :move-end-pos #f(50 0 0))) (list 5000 3000))
(send *ri* :wait-interpolation)
(send *ri* :start-grasp :rarm)
(send *pr2* :rarm :inverse-kinematics right-hand-cds :rotation-axis t)
(send *ri* :angle-vector-sequence (list (send *pr2* :larm :inverse-kinematics left-hand-cds :rotation-axis t) (send *pr2* :larm :move-end-pos #f(50 0 0))) (list 5000 3000))
(send *ri* :wait-interpolation)
(send *ri* :start-grasp :larm)
