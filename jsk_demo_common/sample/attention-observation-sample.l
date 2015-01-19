#!/usr/bin/env roseus

(ros::roseus "attention_observation_test")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :attention-observation "package://jsk_demo_common/euslisp/attention-observation.l")

(make-attention-action pick (&rest args)
 (send *pr2* :rarm :inverse-kinematics
       (make-coords :pos #f(600 0 700))
       :rotation-axis t
       :look-at-target t)
 (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
 (send *ri* :wait-interpolation)
 (send *ri* :stop-grasp :rarm :wait t)
 (send *pr2* :rarm :move-end-pos #f(100 0 0))
 (send *ri* :angle-vector (send *pr2* :angle-vector))
 (send *ri* :wait-interpolation)
 (send *ri* :start-grasp :rarm)
 (send *ri* :wait-interpolation)
 (send *pr2* :rarm :move-end-pos #f(-100 0 0))
 (send *ri* :angle-vector (send *pr2* :angle-vector))
 (send *ri* :wait-interpolation)
 t)

(make-attention-action turn-left (&rest args)
 (send *ri* :go-pos-unsafe 0 0 90)
 (send *ri* :wait-interpolation)
 t)

(make-attention-action place (&rest args)
 (send *pr2* :rarm :move-end-pos #f(100 0 0))
 (send *ri* :angle-vector (send *pr2* :angle-vector))
 (send *ri* :wait-interpolation)
 (send *ri* :stop-grasp :rarm :wait t)
 (send *ri* :wait-interpolation)
 (send *pr2* :rarm :move-end-pos #f(-100 0 0))
 (send *ri* :angle-vector (send *pr2* :angle-vector))
 (send *ri* :wait-interpolation)
 t)

(defun test ()
  (pr2-init)
  (pr2-reset-pose)
  (pick)
  (turn-left)
  (place))
