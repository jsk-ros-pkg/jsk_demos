(ros::roseus "preemptive_task_execution_sample")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :preemptive-task-execution "package://jsk_demo_common/euslisp/preemptive-task-execution.l")

(setq pick-action
 (action pick
  (motion pick-pose
   (send *pr2* :rarm
         :inverse-kinematics
         (make-coords :pos #f(600 0 700))
         :rotation-axis t
         :look-at-target t)
   (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
   (send *ri* :wait-interpolation))
  (motion reach
   (send *pr2* :rarm :move-end-pos #f(100 0 0))
   (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
   (send *ri* :wait-interpolation))
  (motion grasp
   (send *ri* :start-grasp :rarm)
   (send *ri* :wait-interpolation))
  (motion reach-out
   (send *pr2* :rarm :move-end-pos #f(-100 0 0))
   (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
   (send *ri* :wait-interpolation))
))

(setq carry-action
 (action carry
  (motion move
   (send *ri* :go-pos-unsafe 0 0 90)
   (send *ri* :wait-interpolation))
))

(setq place-action
 (action place
  (motion reach
   (send *pr2* :rarm :move-end-pos #f(100 0 0))
   (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
   (send *ri* :wait-interpolation))
  (motion release
   (send *ri* :stop-grasp :rarm)
   (send *ri* :wait-interpolation))
  (motion reach-out
   (send *pr2* :rarm :move-end-pos #f(-100 0 0))
   (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
   (send *ri* :wait-interpolation))
))

(setq pick-and-place
 (task
  (list pick-action
        carry-action
        place-action)))

(setq smt (instance state-machine-inspector :init pick-and-place)) ;; for debug

(unless (boundp '*ri*)
  (pr2-init))
(execute-task pick-and-place)
