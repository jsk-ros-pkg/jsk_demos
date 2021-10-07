#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")

(dual_panda-init)

(objects (list *robot*))

(send *robot* :reset-pose)
(send *robot* :larm :move-end-pos #f(-500 0 0))
(send *robot* :rarm :move-end-pos #f(-500 0 0))

(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :rarm)
(send *ri* :stop-grasp :larm)
