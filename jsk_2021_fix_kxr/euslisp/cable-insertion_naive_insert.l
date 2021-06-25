#!/usr/bin/env roseus

(send *robot* :rarm :move-end-pos #f(6 0 0))

(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :rarm :width 0.004)

(send *robot* :rarm :move-end-pos #f(-6 0 0))

(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

