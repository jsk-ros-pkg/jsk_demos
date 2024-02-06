(send *ri* :start-grasp :rarm)
(unix::sleep 1)

(send *ri* :stop-grasp :rarm)
(unix::sleep 1)

(send *ri* :start-grasp :rarm)
(unix::sleep 1)

(send *ri* :stop-grasp :rarm :width 0.04)
(unix::sleep 1)

(send *ri* :stop-grasp :rarm)
(unix::sleep 1)
