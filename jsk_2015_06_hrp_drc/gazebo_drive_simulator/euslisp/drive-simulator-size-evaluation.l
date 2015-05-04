#!/usr/bin/env roseus

(load "package://gazebo_drive_simulator/euslisp/drive-simulator-environment-ranger-xp900-size.l")

(setq simulator (instance drive-simulator-ranger-xp900-size :init))
(setq car (with-open-file (f (format nil "~A/models/polaris-xp900.l" (ros::rospack-find "hrpsys_gazebo_atlas"))) (read f)))


(send car :rotate (deg2rad -90) :z)
(send car :translate (float-vector -70 -330 0) :world)

(objects (list simulator car))
