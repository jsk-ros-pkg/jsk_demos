#!/usr/bin/env roseus
(ros::roseus "pr2_kettle")

(load "package://pr2eus/pr2-interface.l")
(load "models/kettle-object.l")
(kettle)
(setq c (kettle))
(pr2)
(objects *pr2*)
(send c :locate  #f(400 0 700))
(send c :rotate (/ pi 4) :z)
(objects c)

(setq *c-coord* (send c :coords))
(setq *c-handle* (send c :handle)

(setq *ri* (instance pr2-interface :init))

;;(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)

(send *pr2* :inverse-kinematics
      (make-coords :pos #f(400 300 1600))
      :rotation-axis nil
      :move-target (send *pr2* :larm :end-coords)
      :link-list (send *pr2* :link-list
		       (send *pr2* :larm :end-coords :parent)
		       (send *pr2* :torso_lift_link_lk))
      :debug-view t)

;;(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)

(send *pr2* :inverse-kinematics
      (list rtarget
            (send (send *pr2* :larm :end-coords) :copy-worldcoords))
      :link-list (list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
                       (send *pr2* :link-list (send *pr2* :larm :end-coords :parent)))
      :move-target (list (send *pr2* :rarm :end-coords)
                         (send *pr2* :larm :end-coords))
      )
