(load "package://fetcheus/fetch-interface.l")
(require "models/arrow-object.l")
(fetch-init)

(setq *co-handle* (arrow))
(send *co-handle* :newcoords (make-coords :pos #f(800 300 1000) :rpy #f(0.0 0.0 1.54)))

(setq *co-endeffector* (arrow))
(send *co-endeffector* :newcoords (send (send *fetch* :rarm :end-coords) :copy-worldcoords))
(objects (list *fetch* *co-endeffector* *co-handle*))

(setq *co-ik-target* (arrow))
(send *co-ik-target* :newcoords (send *co-handle* :copy-worldcoords))
(send *co-ik-target* :translate #f(-80 0 0) :local)

;; solve ik
(send *fetch* :rarm :inverse-kinematics *co-ik-target*
        :rotation-axis t :check-collision t :use-torso t)

;; command to target pose
(send *ri* :angle-vector (send *fetch* :angle-vector) 1000) ;; command 1
(send *ri* :wait-interpolation)

;; move forward
(send *fetch* :rarm :move-end-pos #f(100 0 0) :local)
(send *ri* :angle-vector (send *fetch* :angle-vector) 1000) ;; command 2
(send *ri* :wait-interpolation)

;; grasp
(send *ri* :start-grasp :effort 100)

;; lift
(send *fetch* :rarm :move-end-pos #f(0 0 100) :world)
(send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
(send *ri* :wait-interpolation)
