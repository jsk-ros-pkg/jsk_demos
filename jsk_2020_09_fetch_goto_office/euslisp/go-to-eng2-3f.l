(require :fetch-interface "package://fetcheus/fetch-interface.l")
(require :fetch-interface "package://jsk_maps/src/change-floor.l")
(fetch-init)
(ros::load-ros-manifest "speech_recognition_msgs")

(defun go-to-eng2-3f()
  ;; move to front of the elevator
  (send *ri* :move-to (make-coords :pos #f(1850 -30746 0) :rpy (float-vector -pi/2 0 0)) :frame-id "/map")
  ;; (push-elevator-button())
  ;; (confirm-opened-door())
  (send *ri* :go-pos 3.5 0 180)
  ;; (recognize-arrive-at-eng2-3f())
  (change-floor "/eng2/3f" :base-frame-name "/base_link")
  (send *ri* :clear-costmap)
  (send *ri* :go-pos 3.5 0 0)
  
  )