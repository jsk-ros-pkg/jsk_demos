(require "package://pr2eus/pr2-interface.l")
(require "package://pr2eus/speak.l")
(pr2-init)

(send *pr2* :reset-pose)

(defclass box-label-synchronizer
  :super exact-time-message-filter)

(defmethod box-label-synchronizer
  (:callback (box label)
    (print (list box label))
    (print (send-all (list box label) :header :stamp))
    ))

;; ;; test
;; (setq *box-label* (instance box-label-synchronizer :init
;; 			    (list (list "/docker/detic_segmentor/output/boxes" jsk_recognition_msgs::BoundingBoxArray)
;; 				  (list "/docker/detic_segmentor/detected_classes" jsk_recognition_msgs::LabelArray))))

(ros::load-ros-manifest "jsk_recognition_msgs")
(setq *target-box* (make-cube 100 100 100))
(objects (list *pr2* *target-box*))
(defun box-cb (msg)
  (ros::ros-info "received ~A boxes" (length (send msg :boxes)))
  (dolist (box (send msg :boxes))
    (when (member (send box :label) '(566 615 469 923)) ;; 566 -> hook ;615:knife 469:fork 923:scissors
      (ros::ros-info "hook is detected" )
      (send *ri* :speak-jp "ハンガー発見！")
      (setq *target-coords* (send (ros::tf-pose->coords (send box :pose)) :copy-worldcoords))
      (setq *target-dimensions* (send box :dimensions))
      (format t "coords ~A, dimension ~A~%" (send *target-coords* :worldcoords) (* (send *target-dimensions* :x) (send *target-dimensions* :y) (send *target-dimensions* :z)))
      (when (and (< (elt (send *target-coords* :worldpos) 2) 2000)
                 (> (elt (send *target-coords* :worldpos) 2) 0))
        (send *target-box* :move-to *target-coords* :world)
        (print "update target position")
        ))))

(ros::subscribe "/synchronized_detic_label_boxes" jsk_recognition_msgs::BoundingBoxArray #'box-cb)
;;(ros::subscribe "/kinect_head/depth_registered/boxes" jsk_recognition_msgs::BoundingBoxArray #'box-cb)
(do-until-key
 (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
 (send *irtviewer* :draw-objects)
 (x::window-main-one)
 (ros::spin-once)
 )
