#!/usr/bin/env roseus

(ros::roseus "qr-data-manager")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_rviz_plugins")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(load "package://drc_task_common/euslisp/primitive-marker-util.l")

(setq *tf* (instance ros::transform-listener :init))
(setq *current-pose-stamped* nil)
(setq *pose-list-raw* nil)
(setq *answer-list* nil)
(setq *base-frame-id* "/map")
(setq detected nil)
(setq *offset-trans* (make-coords :pos (float-vector 540 -30 0) :rpy (list -pi/2 0 0)))
(defun qr-data-manager-init
  ()
  ;; regist subscriber
  (if (ros::get-param "reload" nil)
      (progn (load "/home/leus/.ros/saved_data.l")
             (ros::ros-info "reload"))
    (progn
      (ros::set-param "reload" t)
      (save-cb)
      )
    )

  (ros::subscribe "/bounding_box_marker/selected_box" jsk_recognition_msgs::BoundingBox #'b-box-cb 1)
  (ros::subscribe "/append_with_name" std_msgs::String #'append-marker-cb 1)
  ;;(ros::subscribe "/transformable_interactive_server/pose" geometry_msgs::PoseStamped #'pose-stamped-cb 1)
  (ros::subscribe "/barcode" std_msgs::String #'qr-cb 1)
  (ros::advertise "/qr_state_text" jsk_rviz_plugins::OverlayText 1)
  (ros::advertise-service "/reload_old_data" std_srvs::Empty #'reload-cb)
  )


(defun b-box-cb
  (msg)
  ;; append list with pose
  (let* (
      (raw-pose (ros::tf-pose->coords (send msg :pose)))
      (trans (send *tf* :lookup-transform *base-frame-id*
                         (send msg :header :frame_id) (ros::time 0)))
      (box-center (send trans :transform raw-pose))
      (box-buttom (send box-center :translate (float-vector 0 0 (* (send msg :dimensions :z) 500)) :local))
      (box-buttom-trans (send *offset-trans* :transformation box-buttom))
      )
    (ros::ros-info (format nil "newpose ~A" box-buttom-trans))
    (setq *pose-list-raw* (append (list (send box-buttom-trans :pos)) *pose-list-raw*)))
    (save-cb)
  )

(defun append-marker-cb
  (msg)
  ;; append list with right click
  (ros::ros-info "new marker appended")
  (let* (
        (box-center (get-primitive-marker-pose :name (send msg :data) :frame-id *base-frame-id*))
        (box-buttom (send box-center :translate (float-vector 0 0 (* 0.5 (elt (get-primitive-marker-dimensions :name (send msg :data)) 2)))))
        (box-buttom-trans (send *offset-trans* :transformation box-buttom))
        )
    (ros::ros-info (format nil "newpose ~A" box-buttom-trans))
    (setq *pose-list-raw* (append (list (send box-buttom-trans :pos)) *pose-list-raw*)))
    (save-cb)
  )

;; (defun pose-stamped-cb
;;   (msg)
;;   ;; store current pose
;;   (setq *current-pose-stamped* msg)
;;   (save-cb)
;;   )

(defun qr-cb
  (msg)
;; if new qr
  (ros::ros-info "qr detected")
  (when (find-if #'(lambda (x) (equal (elt x 0) (send msg :data))) *answer-list*)
    (ros::ros-info "known qr! ~A" (send msg :data))
    (return-from qr-cb)
    )
  (ros::ros-info "new qr detected!")
;; if there are any near point
  ;; (when t;;(not *current-pose-stamped*)
  ;;   (let ((coords (get-primitive-marker-pose :frame-id *base-frame-id*)))
  ;;     (setq *current-pose-stamped* (instance geometry_msgs::PoseStamped :init :pose (ros::coords->tf-pose coords) :header (instance std_msgs::Header :init :frame_id *base-frame-id*)))
  ;;     )
  ;;   )
  (let* (
         (box-center (get-primitive-marker-pose :frame-id *base-frame-id*))
         (box-buttom (send box-center :translate (float-vector 0 0 (* 0.5 (elt (get-primitive-marker-dimensions :name (send msg :data)) 2)))))
         (box-buttom-trans (send *offset-trans* :transformation box-buttom))
         (current-pos (send box-buttom-trans :pos))
         )
    (setq detected nil)
    (unix::system (format nil "gnome-screenshot -d 3 -f ~/.ros/test~a.png &" (length *answer-list*)))
    (block finding-near
     (dotimes (i (length *pose-list-raw*))
       (when (< (norm (v- (elt *pose-list-raw* i) current-pos)) 500)
         (setq *answer-list* (append (list (list (send msg :data) (elt *pose-list-raw* i))) *answer-list*))
         (ros::ros-info "seen before")
         (setq detected t)
         (return-from finding-near)
         )
       )
     )
    (if (not detected) (progn (ros::ros-info "new pose") (setq *answer-list* (append (list (list (send msg :data) current-pos)) *answer-list*))))
    (save-cb)
    (publish-overlay)
    ;;(ros::ros-info "hoge") (return-from qr-cb)
    )
  )

(defun publish-overlay
  ()
  (let (
      (text-msg (instance jsk_rviz_plugins::OverlayText :init))
      )
    (send text-msg :text (format nil "~aMarker: [~a] Pos: ~a"
                                 (if detected "Detected" "Unseen")
                                 (elt (elt *answer-list* 0) 0)
                                 (elt (elt *answer-list* 0) 1)
                                 ))
    (send text-msg :top 190)
    (send text-msg :left 10)
    (send text-msg :width 1500)
    (send text-msg :height 50)
    (send text-msg :bg_color :r 0.9)
    (send text-msg :bg_color :b 0.9)
    (send text-msg :bg_color :g 0.9)
    (send text-msg :bg_color :a 0.1)
    (send text-msg :fg_color :r (if detected 0.2 1.0))
    (send text-msg :fg_color :g 0.7)
    (send text-msg :fg_color :b 0.7)
    (send text-msg :fg_color :a 1)
    (send text-msg :line_width 1)
    (send text-msg :text_size 28)
    (ros::publish "/qr_state_text" text-msg)
    )
  )

(defun write-to-csv ()
  (with-open-file
   (f "/home/leus/.ros/qr_position.csv" :direction :output :permission #o744)
   (dolist (qrlst *answer-list* nil)
     (format f "~A , ~A, ~A, ~A~%"
             (car qrlst)
             (aref (cadr qrlst) 0)
             (aref (cadr qrlst) 1)
             (aref (cadr qrlst) 2))
   ))
  (unix::system (format nil "mkdir -p /home/leus/tmp"))
  (unix::system (format nil "cp /home/leus/.ros/qr_position.csv ~/tmp/qr_position_~A_~A_~A_~A.csv"
                        (aref (unix:localtime) 4)
                        (aref (unix:localtime) 3)
                        (aref (unix:localtime) 2)
                        (aref (unix:localtime) 1)))
  )


(defun save-cb ()
  (dump-loadable-structure "/home/leus/.ros/saved_data.l" *current-pose-stamped* *pose-list-raw* *answer-list*)
  (write-to-csv)
  (unix::system (format nil "mkdir -p /home/leus/tmp"))
  (unix::system (format nil "cp /home/leus/.ros/saved_data.l ~/tmp/saved_data_~A_~A_~A_~A.l"
                        (aref (unix:localtime) 4)
                        (aref (unix:localtime) 3)
                        (aref (unix:localtime) 2)
                        (aref (unix:localtime) 1)))
  )

(defun reload-cb (req)
  (load "/home/leus/.ros/saved_data.l")
  (send req :response)
  )
(qr-data-manager-init)
(ros::rate 10)
(while t
  (ros::sleep)
  (ros::spin-once)
  )
