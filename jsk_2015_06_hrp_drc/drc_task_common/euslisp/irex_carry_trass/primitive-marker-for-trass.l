(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_rviz_plugins")

(load "package://drc_task_common/euslisp/primitive-marker-util.l")

(ros::roseus "koyama_primitive_marker")
(ros::roseus-add-msgs "posedetection_msgs")

(defun sub-object-pf (msg)
  (setq *pf-msg* msg)
  )
(ros::subscribe "/ObjectDetectionTrass"  posedetection_msgs::ObjectDetection #'sub-object-pf)

(defun wait-pfilter-obj (&key (wait 4.0)) ;; sec
  (setq *pf-msg* nil)
  (ros::spin-once)
  (dotimes (i (round (/ 4.0 0.2)))
    (if *pf-msg* (return))
    (ros::spin-once)
    (unix::usleep (* 1000 200)))
  (when *pf-msg*
    (let ((obj (car (send *pf-msg* :objects)))
          cds)
      (setq cds (ros::tf-pose->coords (send obj :pose)))
      (setf (get cds :reliability) (send obj :reliability))
      cds)))

(defun set-trass-coords-by-pf (&key (wait) (reset))
  (when reset
    (ros::service-call "/pfilter/reset_with_guess" (instance std_srvs::EmptyRequest :init))
    (wait-pfilter-obj :wait wait)
    (wait-pfilter-obj :wait wait))
  (ros::service-call "/transformable_interactive_server/set_color"
                     (instance jsk_interactive_marker::SetTransformableMarkerColorRequest :init :target_name "trass" :color (instance std_msgs::colorRGBA :init :r 1 :g 0 :b 0 :a 1)))
  (let ((cds (wait-pfilter-obj :wait wait)))
    (when cds
      ;;(send *trass* :reset-coords)
      ;;(send *trass* :transform cds)
      (send cds :translate (float-vector (/ (elt (send *trass* :size) 0) 2) 0
                                         (/ (elt (send *trass* :size) 2) 2)))
      (set-primitive-marker-pose cds "ground" :name "trass")
      cds
      )))
(defun set-trass-coords-by-pf-10  (&key (wait) (reset))
  (let ((eps 1000) cds1 cds2 (ii 0))
    (print eps)
    (setq cds1 (set-trass-coords-by-pf))
    (while (> eps 5)
      (format t "detecting:~A~%" ii)
      (setq cds2 (set-trass-coords-by-pf))
      (setq eps (norm (v- (send cds2 :worldpos)
                          (send cds1 :worldpos))))
      (setq cds1 (send cds2 :copy-worldcoords))
      (setq ii (1+ ii))
      ;;(when (> ii 20) (return nil))
      (when (> ii 10) (return nil))
      ;;(unix::sleep 1)
      )
    (unless (y-or-n-p "recognized?")
      (set-trass-coords-by-pf-10))
    ))

(defun sub-object (msg)
  (setq *msg* msg)
  )


(ros::subscribe "/ObjectDetectionTrassPose"  geometry_msgs::PoseStamped #'sub-object)
(defun set-pose-10 ()
  (block
   :set-pose-loop
   (dotimes (i 10)
     (if (set-pose) (return-from :set-pose-loop nil))
     (unix::usleep 200000)
     )
   (ros::ros-info "recog failed 10 times")
   )
  )

(defun make-interactive-marker (&key (name "trass") (x 265) (y 1750) (z 265) (offset (make-coords)))
  (let (trans coords)
    (ros::spin-once)
    (if (or (not *msg*) (not (send *msg* :pose)))
        (progn
          (if (not *msg*)
              (ros::ros-info "msg not published")
            (ros::ros-info "recognized result is empty")
            )
          ;;(return-from set-pose nil)
          ))
    (setq coords (make-coords :pos (float-vector 300 0 (/ z 2))));;tekito-
    (when *msg*
      (setq trans (send *tf* :lookup-transform "ground"
                        (send *msg* :header :frame_id) (send *msg* :header :stamp)))
      (unless trans
        (setq trans (send *tf* :lookup-transform "ground"
                          (send *msg* :header :frame_id) (ros::time 0))))
      (if trans (setq coords (send trans :transform (ros::tf-pose->coords (send *msg* :pose)))))
      (send coords :transform offset))

    (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*MESH_RESOURCE* :name name :description "" :resource "package://drc_task_common/euslisp/irex_carry_trass/trass.dae" :use-material t)
    ;; (set-primitive-marker-dimensions
    ;;  :x x :y y :z z :name name
    ;;  )

    (set-primitive-marker-pose
     coords "ground" :name name
     )
    (ros::ros-info "interactive maker has made!")
    )
  t
  )
(setq *msg* nil)
(setq *tf* (instance ros::transform-listener :init))

