(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "posedetection_msgs")

(load "package://elevator_move_base_pr2/src/ros-callback-manager.l")


;; get vector in vlist , that is least median distance
(defun lmeds-vector
  (vlist)
  (find-extream
   vlist
   #'(lambda (p) (let ((diffs (mapcar #'(lambda (p1) (distance p p1)) vlist)))
                   (elt (sort diffs #'<) (/ (length diffs) 2))))
   #'<=))

(defun lmeds-pose
  (pose-list)
  (make-coords :pos (lmeds-vector (send-all pose-list :pos))
               :rot (quaternion2matrix
                     (lmeds-vector (mapcar #'matrix2quaternion
                                           (send-all pose-list :rot))))))


;; posedetectiondb -> tf client class
(defclass posedetectiondb-client
  :super ros-callback-object
  :slots (object time-stamp poselist frame-id fixed-frame filter-type tfl tfb))

(defmethod posedetectiondb-client
  (:init (obj
          &key (topic "ObjectDetection") ((:object-frame _obj) nil)
               ((:fixed-frame _fixed) "/base_footprint")
               ((:filter-type ft) nil))
   (setq object obj)
   (setq frame-id _obj)
   (setq fixed-frame _fixed)
   (setq filter-type ft)
   (setq time-stamp (ros::time 0))
   (setq tfl (instance ros::transform-listener :init))
   (setq tfb (instance ros::transform-broadcaster :init))
   (send-super :init topic posedetection_msgs::ObjectDetection :detection-cb))

  (:update-coords (coords &optional (relative :local))
   "Filter detection pose, and move obj to the estimated coords"
   (when (< 5 (send (ros::time- (ros::time-now) time-stamp) :to-sec))
     (setq poselist nil))
   (setq time-stamp (ros::time-now))
   (when (not (equal coords (car poselist)))
     (push (send coords :copy-worldcoords) poselist)
     (when (< 20 (length poselist))
       (nreverse poselist)
       (pop poselist)
       (nreverse poselist)))
   (case filter-type
     (:lmeads
      (send object :move-to (lmeds-pose poselist) relative))
     (t
      (send object :move-to coords relative))))

  (:detection-cb (msg)
   (dolist (object-msg (send msg :objects))
     (when (equal (string-left-trim "/" (send object-msg :type))
                  (string-left-trim "/" (or frame-id (send object :name))))
       ;; broadcast transform
       (send tfl :wait-for-transform
             fixed-frame (send msg :header :frame_id)
             (send msg :header :stamp) 1)
       (let* ((pose (instance geometry_msgs::PoseStamped :init
                              :header (send msg :header)
                              :pose (send object-msg :pose)))
              (coords (ros::tf-pose->coords (send pose :pose)))
              (objcoords (send tfl :transform-pose
                               fixed-frame pose)))
         (ros::ros-info "posedetection-cb type:~A, coords~A"
                        (send object-msg :type) coords)
         (when objcoords
           (send self :update-coords objcoords :world)
           (putprop object (send msg :header :stamp) :stamp)
           (send tfb :send-transform object
                 fixed-frame (or frame-id (send object :name))
                 (send msg :header :stamp)))
         ))))
  )


(defun init-posedetectiondb-client (objects &key topic)
  (let (clients)
    (dolist (obj objects)
      (push
       (if topic
           (instance posedetectiondb-client :init obj :topic topic)
         (instance posedetectiondb-client :init obj))
       clients))
    (ros::ros-info "initialize posedetection db client with ~A~%"
                   (send-all objects :name))
    clients))
