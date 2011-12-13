(load "ros-callback-manager.l")
;;(load "add-msg-pgsql.l")

;; posedetectiondb -> tf client class
;;(load "package://euslib/jsk/kalmanlib.l")
(ros::roseus-add-msgs "posedetection_msgs")
;; get vector in vlist , that is least median distance
(defun lmeds-vector
  (vlist)
  (find-extream vlist
		#'(lambda(p)(let ((diffs (mapcar #'(lambda(p1)(distance p p1)) vlist)))
			      (elt (sort diffs #'<) (/ (length diffs) 2))))
		#'<=))
(defun lmeds-pose
  (pose-list)
  (make-coords :pos (lmeds-vector (send-all pose-list :pos))
	       :rot (quaternion2matrix (lmeds-vector (mapcar #'matrix2quaternion (send-all pose-list :rot))))))

(defclass posedetectiondb-client
  :super ros::ros-callback-object
  :slots (object time-stamp poselist frame-id fixed-frame))
(defmethod posedetectiondb-client
  (:init
   (obj &key (topic "ObjectDetection") ((:object-frame _obj) nil)
	((:fixed-frame _fixed) "/base_footprint"))
   (setq object obj frame-id _obj fixed-frame _fixed)
   (setq time-stamp (ros::time 0))
   (defvar *tfl* (instance ros::transform-listener :init))
   (defvar *tfb* (instance ros::transform-broadcaster :init))
   (send-super :init topic posedetection_msgs::ObjectDetection :detection-cb))
  ;; filter detection pose, and move obj to the estimated coords
  (:update-coords
   (coords &optional (relative :local))
   (when (< 5 (send (ros::time- (ros::time-now) time-stamp) :to-sec))
     (setq poselist nil))
   (setq time-stamp (ros::time-now))
   (when (not (equal coords (car poselist)))
     (push (send coords :copy-worldcoords) poselist)
     (when (< 20 (length poselist))
       (nreverse poselist)
       (pop poselist)
       (nreverse poselist)))
;   (send object :move-to (lmeds-pose poselist) relative))
   (send object :move-to coords relative))
  (:detection-cb
   (msg)
   (dolist (object-msg (send msg :objects))
     (when (equal (string-left-trim "/" (send object-msg :type))
		  (string-left-trim "/" (or frame-id (send object :name))))
       ;; broadcast transform
       ;; (send msg :header :stamp (ros::time 0))
       (let* ((pose (instance geometry_msgs::PoseStamped :init
			      :header (send msg :header)
			      :pose (send object-msg :pose)))
	      (coords (ros::tf-pose->coords (send pose :pose)))
	      (objcoords (send *tfl* :transform-pose
			       fixed-frame pose)))
	 (ros::ros-info "posedetection-cb type:~A, coords~A" (send object-msg :type) coords)
	 (when objcoords
	   (send self :update-coords objcoords :world)
	   (putprop object (send msg :header :stamp) :stamp)
	   (send *tfb* :send-transform object
		 fixed-frame (or frame-id (send object :name))
		 (send msg :header :stamp)))
	 ))))
  )

(defun init-posedetectiondb-client (objects &key topic)
  (let (clients)
    (dolist (obj objects)
	  (push
	   (if topic (instance posedetectiondb-client :init obj :topic topic)
	     (instance posedetectiondb-client :init obj))
	   clients))
    (ros::ros-info "initialize posedetection db client with ~A~%" (send-all objects :name))
    clients))
