#!/usr/bin/env roseus

(load "package://jsk_maps/src/eng2-scene.l")
(setq *scene* (make-eng2-scene))

;; copy from jskgeo.l
(defun hvs2rgb (h &optional i s ret)
  (when (or (listp h) (vectorp h))
    (setq ret s)
    (setq s (elt h 2) i (elt h 1) h (elt h 0)))
  (if (>= h 360) (setq h (mod (round h) 360)))
  (if (< h    0) (setq h (- 360 (mod (round (- h)) 360))))
  (setq s (* 255 s) i (* 255 i))
  (let* ((hi (floor (/ h 60.0)))
	 (f (- (/ h 60.0) hi))
	 (p1 (* i (- 1.0 (/ s 255.0))))
	 (p2 (* i (- 1.0 (* f (/ s 255.0)))))
	 (p3 (* i (- 1.0 (* (- 1 f) (/ s 255.0)))))
	 r g b)
    (case hi
     ((0 6) (setq r i  g p3 b p1))
     (1 (setq r p2 g i  b p1))
     (2 (setq r p1 g i  b p3))
     (3 (setq r p1 g p2 b i))
     (4 (setq r p3 g p1 b i))
     (5 (setq r i  g p1 b p2))
     (t (warn "hri2rgb error")))
    (cond
     (ret
      (setf (elt ret 0) r)
      (setf (elt ret 1) g)
      (setf (elt ret 2) b)
      ret)
     (t (list r g b)))))

(defun make-color-list (color-num &key (hue-max 300.0))
  (do ((ret nil (push
		 (scale (/ 1.0 255.0) (hvs2rgb (* (/ hue-max color-num) x) 1.0 1.0
					       (float-vector 0.0 0.0 0.0)))
		 ret))
       (x 0 (incf x)))
      ((> x color-num) ret)
      ))

(defun make-pin-body (&key (coords (make-cascoords)))
  (let (b)
    (setq b (body+ ;;(send (make-cone #f(0 0 1800) 30 :pos #f(0 0 1800)) :rotate pi :x)
		   (make-cube 30 30 1800 :pos #f(0 0 900))
		   (send (make-cylinder 100 100 :segments 9 :pos #f(50 0 1800)) :rotate -pi/2 :y)))
    (send b :name (send coords :name))
    (send b :transform coords :world)
    b))

(ros::roseus "publish_jskmap_spots")
(defvar *pub-spots-topic* "/spots_marker_array")

(defun pub-spot ()
  (let ((msg-array (instance visualization_msgs::MarkerArray :init))
	(header (instance std_msgs::header :init :stamp (ros::time-now) :frame_id "/map"))
	(colors (make-color-list (length (send *scene* :spots))))
	(id 0) c msgs)
    (dolist (c (send *scene* :spots))
      (ros::ros-info "publish spot~A" (send c :name))
      (push (object->marker-msg (make-pin-body :coords (send c :worldcoords))
				header
				:color (elt colors id)
				:ns "pin"
				:id id)
	    msgs)
      (push (text->marker-msg (send c :name)
			      (make-coords :pos (v+ (send c :worldpos) #f(0 0 2000)))
			      header
			      :ns "text"
			      :id id)
	    msgs)
      (incf id)
      )
    (send msg-array :markers msgs)
    (ros::publish *pub-spots-topic* msg-array)
    ))

(ros::advertise *pub-spots-topic* visualization_msgs::MarkerArray 5)
(do-until-key
 (ros::rate 0.1)
 (pub-spot)
 (ros::spin-once)
 (ros::sleep))
(ros::unadvertise *pub-spots-topic*)


