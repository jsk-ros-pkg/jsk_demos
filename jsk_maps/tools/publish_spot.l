#!/usr/bin/env roseus

(load "package://jsk_maps/src/eng2-scene.l")
(setq *scene* (make-eng2-scene))

(defun make-color-list (color-num &key (hue-max 300.0))
  (do ((ret nil (push
		 (scale (/ 1.0 255.0) (hvs2rgb (* (/ hue-max color-num) x) 1.0 1.0
					       (float-vector 0.0 0.0 0.0)))
		 ret))
       (x 0 (incf x)))
      ((> x color-num) ret)
      ))

(defun make-pin-body (&key (coords (make-cascoords)) (height 1800) (radius 100))
  (let (b)
    (setq b (body+ ;;(send (make-cone #f(0 0 1800) 30 :pos #f(0 0 1800)) :rotate pi :x)
		   (make-cube (/ radius 3) (/ radius 3) height :pos (float-vector 0 0 (/ height 2)))
		   (send (make-cylinder radius radius :segments 9 :pos (float-vector 50 0 height)) :rotate -pi/2 :y)))
    (send b :name (send coords :name))
    (send b :transform coords :world)
    b))

(ros::roseus "publish_jskmap_spots")
(defvar *pub-spots-topic* "/spots_marker_array")
(defvar *tf-listener* (instance ros::transform-listener :init))

(defun pub-spot ()
  (let ((msg-array (instance visualization_msgs::MarkerArray :init))
        (header (instance std_msgs::header :init :stamp (ros::time-now) :frame_id "/map"))
        (colors (make-color-list (length (send *scene* :spots))))
        (id 0) c n msgs current-map-id current-map-coords)
    (setq current-map-id (string-right-trim "_" (string-right-trim "tf" *map-selected*))) ;; remove _tf
    (when (send  *tf-listener* :wait-for-transform "/world" current-map-id (ros::time-now) 1)
      (setq current-map-coords (send *tf-listener* :lookup-transform "/world" current-map-id (ros::time-now)))) ;; resolve /world /eng/7f
    (setq current-map-id (format nil "~A/" current-map-id))     ;; add /
    (ros::ros-info "current-map ~A ~A" current-map-id current-map-coords)
    (dolist (c (send *scene* :spots))
      (when (and current-map-coords (substringp current-map-id (send c :name)))
        ;(ros::ros-info "publish spot~A" (send c :name))
        (push (object->marker-msg (make-pin-body :coords (send (send current-map-coords :transformation c) :worldcoords) :height 500 :radius 61)
                                  header
                                  :color (elt colors id)
                                  :ns "pin"
                                  :lifetime 15
                                  :id id)
              msgs)
        (push (text->marker-msg (subseq (send c :name) (length current-map-id))
                                (make-coords :pos (v+ (send (send current-map-coords :transformation c) :worldpos) #f(0 0 700)))
                                header
                                :ns "text"
                                :scale 100
                                :lifetime 15
                                :id id)
              msgs)
        (incf id)
        ))
    (send msg-array :markers msgs)
    (ros::publish *pub-spots-topic* msg-array)
    ))

(setq *map-selected* "")
(defun map-selected-cb (msg) (setq *map-selected* (send msg :data)))
(ros::subscribe "/map_tf_mux/selected" std_msgs::String #'map-selected-cb)

(ros::advertise *pub-spots-topic* visualization_msgs::MarkerArray 5)

(dotimes (i 3) (pub-spot))
(ros::rate 0.1)
(do-until-key
 (pub-spot)
 (ros::spin-once)
 (ros::sleep))
(ros::unadvertise *pub-spots-topic*)


