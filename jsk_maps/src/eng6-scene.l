(load "package://jsk_maps/src/building-model.l")
(when (find :ros *features*)
  (require :db-building-model "package://jsk_maps/src/db-building-model.l"))

(load "models/elevator_call_panel-object.l")
(load "models/elevator_inside_panel-object.l")

(defun make-eng6-b1f-scene()
  (let (map scene)
    (setq map
	  (make-floor-plane (ros::resolve-ros-path "package://jsk_maps/raw_maps/eng6-b1f-0.05.pgm") 50 -43500 -42100 :name "/eng6/b1f/floor-plane"))
    (setq scene
	  (instance nav-scene-model :init :name "/eng6/b1f"
		    :objects (list map)))
    (send scene :map map)
    scene ))

(defun make-eng6-1f-scene()
  (let (map scene)
    (setq map
	  (make-floor-plane (ros::resolve-ros-path "package://jsk_maps/raw_maps/eng6-1f-0.05.pgm") 50 -43500 -42100 :name "/eng6/1f/floor-plane"))
    (setq scene
	  (instance nav-scene-model :init :name "/eng6/1f"
		    :objects (list map)))
    (send scene :map map)
    scene ))

(defun make-eng6-2f-scene()
  (let (map scene)
    (setq map
	  (make-floor-plane (ros::resolve-ros-path "package://jsk_maps/raw_maps/eng6-2f-0.05.pgm") 50 -43500 -42100 :name "/eng6/2f/floor-plane"))
    (setq scene
	  (instance nav-scene-model :init :name "/eng6/2f"
		    :objects (list map)))
    (send scene :map map)
    scene ))

(defun make-eng6-3f-scene()
  (let (map scene)
    (setq map
	  (make-floor-plane (ros::resolve-ros-path "package://jsk_maps/raw_maps/eng6-3f-0.05.pgm") 50 -43500 -42100 :name "/eng6/3f/floor-plane"))
    (setq scene
	  (instance nav-scene-model :init :name "/eng6/3f"
		    :objects (list map)))
    (send scene :map map)
    scene ))

(defun make-eng6-4f-scene()
  (let (map scene)
    (setq map
	  (make-floor-plane (ros::resolve-ros-path "package://jsk_maps/raw_maps/eng6-4f-0.05.pgm") 50 -43500 -42100 :name "/eng6/4f/floor-plane"))
    (setq scene
	  (instance nav-scene-model :init :name "/eng6/4f"
		    :objects (list map)))
    (send scene :map map)
    scene ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
(defun make-eng6-scene ()
  (let* (scene maps)
    (setq maps (list (make-eng6-b1f-scene)
		     (make-eng6-1f-scene)
		     (make-eng6-2f-scene)
		     (make-eng6-3f-scene)
		     (make-eng6-4f-scene)))

    ;; this is not true
    (dotimes (i (length maps))
      (send (elt maps i) :translate (float-vector 0 0 (* i 10000))))


    (setq scene
          (instance
           (if (boundp 'db-building-model) db-building-model building-model)
           :init
           :name "/eng6"
           :objects maps
           ))

    ;; visualize region
    (send-all (send-all maps :map) :set-color #f(0 1 0 0.3))

    scene))
