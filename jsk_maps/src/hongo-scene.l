(require "package://jsk_maps/src/building-model.l")

(require "models/elevator_call_panel_eng2-object.l")
(require "models/elevator_inside_panel_eng2-object.l")

(require "package://jsk_maps/src/eng2-scene.l")
(require "package://jsk_maps/src/eng8-scene.l")


(defclass campus-model
  :super scene-model
  :slots ())
  )

(defun make-hongo-campus ()
  (let* (scene map bld-eng2 bld-eng8)
    ;; 600m x 600m -> 600 pixel -> resolution 1000 [mm]
    ;; see jsk_maps/raw_maps/hongo-campus-1.00.yaml for parameters
    (setq map (make-floor-plane (ros::resolve-ros-path "package://jsk_maps/raw_maps/hongo-1.00.pgm") 1000 -300000 -300000 :name "/hongo"))

    (setq bld-eng2 (make-eng2-scene)
          bld-eng8 (make-eng8-scene))
    (send bld-eng2 :locate #f( -10000 200000 0) :world)
    (send bld-eng2 :orient (deg2rad -80) :z :world)
    (send bld-eng8 :locate #f(-140000 220000 0) :world)
    (send bld-eng8 :orient (deg2rad 100) :z :world)

    (setq scene
          (instance
           campus-model :init :name "/hongo"
           :objects
           (list map bld-eng2 bld-eng8)
           ))
    (send scene :add-spot (make-cascoords :pos #f(-220000 30000 0) :name "/hongo/main-gate"))
    (send scene :add-spot (make-cascoords :pos #f(-220000 120000 0) :name "/hongo/starbucks"))
    scene))

;; example
#|
(load "package://jsk_maps/src/hongo-scene.l")
(setq *scene* (make-hongo-campus))
(objects (list *scene*))
|#
