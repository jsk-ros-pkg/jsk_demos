;; door model
(load "models/room73b2-door-left-object.l")
(load "models/room73b2-door-right-object.l")

;; wall model
(load "models/room73b2-external-wall-1-object.l")
(load "models/room73b2-external-wall-0-object.l")

(defclass door_with_wall-scene
    :super scene-model
    :slots ())
(defmethod door_with_wall-scene
    (:init
        (&rest args &key (name "door_with_wall") &allow-other-keys)
        (let ()
            (send-super*
                :init :name name :objects
                (list
                (send (room73b2-door-left) :transform (make-coords :pos (float-vector 0.0 405.0 0.0) :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))
                (send (room73b2-door-right) :transform (make-coords :pos (float-vector 0.0 -405.0 0.0) :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))
                (send (room73b2-external-wall-1) :transform (make-coords :pos (float-vector -70.0 -2125.0 1350.0) :rot #2f((2.220446e-16 -1.0 0.0) (1.0 2.220446e-16 0.0) (0.0 0.0 1.0))))
                (send (room73b2-external-wall-0) :transform (make-coords :pos (float-vector -70.0 2567.0 1350.0) :rot #2f((2.220446e-16 -1.0 0.0) (1.0 2.220446e-16 0.0) (0.0 0.0 1.0))))
                )
                args)
        ))
)

(defun door_with_wall (&rest args) (setq *door_with_wall* (instance* door_with_wall-scene :init args)))