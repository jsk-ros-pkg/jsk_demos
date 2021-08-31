#!/usr/bin/env roseus

(require "package://fetcheus/fetch-utils.l")
(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_maps/src/eng2-scene.l")

(unless (boundp '*scene*) (setq *scene* (make-eng2-7f-scene)))
(unless (boundp '*ri*) (fetch-init)) ;; fetch-init instantiate *fetch* and *ri* ;;

(objects (list *fetch* *scene*))
(send *ri* :objects (send *scene* :objects))
;;
;; 14.irteusgl$ (send *ri* :state :worldcoords)
;; #<cascaded-coords #X559a0f08b1e0 world  1898.58 9385.948 30000.0 / 1.609 2.492e-17 -3.942e-15>
;; 15.irteusgl$ (send *ri* :state :worldcoords)
;; #<cascaded-coords #X559a0f13b5c0 world  1895.018 9378.567 30000.0 / 0.042 1.533e-15 -2.347e-15>

;; (send *fetch* :move-to *trash-can-front* :world)

(defun view-robot ()
  (send *irtviewer :look-all
        (send (geo::make-bounding-box
               (flatten (send-all (send *fetch* :bodies) :vertices))) :grow 2.0)))

(setq *trash-can-front* (make-cascoords :pos #f(1895.0 9378.0 0.0) :rpy (float-vector 0 0 0)))
(send *fetch* :move-to *trash-can-front* :world)
(send *ri* :move-to *trash-can-front*)
