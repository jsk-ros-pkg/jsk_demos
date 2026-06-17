;; Simple PR2 grasp utilities for a double-handled pot.
;; This file assumes pot_handle_perception.l and pot_handle_selection.l
;; have already been loaded.

(setq *double-handle-grasp-approach-offset-mm* 0.0)
(setq *left-handle-grasp-rpy* (list 0 pi/2 0))
(setq *right-handle-grasp-rpy* (list 0 (* 3 pi/2) 0))
(setq *double-handle-grasp-move-time* 3000)

(defun detection-center-mm (detection)
  (let ((center (pot-handle-detection-center detection)))
    (float-vector (* 1000.0 (elt center 0))
                  (* 1000.0 (elt center 1))
                  (* 1000.0 (elt center 2)))))

(defun make-handle-grasp-coords (handle grasp-rpy)
  (let ((pos (detection-center-mm handle)))
    (make-cascoords
     :pos (float-vector (elt pos 0)
                        (elt pos 1)
                        (elt pos 2))
     :rpy grasp-rpy)))

(defun move-left-arm-to-handle (left-handle)
  (let ((coords (make-handle-grasp-coords
                 left-handle *left-handle-grasp-rpy*)))
    (send *pr2* :larm :inverse-kinematics coords :rotation-axis :z)
    coords))

(defun move-right-arm-to-handle (right-handle)
  (let ((coords (make-handle-grasp-coords
                 right-handle *right-handle-grasp-rpy*)))
    (send *pr2* :rarm :inverse-kinematics coords :rotation-axis :z)
    coords))

(defun solve-double-handle-pot-grasp (selection)
  (let ((left-handle (selected-left-handle selection))
        (right-handle (selected-right-handle selection)))
    (when (and left-handle right-handle)
      (let ((left-coords (move-left-arm-to-handle left-handle))
            (right-coords (move-right-arm-to-handle right-handle)))
        (list left-coords right-coords)))))

(defun send-double-handle-pot-grasp-to-real ()
  (send *ri* :angle-vector
        (send *pr2* :angle-vector)
        *double-handle-grasp-move-time*)
  (send *ri* :wait-interpolation))

(defun grasp-double-handle-pot (selection &optional (send-to-real nil))
  (let ((coords (solve-double-handle-pot-grasp selection)))
    (when coords
      (when send-to-real
        (send-double-handle-pot-grasp-to-real)
        (send *ri* :start-grasp))
      coords)))
