;; Simple target selection utilities for pot/handle detections.
;; Detection format is defined in pot_handle_perception.l:
;; (class-name center dimensions volume box-msg label-msg label-index)

(defun select-largest-pot (pots)
  (let ((best-pot nil)
        (best-volume nil))
    (dolist (pot pots)
      (let ((volume (pot-handle-detection-volume pot)))
        (when (or (null best-volume) (> volume best-volume))
          (setq best-pot pot)
          (setq best-volume volume))))
    best-pot))

(defun detection-distance (detection-a detection-b)
  (let* ((a (pot-handle-detection-center detection-a))
         (b (pot-handle-detection-center detection-b))
         (dx (- (elt a 0) (elt b 0)))
         (dy (- (elt a 1) (elt b 1))))
    (sqrt (+ (* dx dx) (* dy dy) ))))

(defun select-nearest-handles (target-pot handles)
  (let ((best-handle-1 nil)
        (best-distance-1 nil)
        (best-handle-2 nil)
        (best-distance-2 nil))
    (dolist (handle handles)
      (let ((distance (detection-distance target-pot handle)))
        (cond
          ((or (null best-distance-1) (< distance best-distance-1))
           (setq best-handle-2 best-handle-1)
           (setq best-distance-2 best-distance-1)
           (setq best-handle-1 handle)
           (setq best-distance-1 distance))
          ((or (null best-distance-2) (< distance best-distance-2))
           (setq best-handle-2 handle)
           (setq best-distance-2 distance)))))
    (when (and best-handle-1 best-handle-2)
      (list best-handle-1 best-handle-2))))

(defun assign-left-right-handles (handle-a handle-b)
  (let* ((center-a (pot-handle-detection-center handle-a))
         (center-b (pot-handle-detection-center handle-b))
         (y-a (elt center-a 1))
         (y-b (elt center-b 1)))
    (if (> y-a y-b)
        (list handle-a handle-b)
      (list handle-b handle-a))))

(defun select-target-pot-and-handles (pots handles)
  (let* ((target-pot (select-largest-pot pots))
         (nearest-handles (when target-pot
                            (select-nearest-handles target-pot handles))))
    (when nearest-handles
      (let* ((left-right (assign-left-right-handles
                          (car nearest-handles)
                          (cadr nearest-handles)))
             (left-handle (car left-right))
             (right-handle (cadr left-right)))
        (list target-pot left-handle right-handle)))))

(defun selected-target-pot (selection)
  (car selection))

(defun selected-left-handle (selection)
  (cadr selection))

(defun selected-right-handle (selection)
  (caddr selection))
