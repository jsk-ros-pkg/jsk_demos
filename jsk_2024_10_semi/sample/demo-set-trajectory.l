;; (defun test ()
;;   (let ((r nil))
;;     (dotimes (i 10)
;;       (push
;;        (make-cube 10 10 10 :pos (float-vector (* (- i 5) 20) 0 0))
;;        r))
;;     (dotimes (i (- (length r) 1))
;;       (send (elt r i) :assoc (elt r (+ 1 i))))
;;     r))
(require "package://pr2eus/pr2.l")
(require "package://pr2eus/pr2-utils.l")
(require "package://pr2eus/pr2-interface.l")
(if (not (boundp '*pr2*)) (pr2-init)) 

(defclass test1
  :super cascaded-coords
  :slots (points))
(defmethod test1
  (:init (&rest args)
	 (send-super* :init args)
    (dotimes (i 10)
      (push
       (make-cube 10 10 10 :pos (float-vector (* (- i 5) 20) 0 0))
       points))
    (dotimes (i (- (length points) 1))
      (send (elt points i) :assoc (elt points (+ 1 i))))
    (send self :assoc (car points))
    self)
  (:points () points)
  )
  


;;(setq r (test))
;;(send (car r) :rotate pi/2 :z)
;;(send (car r) :locate #f(800 0 900) :world)
(setq r (instance test1 :init))
(send r :rotate pi/2 :z)
(send r :locate #f(800 0 900) :world)
(objects (append (list *pr2*) (send r :points)))
(send *irtviewer* :draw-objects)
;;(objects (list *pr2* (elt (send r :points) 0) (elt (send r :points) 1) (elt (send r :points) 2)
;(dolist (e (send r :points)
;  (send *pr2* :rarm :inverse-kinematics e
;	:rotation-axis nil
;	:debug-view t)
;  (send *irtviewer* :draw-objects))
