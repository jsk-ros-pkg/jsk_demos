(defvar *src* (unix:getwd))
(setf load-bak #'load)
(shadow 'load)
(make-package :ros)
(defun load (p) (funcall load-bak (fullpath p)))
(defun fullpath (p) (rs "package://jsk_maps" *src* p))
(defun rs (b a s)
  (cond
    ((substringp b s)
     (dotimes (i (- (length s) (length b)))
       (when (string= b (subseq s i (+ i (length b))))
         (return-from rs
           (concatenate string (subseq s 0 i) a (subseq s (+ i (length b)) (length s)))))))
    (t s)))
(defun ros::resolve-ros-path (p) (fullpath p))

(load "package://jsk_maps/src/dump-map-info.l")
