#|
Desctiption :
Make a bend needle model here!

Author :
@Michi-Tsubaki <michi.tsubaki.tech@gmail.com>
|#

;; Load irteus visualization
(require "package://pr2eus/pr2.l")
(require "package://pr2eus/pr2-utils.l")
(require "package://pr2eus/pr2-interface.l")
(require "package://pr2eus/speak.l")

; Define needle parameters
(setq *needle-radius* 35)      ; radius of curvature
(setq *needle-thickness* 0.5)  ; thickness of the needle
(setq *needle-angle* (/ pi 2)) ; angle of the arc

;; Create points for the needle path
(setq segments 40)
(setq theta-step (/ *needle-angle* segments))
(setq points nil)
(dotimes (i (+ segments 1))
  (let ((theta (* i theta-step)))
    (push (float-vector 
           (* *needle-radius* (- 1 (cos theta)))  ; x
           0                                      ; y
           (* *needle-radius* (sin theta)))       ; z
          points)))
(setq points (reverse points))

;; Function to calculate rotation matrix from two points
(defun make-rotation-matrix (p1 p2)
  (let* ((diff (v- p2 p1))
         (z-axis (normalize-vector diff))
         (y-axis (float-vector 0 1 0))
         (x-axis (v* y-axis z-axis)))
    (setq y-axis (v* z-axis x-axis))  ; ensure orthogonality
    (transpose (matrix (normalize-vector x-axis) 
                      (normalize-vector y-axis) 
                      (normalize-vector z-axis)))))

;; Create cylinders along the arc
(setq needle-parts nil)
(let ((prev-point (car points)))
  (dolist (point (cdr points))
    (let* ((diff (v- point prev-point))
           (height (norm diff))
           (center (midpoint 0.5 prev-point point))
           (cylinder (make-cylinder *needle-thickness* height :segments 12))
           (rot-mat (make-rotation-matrix prev-point point)))
      ;; Create coordinates at center with proper orientation
      (setq coords (make-coords :pos center :rot rot-mat))
      ;; Orient and position cylinder
      (send cylinder :transform coords)
      (push cylinder needle-parts)
      (setq prev-point point))))

;; Create the bodyset
(setq needle-bodies (reverse needle-parts))

;; Associate all parts with the first one
(dolist (body (cdr needle-bodies))
  (send (car needle-bodies) :assoc body))

;; 針の色（銀色）にする
(dolist (body needle-bodies)
  (send body :set-color :silver))

;; needleをbodyset-linkとして定義する
(setq *needle* 
      (instance bodyset-link :init (make-cascoords)
                :bodies needle-bodies))

;; 表示する
(make-irtviewer)
(objects (list *needle*))

;; Optional: rotate the view to better see the needle
(send *irtviewer* :viewing :look #f(0 -1 0.1))
(send *irtviewer* :draw-objects)
