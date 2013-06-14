#!/usr/bin/env irteusgl


(load "models/room610-table-object.l")
(load "~/prog/euslib/jsk/pfilter.l")
(defparameter *result-topic* "/narrow_stereo/left/ObjectDetection")
(defparameter *check-particles-topic* "/check_particles_topic")
(defvar *base-frame-id* "/base_footprint")


(setq *room610-origin* "/eng8/6f/610")
(setq *tobject* (instance room610-table-object :init))

;;3rd argument is not meaningful
(setq *object-foot1* (float-vector (/  1.09 2) (/  0.63 2) 0)) ;;1.09
(setq *object-foot2* (float-vector (/ -1.09 2) (/  0.63 2) 0)) ;;0.63
(setq *object-foot3* (float-vector (/  1.09 2) (/ -0.63 2) 0))
(setq *object-foot4* (float-vector (/ -1.09 2) (/ -0.63 2) 0))
(setq *error-thred* 0.05) ;;0.05
(setq *object-xy* nil)
(setq *inner-threshold* 0.2)


(setq *detect-pf* nil)
(setq *detect-system-noise* 0.05)
(setq *detect-measurement-noise* 0.1)
(setq *detect-threshold* 0.1)


(defclass detect-particle-filter
  :super particle-filter)
(defmethod detect-particle-filter
  (:likelihood
   (x y)
   (let ((foot1 (v+ x (rotate-vector *object-foot1* (elt x 2) :z)))
         (foot2 (v+ x (rotate-vector *object-foot2* (elt x 2) :z)))
         (foot3 (v+ x (rotate-vector *object-foot3* (elt x 2) :z)))
         (foot4 (v+ x (rotate-vector *object-foot4* (elt x 2) :z)))
         (foot-list nil)
         (error-sum 0) (error-list nil)
         )
     (setq foot-list (list foot1 foot2 foot3 foot4))
     (print "foot-list ~A " foot-list)
     (dolist (foot foot-list)
       (if (> (length y) 0) (setq min-error (distance foot (elt y 0))))
       (dolist (measure-point y)
         (setq d (distance measure-point foot))
         (if (> min-error d) (setq min-error d))
         )
       (push min-error error-list)
       )
     (setq error-pass-num 0)

     (dolist (error-val error-list)
       ;;is error is over threshold?
       (if (< error-val *detect-threshold*)
           (progn
             (+ error-sum error-val)
             (incf error-pass-num)
             )
         )
       )
     (if (> error-pass-num 2)
         (progn
           (print "~A :error-list" error-list)
           (exp (* measurement-noise error-sum))
           )
       0
       )
     )
   )
  )

(defun init-particles ()
  (setq *detect-pf* (instance detect-particle-filter :init :state-dim 3
                              :particle-num 200
                              :system-noise *detect-system-noise*
                              :measurement-noise *detect-measurement-noise*
                              :initial-noise 1.0
                              :use-adaptive-particle-num t))
  (send *detect-pf* :particle-num 20)
  (send *detect-pf* :initial-noise 0.1)
;;  (setq pr2->target (send (send *pr2* :base_laser_link) :transformation *target-object*))
;;  (steq pr2->target-pos (send pr2->target :worldpos))
;;  (setq pr2->target-rot (send pr2->target :rpy-angle))
;;  (send *detect-pf* :initial-state (float-vector (elt pr2->target-pos 0) (elt pr2->target-pos 1) (rad2deg (car (car pr2->target-rot)))))
  (send *detect-pf* :reset-particles)
  )

;;convert r theta to xy
(defun rtheta->xy (r theta &key (debug nil))
  (let (x y)
    (setq x (* r (cos theta)))
    (setq y (* r (sin theta)))
    (when debug
      (print  "~d ~d x-y" x y)
      )
    (float-vector x y )
    )
  )

(defun distance (x y)
  (sqrt (+ (* x x) (* y y)))
  )

;;calc the distance between two lists
(defun between-distance (xy1 xy2)
  (let (x-diff y-diff)
    (setq x-diff (- (cadr xy1) (cadr xy2)))
    (setq y-diff (- (caddr xy1) (caddr xy2)))
    (distance x-diff y-diff)
    )
  )

;;return if distance is within right range or not
(defun check-likely-target (sample-length target-length &optional (error-thred *error-thred*))
  (if (and (<= sample-length (+ target-length error-thred)) (>= sample-length (- target-length error-thred))) 
      t
    nil
    )
  )

(defun distance-from-origin (xy &key (x-index 0) (y-index 1))
  (let ((x (elt xy x-index))( y (elt xy y-index)))
    (distance x y)
    )
  )

;;CALL-BACK this will be called when subscriver is activated
(defun table-detection-cb (laser-scan-msg)
  (let ((trans nil) (angle (send laser-scan-msg :angle_min)) (angle-step (send laser-scan-msg :angle_increment)) (ranges-vec (send laser-scan-msg :ranges)) (x 0) (y 0) (xy-list nil) (object-width-likely nil) (object-height-likely nil) (object-width2-likely nil) (object-height2-likely nil) ret)

    (setq ranges-vec (send laser-scan-msg :ranges))
    (setq ranges (coerce ranges-vec cons))

    ;; First Calc the x y and put those in xy-list
    ;; and add index too.
    (dolist (range ranges)
      ;;here if range > 0 it is meaningful
      (if (> range 0)
	  (push (rtheta->xy range angle) xy-list)
	)
      (setq angle (+ angle angle-step))
      )

    (setq ret (send *detect-pf* :update xy-list))
    (send *tobject* :reset-coords)
    (send *tobject* :locate (float-vector (elt ret 0) (elt ret 1) 0))
    (send *tobject* :rotate (elt ret 2) :z)

    
))