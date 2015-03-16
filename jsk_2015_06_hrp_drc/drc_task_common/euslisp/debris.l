#!/usr/bin/env roseus

;; (defvar *color-scale* 0.5)

(defun gen-bar-1
  nil
  (let* ((c (make-cube 85 40 1220)))
    (send c :set-color (float-vector (/ (* #xFD 1.0) #xFF) (/ (* #xFD 1.0) #xFF) (/ (* #xFC 1.0) #xFF)))
    c))

(defun gen-bar-2
  nil
  (let* ((c (make-cube 75 75 1220)))
    (send c :set-color (float-vector (/ (* #xFD 1.0) #xFF) (/ (* #xFD 1.0) #xFF) (/ (* #xFC 1.0) #xFF)))
    c))

(defun gen-bar-3
  nil
  (let* ((c (make-cube 75 39 1220)))
    (send c :set-color (float-vector (/ (* #xD3 1.0) #xFF) (/ (* #xD9 1.0) #xFF) (/ (* #xDB 1.0) #xFF)))
    c))

(defun gen-bar-4
  nil
  (let* ((c (make-cube 90 35 1220)))
    (send c :set-color (float-vector (/ (* #xFD 1.0) #xFF) (/ (* #xFD 1.0) #xFF) (/ (* #xFC 1.0) #xFF)))
    c))

(defun gen-trapezoid-bar-1
  nil
  (let* ((c (make-prism (list (float-vector (/ 85 2.0) (/ -35 2.0) (/ -1180 2.0))
			      (float-vector (/ -85 2.0) (/ -35 2.0) (/ -1180 2.0))
			      (float-vector (/ -65 2.0) (/ 35 2.0) (/ -1180 2.0))
			      (float-vector (/ 65 2.0) (/ 35 2.0) (/ -1180 2.0)))
			(float-vector 0 0 1180))))
    (send c :set-color (float-vector (/ (* #xFD 1.0) #xFF) (/ (* #xFD 1.0) #xFF) (/ (* #xFC 1.0) #xFF)))
    c))

(defun gen-pole-1
  nil
  (let* ((c (make-cylinder (/ 60 2.0) 1520)))
    (send c :set-color (float-vector (/ (* #xFE 1.0) #xFF) (/ (* #xFE 1.0) #xFF) (/ (* #xFD 1.0) #xFF)))
    c))

(defun gen-pole-2
  nil
  (let* ((c (make-cylinder (/ 35 2.0) 1220)))
    (send c :set-color (float-vector (/ (* #x8A 1.0) #xFF) (/ (* #x96 1.0) #xFF) (/ (* #xA2 1.0) #xFF)))
    c))

(defun gen-pole-3
  nil
  (let* ((c (make-cylinder (/ 120 2.0) 980)))
    (send c :set-color (float-vector (/ (* #xA9 1.0) #xFF) (/ (* #x77 1.0) #xFF) (/ (* #x6B 1.0) #xFF)))
    c))

(defun gen-pipe-1
  nil
  (let* ((a (make-cylinder (/ 110 2.0) 650))
	 (b (make-cylinder (/ 110 2.0) 205)))
    (send-all (list a b) :set-color (float-vector (/ (* #x3A 1.0) #xFF) (/ (* #x4D 1.0) #xFF) (/ (* #x5B 1.0) #xFF)))
    (send b :rotate (deg2rad 90) :x)
    (send b :translate (float-vector 0 0 110) :world)
    ;; (send b :translate (float-vector 0 0 5) :world)
    (send a :assoc b)
    (instance bodyset :init (make-cascoords) :bodies (list a b))))

(defun make-arc
  (radius-min radius-max
	      &key
	      (start-rad 0)
	      (end-rad 360)
	      (segments 16)
	      (rad start-rad)
	      (step-rad (/ (* 1.0 (- end-rad start-rad)) segments))
	      (step-len (/ (* 2.0 pi radius-max (/ (- end-rad start-rad) 360.0)) segments))
	      (color (float-vector (/ (* #x3A 1.0) #xFF) (/ (* #x4D 1.0) #xFF) (/ (* #x5B 1.0) #xFF)))
	      tmp (buf)
	      )
  (dotimes (i segments)
    (setq tmp (make-cylinder radius-min step-len))
    (send tmp :rotate (deg2rad rad) :x)
    (send tmp :translate (float-vector 0 radius-max (/ step-len -2.0)) :local)
    (send tmp :set-color color)
    (push tmp buf)
    (setq rad (+ rad step-rad))
    )
  (mapcar
   '(lambda (child parent) (send parent :assoc child))
   (cdr buf) buf)
  (instance bodyset :init (make-cascoords) :bodies buf))

(defun gen-arc-pipe-1
  nil
  (make-arc (/ 115 2.0) 800 :end-rad 72))

(defun four-leg-pole-1
  nil
  (labels
   ((in
     (&key
      (rot 45)
      (phi 50)
      (len 400)
      (phi-min 15)
      (len-min (/ 200 (sin (deg2rad rot))))
      (pole (make-cylinder (/ phi 2.0) len))
      (pole-min-1 (make-cylinder (/ phi-min 2.0) len-min))
      (pole-min-2 (make-cylinder (/ phi-min 2.0) len-min))
      )
     ;; (send pole :translate (float-vector 0 -100 0) :world)
     (send pole-min-1 :rotate (deg2rad rot) :x)
     (send pole-min-2 :rotate (deg2rad (+ 90 rot)) :x)
     (send pole-min-1 :translate (float-vector 0 0 0) :world)
     (send pole-min-2 :translate (float-vector 0 0 (- 400 0)) :world)
     (send-all (list pole pole-min-1 pole-min-2)
	       :set-color (float-vector (/ (* #xFE 1.0) #xFF) (/ (* #xFE 1.0) #xFF) (/ (* #xFD 1.0) #xFF)))
     (send pole :assoc pole-min-1)
     (send pole :assoc pole-min-2)
     (instance bodyset :init (make-cascoords)
	       :bodies (list pole pole-min-1 pole-min-2))
     ))
   (let* ((rot 0) (step 90) tmp (buf nil))
     (dotimes (i (/ 360 step))
       (setq tmp (in))
       (if buf (send tmp :newcoords (send (car buf) :copy-worldcoords)))
       (send tmp :rotate (deg2rad (* -1 step)) :z)
       (send tmp :translate (float-vector 200 0 0) :local)
       (push tmp buf))
     (setq buf (send-all buf :bodies))
     (mapcar #'(lambda (obj) (send (caar buf) :assoc (car obj))) (cdr buf))
     (instance bodyset :init (make-cascoords)
	       :bodies (flatten buf))
     )))

(defun gen-four-leg-pole-1
  (&key
   (pole-min-cnt 2)
   (len 400)
   )
  (labels
   ((in
     (&key
      (rot 45)
      (phi 50)
      (phi-min 15)
      (len-min (/ 200 (sin (deg2rad rot))))
      (pole (make-cylinder (/ phi 2.0) len))
      ;;(pole-min-1 (make-cylinder (/ phi-min 2.0) len-min))
      ;;(pole-min-2 (make-cylinder (/ phi-min 2.0) len-min))
      (pole-min (mapcar #'(lambda (a) (make-cylinder (/ phi-min 2.0) len-min))
			(make-list pole-min-cnt)))
      ;;
      (i 0)
      (rot-buf rot)
      (move-buf 0)
      (move-step (/ len pole-min-cnt))
      )
     ;; (send pole-min-1 :rotate (deg2rad rot) :x)
     ;; (send pole-min-2 :rotate (deg2rad (+ 90 rot)) :x)
     ;; (send pole-min-1 :translate (float-vector 0 0 0) :world)
     ;; (send pole-min-2 :translate (float-vector 0 0 (- 400 0)) :world)
     (dolist (p pole-min)
       (cond
	((evenp i)
	 (send p :rotate (deg2rad rot-buf) :x)
	 (send p :translate (float-vector 0 0 move-buf) :world))
	((oddp i)
	 (send p :rotate (deg2rad (+ 90 rot-buf)) :x)
	 (send p :translate
	       (float-vector 0 0
			     (+ move-step move-buf))
	       :world)))
       (setq i (+ i 1))
       ;; (setq rot-buf (+ rot-buf rot))
       (setq move-buf (+ move-buf move-step)))
     (send-all (cons pole pole-min) ;;(list pole pole-min-1 pole-min-2)
	       :set-color (float-vector (/ (* #xFE 1.0) #xFF) (/ (* #xFE 1.0) #xFF) (/ (* #xFD 1.0) #xFF)))
     ;;(send pole :assoc pole-min-1)
     ;;(send pole :assoc pole-min-2)
     (mapcar #'(lambda (a) (send pole :assoc a)) pole-min)
     (instance bodyset :init (make-cascoords)
	       :bodies (cons pole pole-min))
     ))
   (let* ((rot 0) (step 90) tmp (buf nil))
     (dotimes (i (/ 360 step))
       (setq tmp (in))
       (if buf (send tmp :newcoords (send (car buf) :copy-worldcoords)))
       (send tmp :rotate (deg2rad (* -1 step)) :z)
       (send tmp :translate (float-vector 200 0 0) :local)
       (push tmp buf))
     (setq buf (send-all buf :bodies))
     (mapcar #'(lambda (obj) (send (caar buf) :assoc (car obj))) (cdr buf))
     (instance bodyset :init (make-cascoords)
	       :bodies (flatten buf))
     )))

(defun gen-four-leg-pole-2
  nil
  (gen-four-leg-pole-1 :pole-min-cnt 3 :len 580))

(defun gen-suihanki
  nil
  (let* ((base (make-cube 180 260 10))
	 (pole (make-cylinder 75 200))
	 (cube (make-cube 120 40 100))
	 )
    (send-all (list base pole cube) :set-color
	      (float-vector (/ (* #xFE 1.0) #xFF) (/ (* #xFE 1.0) #xFF) (/ (* #xFD 1.0) #xFF)))
    (send pole :translate (float-vector 0 -25 0) :world)
    (send cube :translate (float-vector 0 90 50) :world)
    (send base :assoc pole)
    (send base :assoc cube)
    (instance bodyset :init (make-cascoords) :bodies (list base pole cube))))

(defun view-random-debris
  nil
  (objects
   (mapcar
    #'(lambda (func)
	(let* ((obj (funcall func)))
	  (send obj :newcoords
		(make-coords
		 :pos (scale 1000 (random-vector))
		 :rpy (scale 1.57 (random-vector))))))
    '(gen-suihanki gen-four-leg-pole-2 gen-four-leg-pole-1 gen-arc-pipe-1 gen-arc-pipe-1 gen-pipe-1 gen-pole-3 gen-pole-2 gen-pole-1 gen-pole-1 gen-bar-4 gen-bar-3 gen-bar-2 gen-bar-2 gen-bar-2 gen-bar-1)))
  (send *viewer* :draw-objects))


;; (require "drc-testbed.l")
;; (send (let ((c (instance drc-terrain)))
;; 	(send c :set-val 'block-dimensions (list 390 195 140))
;; 	c)
;;       :make-drc-block-one)

#|
