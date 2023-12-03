(defun point-mul (point x1 x2 y z)
  (if (> (elt point 0) 0) (float-vector (* (elt point 0) x1) (* (elt point 1) y) (* (elt point 2) z))
    (float-vector (* (elt point 0) x2) (* (elt point 1) y) (* (elt point 2) z))))

(defun make-ellipsoid-ratio (width1 width2 length height)
  (let (b1 b2)
    (setq x-scale1 (/ (float width1) (/ (float height) 2)))
    (setq x-scale2 (/ (float width2) (/ (float height) 2)))
    (setq y-scale (/ (float length) (float height)))
    (setq b1 (make-sphere (/ height 2)))
    (setq v-list (list (point-mul (car (send b1 :vertices)) x-scale1 x-scale2 y-scale 1)))
    (dolist (x (cdr (send b1 :vertices))) (setq v-list (append v-list (list (point-mul x x-scale1 x-scale2 y-scale 1)))))
    (setq b2 (convex-hull-3d v-list))
    ;; (send b2 :locate (float-vector (/ (- width2 width1) 2) 0 0))
    ))


(defun body-cut-x-with-c (abody box coords)
  (let (b1 b2 c-flag i)
    (setq c-flag nil)
    (setq i 0)
    (until c-flag
       (catch 'hoge
         (unwind-protect
         (progn
           (setq *error-handler* #'(lambda (code msg1 form &optional msg2) (throw 'hoge -1)));;エラーハンドラを変える
           (setq b1 (body- abody box)) ;;ここでエラーが起こるとcatchを抜ける
           (setq c-flag t)
           0)
           (setq *error-handler* #'euserror)));;エラーハンドラをもとに戻す
       (send coords :locate #f(-1 0 0))
       (setq i (+ i 1))
       )

    (setq b2 (body- abody b1))
    (list b1 b2)))

(defun body-cut-y (abody box)
  (let (b1 b2 c-flag i)
    (setq c-flag nil)
    (setq i 0)
    (until c-flag
       (catch 'hoge
         (unwind-protect
         (progn
           (setq *error-handler* #'(lambda (code msg1 form &optional msg2) (throw 'hoge -1)));;エラーハンドラを変える
           (setq b1 (body- abody box)) ;;ここでエラーが起こるとcatchを抜ける
           (setq c-flag t)
           0)
           (setq *error-handler* #'euserror)));;エラーハンドラをもとに戻す
       (send box :locate #f(0 -1 0))
       (setq i (+ i 1))
       )

    (setq b2 (body- abody b1))
    (list b1 b2)))

(defun body-cut-x (abody box)
  (let (b1 b2 c-flag i)
    (setq c-flag nil)
    (setq i 0)
    (until c-flag
       (catch 'hoge
         (unwind-protect
         (progn
           (setq *error-handler* #'(lambda (code msg1 form &optional msg2) (throw 'hoge -1)));;エラーハンドラを変える
           (setq b1 (body- abody box)) ;;ここでエラーが起こるとcatchを抜ける
           (setq c-flag t)
           0)
           (setq *error-handler* #'euserror)));;エラーハンドラをもとに戻す
       (send box :locate #f(-1 0 0))
       (setq i (+ i 1))
       )
    
    (setq b2 (body- abody b1))
    (list b1 b2))) 

(defun dev-2-y (abody)
  (let (b box b1 b2 m m2)
    (setq b (make-bounding-box (send abody :vertices)))

    (setq m (v- (send b :maxpoint) (send b :minpoint)))
    (setq m2 (v+ (send b :maxpoint) (send b :minpoint)))
    (setq m2 (float-vector (/ (elt m2 0) 2) (/ (elt m2 1) 2) (/ (elt m2 2) 2)))
    
    (setq box (make-cube (* (elt m 0) 2) (* (elt m 1) 2) (* (elt m 2) 2)))
    (send box :locate m2)
    (send box :locate (float-vector 0 (* 1 (elt m 1))  0))

    (body-cut-y abody box)))

(defun slice-x-10 (abody)
  (let (b box b-list b1 b2 b3 r)
    (setq b (make-bounding-box (send abody :vertices)))

    (setq m (v- (send b :maxpoint) (send b :minpoint)))
    (setq m2 (v+ (send b :maxpoint) (send b :minpoint)))
    (setq m2 (float-vector (/ (elt m2 0) 2) (/ (elt m2 1) 2) (/ (elt m2 2) 2)))
    
    (setq box (make-cube (* (elt m 0) 2) (* (elt m 1) 2) (* (elt m 2) 2)))
    (send box :locate m2)
    (send box :locate (float-vector (* 3 (/ (elt m 0) 2)) 0 0))

    (setq b1 abody)
    (setq num (- (round (/ (elt m 0) 10)) 1))
    
    (dotimes (i num)
      (send box :locate #f(-10 0 0))
      (setq b-list (body-cut-x b1 box))
      (setq b1 (elt b-list 0))
      (setq b2 (elt b-list 1))

      (push b2 r)
      )

    (push b1 r)

    r))

(defun random-cut (abody)
  (let (b m box c  b-list b1 b2 b3 r)
    (setq b (make-bounding-box (send abody :vertices)))
    (setq m (v- (send b :maxpoint) (send b :minpoint)))
    
    (setq box (make-cube (* (elt m 0) 2) (* (elt m 1) 2) (* (elt m 2) 2)))
    (send box :locate (send b :center))
    (send box :translate (float-vector (* (elt m 0) -1) 0 0))
    (setq c (make-cascoords :pos (send b :center)))
    
    (send c :assoc box)
    (send c :translate (float-vector (/ (elt m 0) -2) 0 0) :world)

    (setq b1 abody)
    
    (send c :rotate (deg2rad 45) :z)
    (send c :translate #f(10 0 0) :world)
    (setq tmp (body-cut-x-with-c b1 box c))
    (setq b1 (elt tmp 0))
    (setq b2 (elt tmp 1))
    (push b2 r)

    (setq num (- (round (/ (elt m 0) 10)) 1)) 

    (dotimes (i num)
      (send c :rotate (deg2rad 90) :x :world)
      (send c :translate #f(10 0 0) :world)
      (setq tmp (body-cut-x-with-c b1 box c))
      (setq b1 (elt tmp 0))
      (setq b2 (elt tmp 1))
      (push b2 r))

    (push b1 r)

    r))


(defclass carrot-object
  :super bodyset-link
  :slots (width
      width1
      width2
      length
      height
      ))

(defmethod carrot-object
  (:init
   (&rest args &key (w 100) (l 30) (h 30) &allow-other-keys)
   (let (b)
     (setq width w)
     (setq width1 20)
     (setq width2 (- w 20))
     (setq length l)
     (setq height h)

     (setq b (make-ellipsoid-ratio width1 width2 l h)) 
     (send b :set-color :orangered)
     (send-super* :init (make-cascoords) :bodies (list b) args)))
  (:cut2
   ()
   (let (b1 b2 b3 r)
     (setq b1 (make-ellipsoid-ratio width1 width2 length height))

     (setq b2 (make-cube (* (+ width1 width2) 2) (* 2 length) (* 2 height)))
     (send b2 :locate (float-vector (- (+ (* 2 width1) width2) 10) 0 0))
     (setq b-list (body-cut-x b1 b2))
     (setq b1 (elt b-list 0))
     (setq c1 (elt b-list 1))

     (send b2 :locate (float-vector (+ (* -0.45 (+ width1 width2)) 10) 0 0))
     (setq b-list (body-cut-x b1 b2))
     (setq b1 (elt b-list 0))
     (setq c2 (elt b-list 1))

     (send b2 :locate (float-vector (+ (* -0.55 (+ width1 width2)) 10) 0 0))
     (setq b-list (body-cut-x b1 b2))
     (setq c4 (elt b-list 0))
     (setq c3 (elt b-list 1))
     
     (send c1 :set-color :orangered)
     (send c2 :set-color :orangered)
     (send c3 :set-color :orangered)
     (send c4 :set-color :orangered)     

     (setq *cut-point-1* (make-cascoords :pos (float-vector (- width1 10) 0 (/ height -2))))
     (send *cut-point-1* :rotate (deg2rad 90) :z)
     (setq *cut-point-2* (make-cascoords :pos (float-vector (+ width1 (* -0.55 (+ width1 width2))) 0 (/ height -2))))
     (send *cut-point-2* :rotate (deg2rad 90) :z)
     (setq *cut-point-3* (make-cascoords :pos (float-vector (- 10 width2) 0 (/ height -2))))
     (send *cut-point-3* :rotate (deg2rad 90) :z)

     (send c2 :assoc *cut-point-1*)
     (send c2 :assoc *cut-point-2*)
     (send c3 :assoc *cut-point-3*)

     (send c2 :assoc c1)
     (send c2 :assoc c3)
     (send c3 :assoc c4)

     (send c2 :newcoords (send self :copy-worldcoords))
     (send self :assoc c2)
     
     (push c2 r)
     (push c1 r)
     (push c3 r)
     (push c4 r)
     (setq geo::bodies r))
   )    
  (:cut
   ()
   (let (b1 b2 b3 r)
     (setq b1 (make-ellipsoid-ratio width1 width2 length height))

     (setq b2 (make-cube (* (+ width1 width2) 2) (* 2 length) (* 2 height)))
     (send b2 :locate (float-vector (- (+ (* 2 width1) width2) 15) 0 0))
     (setq b-list (body-cut-x b1 b2))
     (setq b1 (elt b-list 0))
     (setq c1 (elt b-list 1))

     (send b2 :locate (float-vector (+ (* -1 (+ width1 width2)) 30) 0 0))
     (setq b-list (body-cut-x b1 b2))
     (setq c3 (elt b-list 0))
     (setq c2 (elt b-list 1))

     (send c1 :set-color :orangered)
     (send c2 :set-color :orangered)
     (send c3 :set-color :orangered)

     (setq *cut-point-1* (make-cascoords :pos (float-vector (- width1 10) 0 (/ height -2))))
     (send *cut-point-1* :rotate (deg2rad 90) :z)
     (setq *cut-point-2* (make-cascoords :pos (float-vector (- 10 width2) 0 (/ height -2))))
     (send *cut-point-2* :rotate (deg2rad -90) :z)

     (send c2 :assoc *cut-point-1*)
     (send c2 :assoc *cut-point-2*)

     (send c2 :assoc c1)
     (send c2 :assoc c3)

     (send c2 :newcoords (send self :copy-worldcoords))
     (send self :assoc c2)

     (push c2 r)
     (push c1 r)
     (push c3 r)
     (setq geo::bodies r))
   )    
  )



(defun carrot (&rest args) (instance* carrot-object :init args))
