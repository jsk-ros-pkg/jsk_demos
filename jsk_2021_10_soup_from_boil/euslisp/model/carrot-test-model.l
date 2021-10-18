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
  :slots (
      ))

(defmethod carrot-object
  (:init
   (&rest args &key (name "carrot"))
   (let (b1 b2 b3)
     (setq v-list (list #f(100.0 -15.0 25.9808) #f(100.0 -25.9808 15.0) #f(100.0 -30.0 -4.085621e-14) #f(100.0 -25.9808 -15.0) #f(100.0 -15.0 -25.9808) #f(100.0 2.842171e-14 -30.0) #f(100.0 15.0 -25.9808) #f(100.0 25.9808 -15.0) #f(100.0 30.0 1.598721e-14) #f(100.0 25.9808 15.0) #f(100.0 15.0 25.9808) #f(100.0 0.0 30.0)))
     
     (setq b2 (make-cone #f(-100 0 0) v-list))

     (setq b3 (make-sphere 30))
     (send b3 :locate #f(100 0 0))
     (setq b4 (make-cube 100 100 100))
     (send b4 :translate #f(49 0 0))
     (setq b3 (body- b3 b4))

     (send b2 :set-color :orangered)
     (send b3 :set-color :orangered)

     (send b2 :assoc b3)
     (send-super* :init (make-cascoords) :bodies (list b2 b3) args)))
  (:cut
   ()
   (let (t1 b2 b3 b4 b5 b6 b7 r)
     (setq v-list (list #f(100.0 -15.0 25.9808) #f(100.0 -25.9808 15.0) #f(100.0 -30.0 -4.085621e-14) #f(100.0 -25.9808 -15.0) #f(100.0 -15.0 -25.9808) #f(100.0 2.842171e-14 -30.0) #f(100.0 15.0 -25.9808) #f(100.0 25.9808 -15.0) #f(100.0 30.0 1.598721e-14) #f(100.0 25.9808 15.0) #f(100.0 15.0 25.9808) #f(100.0 0.0 30.0)))


     (setq b2 (make-cone #f(-100 0 0) v-list))

     (setq b3 (make-sphere 30))
     (send b3 :locate #f(100 0 0))

     (setq b4 (make-cube 100 100 100))
     (send b4 :translate #f(49 0 0))
     ;; (setq b3 (body- b3 b4))
     (setq c1 (body- b3 b4))

     (setq b5 (make-cube 200 200 200))
     (send b5 :translate #f(-70 0 0))
     (setq c2 (body- b2 b5))

     (setq t1 (body- b2 c2))
     (send b5 :translate #f(-90 0 0))
     (setq c3 (body- t1 b5))
     (setq c4 (body- t1 c3))


     (setq b-list (dev-2-y c2))
     (setq b6 (elt b-list 0))
     (setq b7 (elt b-list 1))

     (setq b-list (slice-x-10 b6))
     (setq c2-1-1 (elt b-list 0))
     (setq c2-1-2 (elt b-list 1))
     (setq c2-1-3 (elt b-list 2))
     (setq c2-1-4 (elt b-list 3))
     (setq c2-1-5 (elt b-list 4))
     (setq c2-1-6 (elt b-list 5))
     (setq c2-1-7 (elt b-list 6))

     (setq b-list (slice-x-10 b7))
     (setq c2-2-1 (elt b-list 0))
     (setq c2-2-2 (elt b-list 1))
     (setq c2-2-3 (elt b-list 2))
     (setq c2-2-4 (elt b-list 3))
     (setq c2-2-5 (elt b-list 4))
     (setq c2-2-6 (elt b-list 5))
     (setq c2-2-7 (elt b-list 6))

     (send c2-1-1 :set-color :orangered)
     (send c2-1-2 :set-color :orangered)
     (send c2-1-3 :set-color :orangered)
     (send c2-1-4 :set-color :orangered)
     (send c2-1-5 :set-color :orangered)
     (send c2-1-6 :set-color :orangered)
     (send c2-1-7 :set-color :orangered)
     (send c2-2-1 :set-color :orangered)
     (send c2-2-2 :set-color :orangered)
     (send c2-2-3 :set-color :orangered)
     (send c2-2-4 :set-color :orangered)
     (send c2-2-5 :set-color :orangered)
     (send c2-2-6 :set-color :orangered)
     (send c2-2-7 :set-color :orangered)

     (send c2-1-1 :assoc c2-1-2)
     (send c2-1-1 :assoc c2-1-3)
     (send c2-1-1 :assoc c2-1-4)
     (send c2-1-1 :assoc c2-1-5)
     (send c2-1-1 :assoc c2-1-6)
     (send c2-1-1 :assoc c2-1-7)
     (send c2-2-1 :assoc c2-2-2)
     (send c2-2-1 :assoc c2-2-3)
     (send c2-2-1 :assoc c2-2-4)
     (send c2-2-1 :assoc c2-2-5)
     (send c2-2-1 :assoc c2-2-6)
     (send c2-2-1 :assoc c2-2-7)
     (send c2-1-1 :assoc c2-2-1)

     (setq b-list (dev-2-y c3))
     (setq b6 (elt b-list 0))
     (setq b7 (elt b-list 1))

     (setq b-list (slice-x-10 b6))
     (setq c3-1-1 (elt b-list 0))
     (setq c3-1-2 (elt b-list 1))
     (setq c3-1-3 (elt b-list 2))
     (setq c3-1-4 (elt b-list 3))
     (setq c3-1-5 (elt b-list 4))
     (setq c3-1-6 (elt b-list 5))
     (setq c3-1-7 (elt b-list 6))
     (setq c3-1-8 (elt b-list 7))
     (setq c3-1-9 (elt b-list 8))

     (setq b-list (slice-x-10 b7))
     (setq c3-2-1 (elt b-list 0))
     (setq c3-2-2 (elt b-list 1))
     (setq c3-2-3 (elt b-list 2))
     (setq c3-2-4 (elt b-list 3))
     (setq c3-2-5 (elt b-list 4))
     (setq c3-2-6 (elt b-list 5))
     (setq c3-2-7 (elt b-list 6))
     (setq c3-2-8 (elt b-list 7))
     (setq c3-2-9 (elt b-list 8))


     (send c3-1-1 :set-color :orangered)
     (send c3-1-2 :set-color :orangered)
     (send c3-1-3 :set-color :orangered)
     (send c3-1-4 :set-color :orangered)
     (send c3-1-5 :set-color :orangered)
     (send c3-1-6 :set-color :orangered)
     (send c3-1-7 :set-color :orangered)
     (send c3-1-8 :set-color :orangered)
     (send c3-1-9 :set-color :orangered)
     (send c3-2-1 :set-color :orangered)
     (send c3-2-2 :set-color :orangered)
     (send c3-2-3 :set-color :orangered)
     (send c3-2-4 :set-color :orangered)
     (send c3-2-5 :set-color :orangered)
     (send c3-2-6 :set-color :orangered)
     (send c3-2-7 :set-color :orangered)
     (send c3-2-8 :set-color :orangered)
     (send c3-2-9 :set-color :orangered)

     (send c3-1-1 :assoc c3-1-2)
     (send c3-1-1 :assoc c3-1-3)
     (send c3-1-1 :assoc c3-1-4)
     (send c3-1-1 :assoc c3-1-5)
     (send c3-1-1 :assoc c3-1-6)
     (send c3-1-1 :assoc c3-1-7)
     (send c3-1-1 :assoc c3-1-8)
     (send c3-1-1 :assoc c3-1-9)
     (send c3-2-1 :assoc c3-2-2)
     (send c3-2-1 :assoc c3-2-3)
     (send c3-2-1 :assoc c3-2-4)
     (send c3-2-1 :assoc c3-2-5)
     (send c3-2-1 :assoc c3-2-6)
     (send c3-2-1 :assoc c3-2-7)
     (send c3-2-1 :assoc c3-2-8)
     (send c3-2-1 :assoc c3-2-9)
     (send c3-1-1 :assoc c3-2-1)



     (send c1 :set-color :orangered)
     (send c4 :set-color :orangered)

     (setq *cut-point-1* (make-cascoords :pos #f(100 0 0)))
     (send *cut-point-1* :rotate (deg2rad 90) :z)

     (setq *cut-point-2* (make-cascoords :pos #f(30 0 0)))
     (send *cut-point-2* :rotate (deg2rad 90) :z)

     (setq *cut-point-3* (make-cascoords :pos #f(-60 0 0)))
     (send *cut-point-3* :rotate (deg2rad 90) :z)
     (send *cut-point-3* :rotate (deg2rad 180) :z :world)

     (setq *cut-point-4* (make-cascoords :pos #f(65 0 -30)))

     (send c2-1-1 :assoc *cut-point-1*)
     (send c2-1-1 :assoc *cut-point-2*)
     (send c3-1-1 :assoc *cut-point-3*)
     (send c2-1-1 :assoc *cut-point-4*)

     (send c2-1-1 :assoc c1)
     (send c2-1-1 :assoc c3-1-1)
     (send c3-1-1 :assoc c4)

     (send c2-1-1 :newcoords (send self :copy-worldcoords))
     (send self :assoc c2-1-1)

     (push c2-1-1 r)
     (push c2-1-2 r)
     (push c2-1-3 r)
     (push c2-1-4 r)
     (push c2-1-5 r)
     (push c2-1-6 r)
     (push c2-1-7 r)
     (push c2-2-1 r)
     (push c2-2-2 r)
     (push c2-2-3 r)
     (push c2-2-4 r)
     (push c2-2-5 r)
     (push c2-2-6 r)
     (push c2-2-7 r)
     (push c1 r)
     (push c4 r)
     (push c3-1-1 r)
     (push c3-1-2 r)
     (push c3-1-3 r)
     (push c3-1-4 r)
     (push c3-1-5 r)
     (push c3-1-6 r)
     (push c3-1-7 r)
     (push c3-1-8 r)
     (push c3-1-9 r)
     (push c3-2-1 r)
     (push c3-2-2 r)
     (push c3-2-3 r)
     (push c3-2-4 r)
     (push c3-2-5 r)
     (push c3-2-6 r)
     (push c3-2-7 r)
     (push c3-2-8 r)
     (push c3-2-9 r)
     (setq geo::bodies r))
   )
  (:random
   ()
   (let (t1 b2 b3 b4 b5 b6 b7 r)   
     (setq v-list (list #f(100.0 -15.0 25.9808) #f(100.0 -25.9808 15.0) #f(100.0 -30.0 -4.085621e-14) #f(100.0 -25.9808 -15.0) #f(100.0 -15.0 -25.9808) #f(100.0 2.842171e-14 -30.0) #f(100.0 15.0 -25.9808) #f(100.0 25.9808 -15.0) #f(100.0 30.0 1.598721e-14) #f(100.0 25.9808 15.0) #f(100.0 15.0 25.9808) #f(100.0 0.0 30.0)))

     (setq b2 (make-cone #f(-100 0 0) v-list))
     
     (setq b3 (make-sphere 30))
     (send b3 :locate #f(100 0 0))

     (setq b4 (make-cube 100 100 100))
     (send b4 :translate #f(49 0 0))
     (setq c1 (body- b3 b4))

     (setq b5 (make-cube 200 200 200))
     (send b5 :translate #f(-70 0 0))
     (setq c2 (body- b2 b5))

     (setq t1 (body- b2 c2))
     (send b5 :translate #f(-90 0 0))
     (setq c3 (body- t1 b5))
     (setq c4 (body- t1 c3))

     (setq b-list (random-cut c2))
     (setq c2-1-1 (elt b-list 0))
     (setq c2-1-2 (elt b-list 1))
     (setq c2-1-3 (elt b-list 2))
     (setq c2-1-4 (elt b-list 3))
     (setq c2-1-5 (elt b-list 4))
     (setq c2-1-6 (elt b-list 5))
     (setq c2-1-7 (elt b-list 6))
     (setq c2-1-8 (elt b-list 7))

     (setq b-list (random-cut c3))
     (setq c3-1-1 (elt b-list 0))
     (setq c3-1-2 (elt b-list 1))
     (setq c3-1-3 (elt b-list 2))
     (setq c3-1-4 (elt b-list 3))
     (setq c3-1-5 (elt b-list 4))
     (setq c3-1-6 (elt b-list 5))
     (setq c3-1-7 (elt b-list 6))
     (setq c3-1-8 (elt b-list 7))
     (setq c3-1-9 (elt b-list 8))
     (setq c3-1-10 (elt b-list 9))

     (send c2-1-1 :set-color :orangered)
     (send c2-1-2 :set-color :orangered)
     (send c2-1-3 :set-color :orangered)
     (send c2-1-4 :set-color :orangered)
     (send c2-1-5 :set-color :orangered)
     (send c2-1-6 :set-color :orangered)
     (send c2-1-7 :set-color :orangered)
     (send c2-1-8 :set-color :orangered)

     (send c2-1-1 :assoc c2-1-2)
     (send c2-1-1 :assoc c2-1-3)
     (send c2-1-1 :assoc c2-1-4)
     (send c2-1-1 :assoc c2-1-5)
     (send c2-1-1 :assoc c2-1-6)
     (send c2-1-1 :assoc c2-1-7)
     (send c2-1-1 :assoc c2-1-8)

     (send c3-1-1 :set-color :orangered)
     (send c3-1-2 :set-color :orangered)
     (send c3-1-3 :set-color :orangered)
     (send c3-1-4 :set-color :orangered)
     (send c3-1-5 :set-color :orangered)
     (send c3-1-6 :set-color :orangered)
     (send c3-1-7 :set-color :orangered)
     (send c3-1-8 :set-color :orangered)
     (send c3-1-9 :set-color :orangered)
     (send c3-1-10 :set-color :orangered)

     (send c3-1-1 :assoc c3-1-2)
     (send c3-1-1 :assoc c3-1-3)
     (send c3-1-1 :assoc c3-1-4)
     (send c3-1-1 :assoc c3-1-5)
     (send c3-1-1 :assoc c3-1-6)
     (send c3-1-1 :assoc c3-1-7)
     (send c3-1-1 :assoc c3-1-8)
     (send c3-1-1 :assoc c3-1-9)
     (send c3-1-1 :assoc c3-1-10)

     (send c1 :set-color :orangered)
     (send c4 :set-color :orangered)

     (setq *cut-point-1* (make-cascoords :pos #f(100 0 0)))
     (send *cut-point-1* :rotate (deg2rad 90) :z)

     (setq *cut-point-2* (make-cascoords :pos #f(30 0 0)))
     (send *cut-point-2* :rotate (deg2rad 90) :z)

     (setq *cut-point-3* (make-cascoords :pos #f(-60 0 0)))
     (send *cut-point-3* :rotate (deg2rad 90) :z)
     (send *cut-point-3* :rotate (deg2rad 180) :z :world)

     (setq *cut-point-4* (make-cascoords :pos #f(65 0 -30)))

     (send c2-1-1 :assoc *cut-point-1*)
     (send c2-1-1 :assoc *cut-point-2*)
     (send c3-1-1 :assoc *cut-point-3*)
     (send c2-1-1 :assoc *cut-point-4*)

     (send c2-1-1 :assoc c1)
     (send c2-1-1 :assoc c3-1-1)
     (send c3-1-1 :assoc c4)

     (send c2-1-1 :newcoords (send self :copy-worldcoords))
     (send self :assoc c2-1-1)
     
     (push c2-1-1 r)
     (push c2-1-2 r)
     (push c2-1-3 r)
     (push c2-1-4 r)
     (push c2-1-5 r)
     (push c2-1-6 r)
     (push c2-1-7 r)
     (push c2-1-8 r)
     (push c1 r)
     (push c4 r)
     (push c3-1-1 r)
     (push c3-1-2 r)
     (push c3-1-3 r)
     (push c3-1-4 r)
     (push c3-1-5 r)
     (push c3-1-6 r)
     (push c3-1-7 r)
     (push c3-1-8 r)
     (push c3-1-9 r)
     (push c3-1-10 r)
     (setq geo::bodies r))
   )

  )


(defun carrot (&rest args) (instance* carrot-object :init args))
