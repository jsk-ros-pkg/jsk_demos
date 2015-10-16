;;dump-convex-hull is a function created by noda-san
;;
(load "math.l")
(require "~/prog/euslib/irteus_proposals/motion-lib-proposal.l")
;for use of calc-static-balance-point


(defun dump-convex-hull;define function
  (&key (robot *robot*));option var(name or robot)
  ;;show objects which follows this. In this case robot&foot-convex will appear
  (setq *foot-convex*
         (make-prism (butlast (send (send robot :support-polygon '(:rleg :lleg)) :vertices)) -5))
   (send *foot-convex* :set-color #f(0 1 0))
   (objects (cons *foot-convex* (objects)))
   )

(defun print-all-edges  nil
  (mapcar #'(lambda (e) (print e)) (send *foot-convex* :edges))
  )

;;get edges in top face of foot-convex
(defun get-foot-convex-plain (&key (robot *robot*))
  (pprint  (send (cadr (send *foot-convex* :faces)) :edges))
  )



;;get intersecton on line1 with line2
;;send line1 :point (car (send line1 :intersection line2))
;;send line1 :intersection line2
;;上の式は、ベクトルaとbを何倍に拡大した点で交わるかを返す

;;get the length of line which start from ZMP to the edge of the convex
;;th[rad]
(defun stability-margin-in-particular-direction (th &key (robot *robot*) 
                                                    (zmp nil)
                                                    (force-vector-list (list #f(0 0 0) #f(0 0 0)))
                                                    (force-pos-list 
                                                     (mapcar #'(lambda (tmp-arm)
                                                                 (send (send robot tmp-arm :end-coords) :worldpos))
                                                             '(:rarm :larm)))
                                                    )
  (let* (len end-of-dir line-dir convex-edges intersection-points judge-cross intersection-point)
    (when (eq zmp nil)
      (setq zmp (send robot :calc-static-balance-point 
                      force-vector-list force-pos-list
                      )))
    (setq len 10000)
    (setq end-of-dir ;end point of direction vector
          (float-vector (* len (cos th)) (* len (sin th)) 0))
    (setq line-dir ;line from zmp to end-of-dir
          (make-instance line :pvert zmp :nvert (v+ zmp end-of-dir)))
    (setq convex-edges ;get egdes of top plain in convex hull
          (send (cadr (send *foot-convex* :faces)) :edges))    
    (setq intersection-points ;calculate intersections on all edges
          (mapcar #'(lambda (myedge) (send line-dir :intersection myedge))
                  convex-edges))
    (setq judge-cross (mapcar #'(lambda (p) ;find the edge which cross with line-dir
                                  (cond ((< (car p) 0) nil)
                                        ((> (car p) 1) nil)
                                        ((< (cadr p) 0) nil)
                                        ((> (cadr p) 1) nil)
                                        (t t)))
                              intersection-points))
    (setq intersection-point
          (send line-dir :point
                (reduce #'+ (mapcar #'(lambda (j p) 
                                        (if j (car p)  0))
                                    judge-cross intersection-points
                                    ))));get intersection point
    (cond ((eval (cons 'or judge-cross));zmp is in the convex-hull?
           (send (make-instance line :pvert zmp :nvert intersection-point) 
                 :length));get lentgh
          (t nil)
          )
    )
  )



;;get the distance between zmp and the nearest edge to zmp.
;;if there is no input, output will be based on the groud that there is no force applied to robot.
;;if force-vector-list and force-pos-list(the position force applied) are input, zmp will calculated based on the input.
;;if only zmp is input, the output is based on the input zmp without calculating where is zmp.
(defun stability-margin (&key (robot *robot*) 
                              (zmp nil) 
                              (force-vector-list (list #f(0 0 0) #f(0 0 0)))
                              (force-pos-list 
                               (mapcar #'(lambda (tmp-arm)
                                           (send (send robot tmp-arm :end-coords) :worldpos))
                                       '(:rarm :larm)))
                              )
  (let* (len end-of-dir line-dir convex-edges distance-between-zmp-and-each-edge)
    (when (eq zmp nil)
      (dotimes (calc-zmp 3) (send *robot* :calc-zmp))
      (setq zmp0 (send *robot* :get :zmp))
      (setq zmp (send robot :calc-static-balance-point 
                      :force-list force-vector-list :target-points force-pos-list)))
      ;; (setq zmp-diff (v- zmp zmp0))
      ;; (setq zmp (v+ zmp0 (mapvector #'(lambda (x) (* 3 x)) zmp-diff))))
      (setq convex-edges ;get egdes of top plain in convex hull
            (send (send *robot* :support-polygon '(:rleg :lleg)) :edges)
            )
    ;;calculate distance between zmp and each edge of convex hull
    (setq distance-between-zmp-and-each-edge
          (mapcar #'(lambda (myedge) (send myedge :distance zmp))
                  convex-edges))
    ;;check zmp is in convex hull
    ;; (setq lengest
    ;;  (v- (send (send (send *robot* :support-polygon '(:lleg :rleg)) :box) :minpoint)
    ;;      (send (send (send *robot* :support-polygon '(:lleg :rleg)) :box) :maxpoint)
    ;;      ))
    (setq len 10000)
    (setq end-of-dir ;end point of direction vector
          (float-vector len 0 0))
    (setq line-dir ;line from zmp to end-of-dir
          (make-instance line :pvert zmp :nvert (v+ zmp end-of-dir)))
    (setq intersection-points ;calculate intersections on all edges
          (mapcar #'(lambda (myedge) (send line-dir :intersection myedge))
                  convex-edges))
    (setq judge-cross (mapcar #'(lambda (p) ;find the edge which cross with line-dir
                                  (cond ((< (car p) 0) nil)
                                        ((> (car p) 1) nil)
                                        ((< (cadr p) 0) nil)
                                        ((> (cadr p) 1) nil)
                                        (t t)))
                              intersection-points))
    ;;return the stability-margin
    (cond ((eval (cons 'or judge-cross))
           (eval (cons 'min distance-between-zmp-and-each-edge)))
          (t nil)
          )
    ))