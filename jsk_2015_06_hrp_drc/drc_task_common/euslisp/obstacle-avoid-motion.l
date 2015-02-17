(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(load "package://drc_task_common/euslisp/robot-util.l")

(defun set-test-environment
  (&key (generate-ri? t) (draw? t))
  ;; generate robot
  (unless (boundp '*robot*)
    (cond (generate-ri?
           (hrp2jsknts-init)
           (setq *robot* *hrp2jsknts*))
          (t
           (setq *robot* (instance hrp2jsknts-robot :init)))))
  ;; place robot to origin
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  ;; initialize obstacle
  (setq *obstacle-list* nil)
  (let* ((obstacle1 (make-cube 100 1000 100)))
    (send obstacle1 :set-color #f(1 1 0))
    (send obstacle1 :locate (float-vector 200 0 750))
    (push obstacle1 *obstacle-list*))
  ;; draw
  (when draw?
    (objects (append (list *robot*) *obstacle-list*))
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    )
  )

(defun test
  ()
  (let* (start-av goal-av)
    (send *robot* :reset-pose)
    (setq start-av (send *robot* :angle-vector))
    (send *robot* :larm :angle-vector #f(50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0))
    (setq goal-av (send *robot* :angle-vector))
    (check-trajectory-obstacle :start-av start-av :goal-av goal-av)
    ))

(defun check-trajectory-obstacle
  (&key start-av goal-av (limb :larm))
  (let* ((devide-num 10))
    ;; set default angle-vector of start and goal
    (unless goal-av
      (setq goal-av (copy-seq (send *robot* :angle-vector))))
    (unless start-av
      (setq start-av (copy-seq (send *robot* :reset-pose))))
    ;; check start angle-vector
    (send *robot* :angle-vector start-av)
    (when (check-robot-collision)
      (warning-message 1 "[obst-avoid] start angle-vector is collision pose.~%")
      (send *robot* :angle-vector goal-av)
      (return-from check-trajectory-obstacle nil nil))
    ;; check trajectory
    (let* ((trajectory-coords-collison (get-trajectory-coords-with-collision start-av goal-av :limb limb :devide-num devide-num))
           (trajectory-coords (elt trajectory-coords-collison 0))
           (trajectory-collision (elt trajectory-coords-collison 1)))
      (dotimes (i devide-num)
        (if (elt trajectory-collision i)
            (send (elt trajectory-coords i) :draw-on :flush t :size 20 :width 5 :color #f(1 0 0))
          (send (elt trajectory-coords i) :draw-on :flush t :size 20 :width 5 :color #f(0 1 0))
          ))
      (setq *trajectory-coords* trajectory-coords)
      (setq *trajectory-collision* trajectory-collision)
      )))

(defun get-trajectory-coords-with-collision
  (start-av goal-av &key (limb :larm) (devide-num 10))
  (let* (trajectory-coords
         trajectory-collision)
    (dotimes (i devide-num)
      (let* ((r (/ (float (1+ i)) devide-num)))
        (send *robot* :angle-vector (midpoint r start-av goal-av))
        (send *irtviewer* :draw-objects)
        (push (send (send *robot* limb :end-coords) :copy-worldcoords)
              trajectory-coords)
        ;; (push (mapcar #'(lambda (limb) (send (send *robot* limb :end-coords) :copy-worldcoords)) (list :larm :rarm :lleg :rleg))
        ;;       trajectory-coords)
        (push (check-robot-collision) trajectory-collision)
        ))
    (list trajectory-coords trajectory-collision)
    ))

(defun check-robot-collision
  (&key (robot *robot*) (obj *obstacle-list*) (link-list (send robot :links))
        (draw? t) (bool? t) (hand? :root))
  (cond ((listp obj)
         (let* (res-all)
           (dolist (obj *obstacle-list*)
             (let* ((res
                     (check-robot-collision :robot robot :obj obj :link-list link-list
                                            :draw? draw? :bool? bool? :hand? hand?)))
               (when (and res bool?) (setq res-all t) (return))
               (push res res-all)
               ))
           (if (equal res-all (list nil)) nil res-all)
           ))
        (t
         (let* (res)
           (cond
            ((equal hand? :root)
             (setq link-list (append link-list (flatten (mapcar #'car (send-all (send robot :hand :arms) :links))))))
            (hand?
             (setq link-list (append link-list (flatten (send-all (send robot :hand :arms) :links)))))
            )
           (dolist (l link-list)
             (let* ((coll-res (pqp-collision-distance l obj)))
               (when (<= (car coll-res) 0)
                 (when draw? (send (elt coll-res 1) :draw-on :flush t :size 50 :width 5 :color #f(1 0 0)))
                 (when bool? (setq res t) (return))
                 (push (list l (elt coll-res 1)) res))))
           res)))
  )

