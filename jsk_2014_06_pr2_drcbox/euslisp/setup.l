(load "package://pr2eus/pr2-interface.l")
(load "models/drcbox-scene.l")

(defun setup-robot ()
  (unless (boundp '*irtviewer*)
    (setq *irtviewer* (make-irtviewer)))
  (send *irtviewer* :change-background #f(0.4 0.7 0.5))
  (pr2-init)
  (setq *robot* *pr2*)
  (objects (list *robot*)))

(warn "~%(setup-drcbox :base-pos #f(1200 0 0))~%")
(defun setup-drcbox (&key (base-pos #f(1200 0 0)))
  (setup-robot)
  (send *robot* :reset-pose)
  (unless (boundp '*drcbox*) (setq *drcbox* (drcbox)))
  (send *drcbox* :locate base-pos :world)
  (objects (list *robot* *drcbox*)))

(defun send-reach-target
  (move-target-list
   root-relative-target-coords-list ;; list of target-coords ;; target-coords is relative to root
   send-time ;; :angle-vector time [ms]
   alpha ;; ratio from move-target to target-coords in [0.0, 1.0] ;; 0.0 = move-target, 1.0 = target-coords
   &key (robot) (real t))
  (let* ((target-coords-list ;; world target-coords list
          (mapcar #'(lambda (tc)
                      (send (send (car (send robot :links)) :copy-worldcoords) :transform tc))
                  root-relative-target-coords-list))
         (ret
          (send robot :inverse-kinematics
                (mapcar #'(lambda (mt tc)
                            (make-coords :pos (send (midcoords alpha mt tc) :worldpos)
                                         :rot (send tc :worldrot)))
                        move-target-list target-coords-list)
                :move-target move-target-list
                :link-list (mapcar #'(lambda (mt) (send robot :link-list (send mt :parent))) move-target-list)
                :look-at-target t
                :debug-view :no-message
                )))
    (if (and ret real)
        (send *ri* :angle-vector (send robot :angle-vector) send-time))
    ))

(defun send-way-point
  (move-target-list
   root-relative-target-coords-list-trajectory ;; list of (list target-coords-list-0 ... target-coords-list-n)
   send-time-list ;; list of (list time-0 ... time-n)
   &key (robot) (real t))
  (let* ((target-coords-list-trajectory ;; world target-coords list
          (mapcar #'(lambda (tc)
                      (send (send (car (send robot :links)) :copy-worldcoords) :transform tc))
                  root-relative-target-coords-list-trajectory))
         (ret
          (mapcar #'(lambda (target-coords-list)
                      (send robot :inverse-kinematics
                            target-coords-list
                            :move-target move-target-list
                            :link-list (mapcar #'(lambda (mt) (send robot :link-list (send mt :parent))) move-target-list)
                            ))
                  target-coords-list-trajectory)))
    (if (and (every #'identity ret) real)
        (send *ri* :angle-vector-sequence ret send-time-list))
    ))




(warn "(gen-motion-valve-rotation :object \"drcbox-valve-large\" :start -30 :end 30 :interval 10 :time-interval 1000 :alpha 0.8 :real nil)~%")
(defun gen-motion-valve-rotation (&key (object "drcbox-valve-large") (start -30) (end 30) (interval 10) (time-interval 1000) (alpha 0.8) (real nil))
  (labels ((get-target
            (valve deg)
            (send (send valve :joint :crank-joint) :joint-angle deg)
            (send (send valve :handle-valve-handle) :copy-worldcoords))
           (make-deg-list
            (start end interval)
            (let (tmp)
              (do ((i start (+ i interval))) ;only counter clockwise
                  ((> i end) 'done)
                (push i tmp))
              (reverse tmp))))
    (let ((deg-list (append (make-deg-list start end interval)
                            (reverse (make-deg-list start end interval))))
          tc-list tm-list)
      (send-reach-target (list (send *robot* :rarm :end-coords))
                         (list (send (car (send *robot* :links))
                                     :transformation (get-target (send *drcbox* :object object) (car deg-list))))
                         2000
                         alpha
                         :robot *robot*
                         :real real)
      (dolist (i deg-list)
        (push (send (car (send *robot* :links))
                    :transformation (get-target (send *drcbox* :object object) i))
              tc-list)
        (push time-interval tm-list))
      (send-way-point (list (send *robot* :rarm :end-coords))
                      (reverse tc-list)
                      (reverse tm-list)
                      :robot *robot*
                      :real real))))
