(load "~/prog/euslib/rbrain/virtual_interpolator.l")
(load "package://drc_task_common/euslisp/lib/angle-vector-sequence.l")
;; (load "~/prog/euslib/jsk/gnuplotlib.l")

;; (defun setup ()
;;   (require :hrp2jsknts-interface "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
;;   (objects (list (setq *robot* (hrp2jsknts))))
;;   )

(defun calc-zmp-diff-from-two-poses
    (key-pose-av-list key-pose-rc-list key-pose-tm-list
                      &key (dt 0.01) (debug-view nil) (robot *robot*))
  (let* ((data (pos-list-interpolation
                key-pose-av-list key-pose-tm-list dt))
         (av-list (cadr (memq :data data)))
         (tm-list (cadr (memq :time data))))
    ;; Initialize pose
    (send robot :angle-vector (car key-pose-av-list))
    (send robot :move-coords (car key-pose-rc-list) (car (send *robot* :links)))
    (dotimes (i 2) (send robot :calc-zmp
                         (car key-pose-av-list)
                         (car key-pose-rc-list)
                         :pzmpz (elt (send (send robot :foot-midcoords) :worldpos) 2)
                         :dt dt))
    (let ((ret
           (mapcar #'(lambda (av ttm)
                       (send robot :angle-vector av)
                       (send robot :fix-leg-to-coords (make-coords)) ;; TODO fix leg to coords is assumed
                       (send robot :move-centroid-on-foot :both '(:rleg :lleg) :min-loop 2) ;; Mimic AutoBalancer behaviour
                       (if debug-view (send *irtviewer* :draw-objects :flush nil))
                       (let* ((fm (send (send robot :foot-midcoords) :worldpos))
                              (czmp (send robot :calc-zmp
                                          av (send (car (send robot :links)) :copy-worldcoords)
                                          :pzmpz (elt fm 2)
                                          :dt dt)))
                         (when debug-view
                           (send fm :draw-on :flush nil :color #f(0 1 0) :size 200)
                           (send-all (send robot :support-polygons) :draw-on :flush nil)
                           (send czmp :draw-on :flush t :color #f(1 0 0) :size 250))
                         ;; Return value
                         (list czmp fm (send robot :centroid) ttm)))
                   av-list tm-list)))
      (subseq ret 2) ;; Why?
      )))

(defclass zmp-trajectory
  :super propertied-object
  :slots (zmps support-polygon))

(defmethod zmp-trajectory
  (:init (&key ((:zmps z)) ((:support-polygon s)))
    (setq zmps z)
    (setq support-polygon s))
  (:zmps ()
    zmps)
  (:support-polygon ()
    support-polygon)
  )

(defclass zmp-evaluator
  :super propertied-object
  :slots (robot))

(defmethod zmp-evaluator
  (:init (r)
    (setq robot r)
    self)
  (:robot-model () robot)
  (:evaluate-zmp (&key ((:angle-vector-sequence avs))
                           (start-angle-vector)
                           (speed-factor 1.0)
                           (debug-view nil))
    (let* ((rcs (mapcar #'(lambda (av)
                           (send robot :angle-vector av)
                           (send robot :fix-leg-to-coords (make-coords))
                           (send robot :move-centroid-on-foot :both '(:rleg :lleg)) ;need?
                           (send (car (send robot :links)) :copy-worldcoords))
                        (send avs :angle-vectors)))
           (ret (calc-zmp-diff-from-two-poses
                 (send avs :angle-vectors)
                 rcs
                 (cdr (send avs :duration-list robot start-angle-vector speed-factor))
                 :debug-view debug-view
                 :robot robot)))
      (let ((actual (mapcar #'car ret))
            (reference (mapcar #'cadr ret))
            (cog (mapcar #'caddr ret)))
        (send robot :angle-vector start-angle-vector)
        (send robot :fix-leg-to-coords (make-coords))
        (send robot :move-centroid-on-foot :both '(:rleg :lleg)) ;need?
        ;; compute support convex hull
        (let ((p (send self :support-convex-polygon robot)))
          (instance zmp-trajectory :init :zmps actual :support-polygon p)))))
  (:support-convex-polygon (r)
    (let ((lleg (send r :make-sole-polygon :lleg))
          (rleg (send r :make-sole-polygon :rleg)))
      (let ((all-vertices (append (send lleg :vertices) (send rleg :vertices))))
        (instance face :init :vertices (quickhull all-vertices)))))
  )

(defun test-zmp-evaluator ()
  (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
  (unless (boundp '*robot*)
    (setq *robot* (instance hrp2jsknt-robot :init)))
  (objects (list *robot*))
  (let ((avs (instance angle-vector-sequence :init)))
    (send avs :add (send *robot* :reset-pose))
    (send avs :add (send *robot* :reset-manip-pose))
    (send *robot* :larm :move-end-pos (float-vector 300 0 0) :world :rotation-axis nil)
    (send avs :add (send *robot* :angle-vector))
    (send avs :add (send *robot* :reset-pose))
    (let ((zmp (instance zmp-evaluator :init (instance hrp2jsknt-robot :init))))
      (send zmp :evaluate-zmp :angle-vector-sequence avs
            :start-angle-vector (send *robot* :reset-manip-pose)
            :debug-view t)
      )))


(defun test-calc-zmp-diff-from-two-poses-0
  (&optional (tm 0.5))
  ;; Set pose0
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
  (setq *av0* (send *robot* :angle-vector)
        *rc0* (send (car (send *robot* :links)) :copy-worldcoords))
  ;; Set pose1
  (send *robot* :fullbody-inverse-kinematics
        (append (list (make-coords :pos (float-vector 400 0 700)))
                (mapcar #'(lambda (l) (send *robot* l :end-coords)) '(:rleg :lleg)))
        :move-target (mapcar #'(lambda (l) (send *robot* l :end-coords)) '(:rarm :rleg :lleg))
        :link-list (mapcar #'(lambda (l) (send *robot* :link-list (send *robot* l :end-coords :parent))) '(:rarm :rleg :lleg))
        :rotation-axis (list nil t t))
  (setq *av1* (send *robot* :angle-vector)
        *rc1* (send (car (send *robot* :links)) :copy-worldcoords))
  ;; Calculate
  (let ((ret (calc-zmp-diff-from-two-poses
              (list *av0* *av1*)
              (list *rc0* *rc1*)
              (list tm)
              :debug-view t)))
    (graph-view (list (mapcar #'(lambda (x)
                                  (norm (float-vector (elt (car x) 0)
                                                      (elt (car x) 1)))) ret)
                      (mapcar #'(lambda (x) (elt (cadr x) 0)) ret)
                      (mapcar #'(lambda (x) (elt (caddr x) 0)) ret))
                (mapcar #'(lambda (x) (cadddr x)) ret)
                :keylist (list "actual" "reference" "COG")
                :title "ZMP (x)"
                :xlabel "Time [s]"
                :ylabel "ZMP [mm]")
    (apply #'max
           (mapcar #'(lambda (x)
                       (norm (float-vector (elt (car x) 0)
                                           (elt (car x) 1)))) ret))))

(defun test-calc-zmp-diff-from-two-poses-0
  (&optional (tm-rate 1.0))
  ;; Set pose0
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
  (setq *av0* (send *robot* :angle-vector)
        *rc0* (send (car (send *robot* :links)) :copy-worldcoords))
  ;; Set pose1
  (send *robot* :fullbody-inverse-kinematics
        (append (list (make-coords :pos (float-vector 400 0 700)))
                (mapcar #'(lambda (l) (send *robot* l :end-coords)) '(:rleg :lleg)))
        :move-target (mapcar #'(lambda (l) (send *robot* l :end-coords)) '(:rarm :rleg :lleg))
        :link-list (mapcar #'(lambda (l) (send *robot* :link-list (send *robot* l :end-coords :parent))) '(:rarm :rleg :lleg))
        :rotation-axis (list nil t t))
  (setq *av1* (send *robot* :angle-vector)
        *rc1* (send (car (send *robot* :links)) :copy-worldcoords))
  ;; Calculate
  (let ((ret (calc-zmp-diff-from-two-poses
              (list *av0* *av1*)
              (list *rc0* *rc1*)
              (list (* tm-rate (send *robot* :angle-vector-duration *av0* *av1*)))
              :debug-view t)))
    (graph-view (list (mapcar #'(lambda (x)
                                  (norm (float-vector (elt (car x) 0)
                                                      (elt (car x) 1)))) ret)
                      (mapcar #'(lambda (x) (elt (cadr x) 0)) ret)
                      (mapcar #'(lambda (x) (elt (caddr x) 0)) ret))
                (mapcar #'(lambda (x) (cadddr x)) ret)
                :keylist (list "actual" "reference" "COG")
                :title "ZMP (x)"
                :xlabel "Time [s]"
                :ylabel "ZMP [mm]")
    (apply #'max
           (mapcar #'(lambda (x)
                       (norm (float-vector (elt (car x) 0)
                                           (elt (car x) 1)))) ret))))

(defun test-calc-zmp-diff-from-two-poses-2
  ()
  ;; Set pose0
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
  (setq *av0* (send *robot* :angle-vector)
        *rc0* (send (car (send *robot* :links)) :copy-worldcoords))
  ;; Set pose1
  (send *robot* :fullbody-inverse-kinematics
        (append (list (make-coords :pos (float-vector 400 0 700)))
                (mapcar #'(lambda (l) (send *robot* l :end-coords)) '(:rleg :lleg)))
        :move-target (mapcar #'(lambda (l) (send *robot* l :end-coords)) '(:rarm :rleg :lleg))
        :link-list (mapcar #'(lambda (l) (send *robot* :link-list (send *robot* l :end-coords :parent))) '(:rarm :rleg :lleg))
        :rotation-axis (list nil t t))
  (setq *av1* (send *robot* :angle-vector)
        *rc1* (send (car (send *robot* :links)) :copy-worldcoords))
  (send *robot* :reset-manip-pose)
  (send *robot* :fullbody-inverse-kinematics
        (append (list (make-coords :pos (float-vector 400 0 700) :rpy (float-vector 0 pi/2 -pi/2)))
                (mapcar #'(lambda (l) (send *robot* l :end-coords)) '(:rleg :lleg)))
        :move-target (mapcar #'(lambda (l) (send *robot* l :end-coords)) '(:rarm :rleg :lleg))
        :link-list (mapcar #'(lambda (l) (send *robot* :link-list (send *robot* l :end-coords :parent))) '(:rarm :rleg :lleg))
        :rotation-axis (list t t t)
        :centroid-thre 2.5
        :debug-view :no-message)
  (setq *av2* (send *robot* :angle-vector)
        *rc2* (send (car (send *robot* :links)) :copy-worldcoords))
  ;; Calculate
  (let ((tm-list (list 0.1 0.5 1.0 5.0 10.0)))
    (let ((rets (mapcar #'(lambda (tm)
                            (format t "computing ~A~%" tm)
                       (calc-zmp-diff-from-two-poses
                        (list *av0* *av0* *av2* *av2*)
                        (list *rc0* *rc0* *rc2* *rc2*)
                        (list 0.1 tm 0.1)
                        :debug-view t))
                        tm-list)))
      (pprint (mapcar #'(lambda (ret)
                          (list (apply #'max (mapcar #'(lambda (x) (norm (float-vector (elt (car x) 0)
                                                                                       (elt (car x) 1)))) ret))
                                (apply #'max (mapcar #'(lambda (x) (elt (cadr x) 0)) ret))
                                (apply #'max (mapcar #'(lambda (x) (elt (caddr x) 0)) ret))))
                      rets))
      (graph-view (mapcar #'(lambda (ret)
                              (mapcar #'(lambda (x) (norm (float-vector (elt (car x) 0)
                                                      (elt (car x) 1)))) ret)) rets)))))
(defmethod robot-model
  (:angle-vector-duration
    (start end)
    (let* ((joint-list (send self :joint-list)))
      (let ((diff (coerce (v- end start) cons)))
        (let ((time-list (mapcar #'(lambda (d j)
                                     ;; check if j is included in unordered-joint-names
                                     (/ (if (derivedp j linear-joint)
                                            (* 0.001 (abs d))
                                          (deg2rad (abs d)))
                                        (send j :max-joint-velocity)))
                                 diff joint-list)))
          (apply #'max time-list)))))
  )

(defun test-calc-zmp-diff-from-two-poses-1
  ()
  ;; Set pose0
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
  (setq *av0* (send *robot* :angle-vector)
        *rc0* (send (car (send *robot* :links)) :copy-worldcoords))
  ;; Set pose1
  (send *robot* :torso :waist-p :joint-angle 30)
  (send *robot* :fullbody-inverse-kinematics
        (append (list (make-coords :pos (float-vector 400 0 700)))
                (mapcar #'(lambda (l) (send *robot* l :end-coords)) '(:rleg :lleg)))
        :move-target (mapcar #'(lambda (l) (send *robot* l :end-coords)) '(:rarm :rleg :lleg))
        :link-list (mapcar #'(lambda (l) (send *robot* :link-list (send *robot* l :end-coords :parent))) '(:rarm :rleg :lleg))
        :rotation-axis (list nil t t))
  (setq *av1* (send *robot* :angle-vector)
        *rc1* (send (car (send *robot* :links)) :copy-worldcoords))
  ;; Calculate
  (let ((ret (calc-zmp-diff-from-two-poses
              (list *av0* *av0* *av1* *av1*)
              (list *rc0* *rc0* *rc1* *rc1*)
              (list 0.5 0.5 0.5)
              :debug-view t)))
    (graph-view (list (mapcar #'(lambda (x) (elt (car x) 0)) ret)
                      (mapcar #'(lambda (x) (elt (cadr x) 0)) ret)
                      (mapcar #'(lambda (x) (elt (caddr x) 0)) ret))
                (mapcar #'(lambda (x) (cadddr x)) ret)
                :keylist (list "actual" "reference" "COG")
                :title "ZMP (x)"
                :xlabel "Time [s]"
                :ylabel "ZMP [mm]")
    ))

;; (load "package://drc_task_common/euslisp/generate-door-motion.l")
;; (defun get-door-motion* (&rest args)
;;   (setq *door-motion* nil)
;;   (apply #'get-door-motion args)
;;   (if *door-motion*
;;       (ros::ros-warn "IK success!")
;;     (ros::ros-warn "IK failed!"))
;;   *door-motion*)

(defun angle-vector-sequence->angle-vector-sequence-duration (avs)
  (let ((ret nil))
    (dotimes (i (1- (length avs)))
      (let ((next-av (elt avs i))
            (prev-av (if (= i 0) (send *robot* :reset-manip-pose)
                       (elt avs (1- i)))))
        (let ((tm (send *robot* :angle-vector-duration
                        prev-av next-av)))
          (setq ret (append (list (list (elt avs i) tm)) ret))
        ))
      )
    ret))

(defun calc-zmp-diff-from-two-poses* (avs &optional (tm-scale 1.0))
  (let ((rcs (mapcar #'(lambda (av)
                         (send *robot* :angle-vector av)
                         (send *robot* :worldpos)
                         (send (car (send *robot* :links)) :copy-worldcoords))
                     avs)))
    (let ((tms (mapcar #'(lambda (x) (* x tm-scale))
                       (mapcar #'cadr
                               (angle-vector-sequence->angle-vector-sequence-duration avs)))))
      (graph-view* (butlast (calc-zmp-diff-from-two-poses avs rcs tms :debug-view t)))
      )))

(defun graph-view* (ret)
  (graph-view (list (mapcar #'(lambda (x)
                                (norm (float-vector (elt (car x) 0)
                                                    (elt (car x) 1))))
                            ret)
                    (mapcar #'(lambda (x) (elt (cadr x) 0)) ret)
                    (mapcar #'(lambda (x) (elt (caddr x) 0)) ret))
                (mapcar #'(lambda (x) (cadddr x)) ret)
                :keylist (list "actual" "reference" "COG")
                :title "ZMP (x)"
                :xlabel "Time [s]"
                :ylabel "ZMP [mm]")
  (apply #'max (mapcar #'(lambda (x)
                                (norm (float-vector (elt (car x) 0)
                                                    (elt (car x) 1))))
                            ret)))

(defun door-motion ()
  (setq *door-arm* :rarm)
  (setq *door-grasp-preshape-pose* #f(0.0 70.0 0.0 -10.0 30.0 30.0))
  (setq *door-grasp-pose* #f(0.0 -20.0 10.0 -10.0 120.0 120.0))
  (setq *door-stand-coords* (make-coords))
  (setq *robot-name* "HRP2JSKNT")
  (attach-handle-to-robot-for-door 30)
  (setq *rarm-avoid-pose* #f(40.1007 -29.388 4.91744 -71.6494 -22.7883 -2.70865 7.77037 15.0))
  (setq *larm-avoid-pose* #f(40.1007 29.388 -4.91744 -71.6494 22.7883 2.70865 7.77037 15.0))
  (generate-door-from-name :drc-box2)
  (send *door* :locate (float-vector 600 0 0) :world)
  (objects (list *door* *robot*))
  (setq *avs* (get-door-motion* :push/pull :pull :keep-grasp? nil :draw? t :old t))
  (calc-zmp-diff-from-two-poses* *avs*)
  ;; (let ((avs-tm (angle-vector-sequence->angle-vector-sequence-duration *avs*)))
  ;;   )
  )

;; (load "zmp.l")
;; (test-zmp-evaluator)
