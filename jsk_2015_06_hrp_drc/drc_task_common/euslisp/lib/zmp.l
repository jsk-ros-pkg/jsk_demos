(load "package://drc_task_common/euslisp/lib/angle-vector-sequence.l")
;;(load "~/ros/hydro_parent/src/jskeus/irteus/irtdyna.l")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(load "package://drc_task_common/euslisp/lib/table.l")
(load "package://drc_task_common/euslisp/lib/util.l")
(load "package://drc_task_common/euslisp/lib/plotlib.l")
(load "package://drc_task_common/euslisp/lib/periodic-rate.l")
(load "package://drc_task_common/euslisp/lib/end-effector.l")

(defun calc-zmp-diff-from-two-poses
    (original-key-pose-av-list
     original-key-pose-rc-list
     original-key-pose-tm-list
     &key (dt 0.1) (debug-view nil) (robot *robot*))
  ;; (pprint `(calc-zmp-diff-from-two-poses ,key-pose-av-list
  ;;                                        ,(mapcar #'(lambda (c)
  ;;                                                     `(make-coords :pos ,(send c :worldpos)
  ;;                                                                   :rot ,(send c :worldrot)))
  ;;                                                 key-pose-rc-list)
  ;;                                        ,key-pose-tm-list))
  ;; add 1 second offset
  (let ((key-pose-av-list (cons (car original-key-pose-av-list) original-key-pose-av-list))
        (key-pose-rc-list (cons (car original-key-pose-rc-list) original-key-pose-rc-list))
        (key-pose-tm-list (cons 0 (mapcar #'(lambda (tm) (+ 1.0 tm)) original-key-pose-tm-list))))
    (let* ((data (pos-list-interpolation
                  key-pose-av-list key-pose-tm-list dt :interpolator-class minjerk-interpolator))
           (av-list (cadr (memq :data data)))
           (tm-list (cadr (memq :time data))))
      (send robot :angle-vector (car key-pose-av-list))
      (send robot :move-coords (car key-pose-rc-list) (car (send *robot* :links)))
      (dotimes (i 2)
        (send robot :calc-zmp
              (car key-pose-av-list)
              (car key-pose-rc-list)
              :pzmpz (elt (send (send robot :foot-midcoords) :worldpos) 2)
              :dt dt)
        )
      (let ((ret
             (mapcar #'(lambda (av ttm)
                         (send robot :angle-vector av)
                         (send robot :fix-leg-to-coords (make-coords)) ;; TODO fix leg to coords is assumed
                         ;;(send robot :move-centroid-on-foot :both '(:rleg :lleg) :min-loop 3) ;; Mimic AutoBalancer behaviour
                         (if debug-view (send *irtviewer* :draw-objects :flush nil))
                         (let* ((fm (send (send robot :foot-midcoords) :worldpos))
                                (czmp (send robot :calc-zmp
                                            av (send (car (send robot :links)) :copy-worldcoords)
                                            :pzmpz (elt fm 2)
                                            :dt dt)))
                           (when debug-view
                             (send fm :draw-on :flush nil :color #f(0 1 0) :size 200)
                             (send-all (send robot :support-polygons) :draw-on :flush nil)
                             (send czmp :draw-on :flush t :color #f(1 0 0) :size 250)
                             )
                           ;; Return value
                           (list czmp fm (send robot :centroid) ttm av)))
                     av-list tm-list)))
        ;; remove first 1 second
        (mapcar #'(lambda (czmp-fm-centroid-ttm-av)
                    (list (elt czmp-fm-centroid-ttm-av 0)
                          (elt czmp-fm-centroid-ttm-av 1)
                          (elt czmp-fm-centroid-ttm-av 2)
                          (- (elt czmp-fm-centroid-ttm-av 3) 1.0)
                          (elt czmp-fm-centroid-ttm-av 4)))
                (remove-if #'(lambda (czmp-fm-centroid-ttm-av)
                               (< (elt czmp-fm-centroid-ttm-av 3) 1.0))
                           ret))))))


(defclass zmp-trajectory
  :super propertied-object
  :slots (zmps tms support-polygon angle-vectors no-ros))

(defmethod zmp-trajectory
  (:init (&key ((:zmps z)) ((:support-polygon s)) ((:tms atms)) ((:angle-vectors avs)) ((:no-ros nrs) nil))
    (setq no-ros nrs)
    (setq zmps z)
    (setq angle-vectors avs)
    (setq tms atms)
    (setq support-polygon s)
    ;; advertise
    (unless no-ros
      (ros::advertise "/zmp_plot" jsk_recognition_msgs::PlotDataArray)
      (ros::advertise "/zmp_x" jsk_recognition_msgs::PlotData)
      (ros::advertise "/zmp_y" jsk_recognition_msgs::PlotData)
      (ros::advertise "/zmp_distance" jsk_recognition_msgs::PlotData))
    )
  (:dump-to-csv (fname)
    (with-open-file (f fname :direction :output)
      (format f "time,zmp_x,zmp_y,~A~%" (string-join "," (mapcar #'(lambda (i)
                                                                     (format nil "angle_~A" i))
                                                                 (range (length (car angle-vectors))))))
      (let ((prev-tm 0))
        (dotimes (i (length tms))
          (format f "~A,~A,~A,~A~%"
                  (+ prev-tm (elt tms i))
                  (elt (elt zmps i) 0)
                  (elt (elt zmps i) 1)
                  (string-join "," (mapcar #'string
                                           (coerce (elt angle-vectors i) cons))))
          ))
      )
    )
  (:zmps ()
    zmps)
  (:support-polygon ()
    support-polygon)
  (:publish-zmp-plot ()
    (unless no-ros
      (ros::publish "/zmp_plot"
        (multi-plot-data (list (list tms (mapcar #'(lambda (x) (elt x 0)) zmps) :label "zmp(x)")
                               (list tms (mapcar #'(lambda (x) (elt x 1)) zmps) :label "zmp(y)")
                               (list tms (send self :all-distance-from-edge) :label "zmp distance"))
                         :legend-font-size 20 ))
      (ros::publish
        "/zmp_x" (plot-data tms (mapcar #'(lambda (x) (elt x 0)) zmps)))
      (ros::publish
        "/zmp_y" (plot-data tms (mapcar #'(lambda (x) (elt x 1)) zmps)))
      
      (ros::publish
        "/zmp_distance"
        (plot-data tms (send self :all-distance-from-edge))))
    )
  (:all-distance-from-edge ()
    (mapcar #'(lambda (z)
                (send self :distance-from-edge z))
            zmps))
  (:distance-from-edge (zmp-point)
    (let ((edges (send support-polygon :edges)))
      (apply #'min (mapcar #'(lambda (e)
                               (send e :distance zmp-point))
                           edges))))
  (:animate (robot)
    (require "models/arrow-object")
    (let ((arrow (arrow-object 1.0)))
      (send *irtviewer* :objects (cons arrow (send *irtviewer* :objects)))
      (dotimes (i (length angle-vectors))
        (send arrow :locate (elt zmps i) :world)
        (send robot :angle-vector (elt angle-vectors i))
        (send robot :fix-leg-to-coords (make-coords))
        (send *irtviewer* :draw-objects :flush nil)
        (send support-polygon :draw-on :flush t)
        (x::window-main-one)
        )
      ;; remove arrow from objects
      (let ((objs (send *irtviewer* :objects)))
        (send *irtviewer* :objects (remove arrow objs))
        ))
    )
  (:evaluate ()
    ;; return 0-1 value
    ;; 0 --> unreliable
    ;; 1 --> reliable
    (let ((max-distance (send self :distance-from-edge (float-vector 0 0 0))))
      (let ((min-value (apply #'min (send self :all-distance-from-edge))))
        ;; min-value = 0 --> unreliable
        ;; min-value = max-distance --> reliable
        (/ min-value max-distance))))
  (:evaluate-full ()
    (let ((max-distance (send self :distance-from-edge (float-vector 0 0 0))))
      (let ((min-value (apply #'min (send self :all-distance-from-edge))))
        ;; min-value = 0 --> unreliable
        ;; min-value = max-distance --> reliable
        (list max-distance min-value (- 1.0 (min 1.0 (/ min-value max-distance)))))))
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
                       (dt 0.01)
                       (no-ros nil)
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
                 (cdr (send avs :duration-list robot
                            start-angle-vector speed-factor))
                 ;; (send avs :duration-list robot
                 ;;       start-angle-vector speed-factor)
                 :debug-view debug-view
                 :dt dt
                 :robot robot)))
      (let ((actual (mapcar #'car ret))
            (reference (mapcar #'cadr ret))
            (cog (mapcar #'caddr ret)))
        (send robot :angle-vector start-angle-vector)
        (send robot :fix-leg-to-coords (make-coords))
        (send robot :move-centroid-on-foot :both '(:rleg :lleg)) ;need?
        ;; compute support convex hull
        (let ((p (send self :support-convex-polygon robot)))
          (instance zmp-trajectory :init :zmps actual :support-polygon p
                    :tms (mapcar #'cadddr ret)
                    :no-ros no-ros
                    :angle-vectors (mapcar #'caddddr ret))))))
  (:support-convex-polygon (r)
    (let ((lleg (send r :make-sole-polygon :lleg))
          (rleg (send r :make-sole-polygon :rleg)))
      (let ((all-vertices (append (send lleg :vertices) (send rleg :vertices))))
        (instance face :init :vertices (quickhull all-vertices)))))
  (:profile (&key ((:angle-vector-sequence avs))
                  (start-angle-vector)
                  (max-speed-factor 10.0)
                  (dt 0.01)
                  (debug-view nil))
    (let ((tbl (instance table :init '("speed-factor" "time" "max-zmp-distance" "min-zmp-distance" "e"))))
      (let ((speed-factor 1.0))
        (while (< speed-factor max-speed-factor)
          (format t "testing speed-factor=~A~%" speed-factor)
          (let ((zmp-traj (send self :evaluate-zmp :angle-vector-sequence avs
                                :start-angle-vector start-angle-vector
                                :speed-factor speed-factor
                                :dt dt)))
            (multiple-value-bind
             (max-distance min-value e) (send zmp-traj :evaluate-full)
             (send tbl :add (list speed-factor (send avs :all-duration robot speed-factor :start-angle-vector start-angle-vector)
                                  max-distance min-value e))
             )
            )
          (setq speed-factor (+ speed-factor 0.1)))
        )
      tbl))
  )

(defun test-zmp-motion (&optional (dt 0.01))
  (ros::roseus "test_zmp_evaluator")
  (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
  (unless (boundp '*robot*)
    (setq *robot* (instance hrp2jsknt-robot :init)))
  (objects (list *robot*))
  (let ((avs (instance angle-vector-sequence :init)))
    ;; (send *robot* :reset-pose)
    ;; (send avs :add (send *robot* :angle-vector))
    (send *robot* :reset-manip-pose)
    (send avs :add (send *robot* :angle-vector))
    (send *robot* :fullbody-inverse-kinematics
        (append (list (make-coords :pos (float-vector 400 0 700)))
                (mapcar #'(lambda (l) (send *robot* l :end-coords)) '(:rleg :lleg)))
        :move-target (mapcar #'(lambda (l) (send *robot* l :end-coords)) '(:rarm :rleg :lleg))
        :link-list (mapcar #'(lambda (l) (send *robot* :link-list (send *robot* l :end-coords :parent))) '(:rarm :rleg :lleg))
        :rotation-axis (list nil t t))
    (send avs :add (send *robot* :angle-vector))
    (let ((zmp-robot (instance hrp2jsknt-robot :init)))
      (let ((zmp (instance zmp-evaluator :init zmp-robot)))
        (objects (list zmp-robot))
        (let ((zmp-table (send zmp :profile :debug-view t :angle-vector-sequence avs :start-angle-vector (send *robot* :reset-pose))))
          (send zmp-table :write "/tmp/zmp.csv")
          (send (instance end-effector-evaluator :init *robot* :rarm) :profile avs :save-full-log t :log-name "ee"
                :dspeed-factor 0.1
                :max-speed-factor 10.0)
          )))))

(defun test-zmp-motion-jaxon (&optional (dt 0.01))
  (ros::roseus "test_zmp_evaluator")
  (require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
  (unless (boundp '*robot*)
    (setq *robot* (instance jaxon_red-robot :init)))
  (unless (boundp '*zmp-robot*)
    (setq *zmp-robot* (instance jaxon_red-robot :init)))
  (objects (list *robot*))
  (let ((avs (instance angle-vector-sequence :init)))
    ;; (send *robot* :reset-pose)
    ;; (send avs :add (send *robot* :angle-vector))
    (send *robot* :reset-manip-pose)
    (send avs :add (send *robot* :angle-vector))
    (send *robot* :fullbody-inverse-kinematics
        (append (list (make-coords :pos (float-vector 400 0 700)))
                (mapcar #'(lambda (l) (send *robot* l :end-coords)) '(:rleg :lleg)))
        :move-target (mapcar #'(lambda (l) (send *robot* l :end-coords)) '(:rarm :rleg :lleg))
        :link-list (mapcar #'(lambda (l) (send *robot* :link-list (send *robot* l :end-coords :parent))) '(:rarm :rleg :lleg))
        :rotation-axis (list nil t t))
    (send avs :add (send *robot* :angle-vector))
    ;; (unless (boundp '*zmp-table*)
    ;;   (let ((zmp-robot (instance jaxon_red-robot :init)))
    ;;     (let ((zmp (instance zmp-evaluator :init *zmp-robot*)))
    ;;       (let ((zmp-table
    ;;              (normalize-zmp-table (send zmp :profile :debug-view t :angle-vector-sequence avs :start-angle-vector (send *robot* :reset-pose)))))
    ;;         ;; normalize zmp-table
    ;;         (send zmp-table :write "zmp.csv")
    ;;         (setq *zmp-table* zmp-table)
    ;;         ))))
    (setq *zmp-table* (load-table "zmp.csv"))
    ;; ee-table is done
    (unless (boundp '*ee-table*)
      (setq *ee-table*
            (send (instance end-effector-evaluator :init *robot* :rarm) :profile avs :save-full-log t :log-name "ee"
                  :dspeed-factor 0.1
                  :max-speed-factor 10.0)))
    (setq *ee-table* (load-table "JAXON_RED_ee_ee.csv"))
    (setq *qe-table* (normalize-execution-table (merge-ee-zmp-table *ee-table* *zmp-table*)))
    (send *qe-table* :write "qe.csv")
    ))

(defun test-zmp-evaluator (&optional (dt 0.01))
  (ros::roseus "test_zmp_evaluator")
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
    (let ((zmp-robot (instance hrp2jsknt-robot :init)))
      (let ((zmp (instance zmp-evaluator :init zmp-robot)))
        (objects (list zmp-robot))
        (let ((zmp-traj (send zmp :evaluate-zmp :angle-vector-sequence avs
                              :start-angle-vector (send *robot* :reset-manip-pose)
                              :dt dt
                              :speed-factor 10.0
                              :debug-view nil)))
          (send zmp-traj :publish-zmp-plot)
          zmp-traj)))))

;; (load "zmp.l")
;; (setq hoge (test-zmp-evaluator))

;; fix zmp
;; (let ((tbl (instance table :init (send aa :column)))) (dolist (d (send aa :data)) (send tbl :add (list (elt d 0) (elt d 1) (elt d 2) (elt d 3) (+ 1 (elt d 4))))) tbl)
;; e - 1 -> e