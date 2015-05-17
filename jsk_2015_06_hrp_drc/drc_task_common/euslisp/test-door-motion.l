(load "package://drc_task_common/euslisp/generate-door-motion.l")

(ros::roseus "test")

(defun test-door-motion
  (&key (robot-name "HRP2JSKNT")
        (door-name nil)
        (door-width 895)
        (handle-l/r :left)
        (door-knob-size (float-vector 60 110 20))
        (door-knob-pos (float-vector 0 800 845))
        (x-pos 1000) (y-pos 0) (yaw-angle 0)
        (door-coords (make-coords :pos (float-vector x-pos y-pos 0) :rpy (list (deg2rad yaw-angle) 0 0)))
        (stand-coords nil)
        (arm :rarm) (push/pull :push)
        (overwrite-stand-coords? nil) (overwrite-arm-side? nil)
        (ri-test? nil)
        )
  (cond ((not ri-test?)
         (get-door-motion-full
          :door-name door-name
          :door-width door-width :handle-l/r handle-l/r
          :door-knob-size door-knob-size :door-knob-pos door-knob-pos
          :door-coords door-coords :door-coords-system :origin-from-world
          :stand-coords stand-coords
          :arm arm :push/pull push/pull :use-ri? nil
          :robot-name robot-name :draw-environment? t
          :overwrite-stand-coords? overwrite-stand-coords? :overwrite-arm-side? overwrite-arm-side?)
         (play-motion-on-eus-viewer
          *door-motion*
          :interpolation? t :collision-check? t
          :collision-check-robot-link-list (send *robot* *door-arm* :links)
          :collision-check-obj-link (send *door* :link :root-link))
         )
        (t ;; ri-test?
         (unless (boundp '*roscore-checked*)
           (warning-message 1 "this is test function. ROS_MASTER_URI is correct ?~%")
           (y-or-n-p)
           (setq *roscore-checked* t))
         (generate-robot robot-name :generate-ri? t)
         (send *robot* :reset-pose)
         (model2real)
         (dotimes (i 3) (model2real :time 100 :wait? nil)) ;; this is necessary for kinematics simulator bag
         (send *robot* :fix-leg-to-coords (make-coords))
         ;; generate and place door and robot
         (set-door-environment
          :door-name door-name
          :door-width door-width :handle-l/r handle-l/r
          :door-knob-size door-knob-size :door-knob-pos door-knob-pos
          :door-coords door-coords :door-coords-system :origin-from-world
          :arm arm :draw? t
          :use-ri? t :generate-ri? t :robot-name robot-name)
         (let* (door-knob-coords-relative-to-body)
           ;; transfrom door-coords because the knob coords direction of eus model and recognitin result is different
           (setq door-knob-coords-relative-to-body
                 (send (send (send *door* :handle :knob-handle) :rotate (deg2rad -180) :z) :rotate (deg2rad -90) :y))
           ;; transfrom door-coords to become relative to body
           (setq door-knob-coords-relative-to-body
                 (send (send (car (send *robot* :links)) :copy-worldcoords) :transformation door-knob-coords-relative-to-body :local))
           (publish-door-motion
            :door-name door-name
            :door-width door-width :handle-l/r handle-l/r
            :door-knob-size door-knob-size :door-knob-pos door-knob-pos
            :door-coords door-knob-coords-relative-to-body
            :stand-coords stand-coords
            :step-time 0 :test-mode? t
            :arm arm :push/pull push/pull
            :overwrite-stand-coords? overwrite-stand-coords? :overwrite-arm-side? overwrite-arm-side?)
           (send-door-motion
            :door-name door-name
            :door-width door-width :handle-l/r handle-l/r
            :door-knob-size door-knob-size :door-knob-pos door-knob-pos
            :door-coords door-knob-coords-relative-to-body
            :stand-coords stand-coords
            :arm arm :push/pull push/pull
            :overwrite-stand-coords? overwrite-stand-coords? :overwrite-arm-side? overwrite-arm-side?
            :test-mode? t))
         (cond ((check-coords-distance door-coords (send *door* :worldcoords))
                (warning-message 2 "transformed coords is correct. target: ~a  actual:~a~%" door-coords (send *door* :worldcoords)))
               (t
                (error "transformed coords is not correct. target: ~a  actual:~a~%" door-coords (send *door* :worldcoords))))
         )
        )
  )

(defun test-door-motion-without-ri
  ()
  (setq *robot* nil)
  (test-door-motion-hrp2jsknt-pattern1 :ri-test? nil)
  (test-door-motion-hrp2jsknt-pattern2 :ri-test? nil)
  (test-door-motion-hrp2jsknt-pattern3 :ri-test? nil)
  (test-door-motion-hrp2jsknt-pattern4 :ri-test? nil)
  (test-door-motion-hrp2jsknt-pattern5 :ri-test? nil)
  (test-door-motion-hrp2jsknt-pattern6 :ri-test? nil)
  (test-door-motion-hrp2jsknt-pattern7 :ri-test? nil)
  (test-door-motion-hrp2jsknt-pattern8 :ri-test? nil)
  (setq *robot* nil)
  (test-door-motion-hrp2jsknts-pattern1 :ri-test? nil)
  (test-door-motion-hrp2jsknts-pattern2 :ri-test? nil)
  (test-door-motion-hrp2jsknts-pattern3 :ri-test? nil)
  (test-door-motion-hrp2jsknts-pattern4 :ri-test? nil)
  (test-door-motion-hrp2jsknts-pattern5 :ri-test? nil)
  (test-door-motion-hrp2jsknts-pattern6 :ri-test? nil)
  (test-door-motion-hrp2jsknts-pattern7 :ri-test? nil)
  (test-door-motion-hrp2jsknts-pattern8 :ri-test? nil)
  (setq *robot* nil)
  (test-door-motion-jaxon-pattern1 :ri-test? nil)
  (test-door-motion-jaxon-pattern2 :ri-test? nil)
  (test-door-motion-jaxon-pattern3 :ri-test? nil)
  (test-door-motion-jaxon-pattern4 :ri-test? nil)
  (test-door-motion-jaxon-pattern5 :ri-test? nil)
  (test-door-motion-jaxon-pattern6 :ri-test? nil)
  (test-door-motion-jaxon-pattern7 :ri-test? nil)
  (test-door-motion-jaxon-pattern8 :ri-test? nil)
  )

(defun test-door-motion-hrp2jsknt-with-ri
  ()
  (setq *robot* nil)
  (test-door-motion-hrp2jsknt-pattern1 :ri-test? t)
  (test-door-motion-hrp2jsknt-pattern2 :ri-test? t)
  (test-door-motion-hrp2jsknt-pattern3 :ri-test? t)
  (test-door-motion-hrp2jsknt-pattern4 :ri-test? t)
  )

(defun test-door-motion-hrp2jsknt-pattern1
  (&key (ri-test? nil))
  (print "drc-lasvegas-outdoor, test for handle left, force-overwrite arm-side, push, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "HRP2JSKNT" :door-name :drc-lasvegas-outdoor :push/pull :push :overwrite-stand-coords? :force :overwrite-arm-side? :force)
  )

(defun test-door-motion-hrp2jsknt-pattern2
  (&key (ri-test? nil))
  (print "drc-lasvegas-outdoor, test for handle left, larm, pull, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "HRP2JSKNT" :door-name :drc-lasvegas-outdoor :arm :larm :push/pull :pull :overwrite-stand-coords? :force)
  )

(defun test-door-motion-hrp2jsknt-pattern3
  (&key (ri-test? nil))
  (print "drc-lasvegas-indoor, test for handle left, force-overwrite arm-side, push, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "HRP2JSKNT" :door-name :drc-lasvegas-indoor :push/pull :push :overwrite-stand-coords? :force :overwrite-arm-side? :force)
  )

(defun test-door-motion-hrp2jsknt-pattern4
  (&key (ri-test? nil))
  (print "drc-lasvegas-indoor, test for handle left, larm, pull, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "HRP2JSKNT" :door-name :drc-lasvegas-indoor :arm :larm :push/pull :pull :overwrite-stand-coords? :force)
  )

(defun test-door-motion-hrp2jsknt-pattern5
  (&key (ri-test? nil))
  (print "test for handle left, larm, push, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "HRP2JSKNT" :door-name :drc-sagami :arm :larm :push/pull :push :overwrite-stand-coords? :force)
  )

(defun test-door-motion-hrp2jsknt-pattern6
  (&key (ri-test? nil))
  (print "test for handle left, larm, pull, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "HRP2JSKNT" :door-name :drc-sagami :arm :larm :push/pull :pull :overwrite-stand-coords? :force)
  )

(defun test-door-motion-hrp2jsknt-pattern7
  (&key (ri-test? nil))
  (print "test for handle right, rarm, pull, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "HRP2JSKNT" :door-name :drc-sagami-mirror :arm :rarm :push/pull :push :overwrite-stand-coords? :force)
  )

(defun test-door-motion-hrp2jsknt-pattern8
  (&key (ri-test? nil))
  (print "test for handle right, rarm, pull, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "HRP2JSKNT" :door-name :drc-sagami-mirror :arm :rarm :push/pull :pull :overwrite-stand-coords? :force)
  )

(defun test-door-motion-hrp2jsknts-with-ri
  ()
  (setq *robot* nil)
  (test-door-motion-hrp2jsknts-pattern1 :ri-test? t)
  (test-door-motion-hrp2jsknts-pattern2 :ri-test? t)
  (test-door-motion-hrp2jsknts-pattern3 :ri-test? t)
  (test-door-motion-hrp2jsknts-pattern4 :ri-test? t)
  )

(defun test-door-motion-hrp2jsknts-pattern1
  (&key (ri-test? nil))
  (print "drc-lasvegas-outdoor, test for handle left, force-overwrite arm-side, push, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "HRP2JSKNTS" :door-name :drc-lasvegas-outdoor :push/pull :push :overwrite-stand-coords? :force :overwrite-arm-side? :force)
  )

(defun test-door-motion-hrp2jsknts-pattern2
  (&key (ri-test? nil))
  (print "drc-lasvegas-outdoor, test for handle left, larm, pull, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "HRP2JSKNTS" :door-name :drc-lasvegas-outdoor :arm :larm :push/pull :pull :overwrite-stand-coords? :force)
  )

(defun test-door-motion-hrp2jsknts-pattern3
  (&key (ri-test? nil))
  (print "drc-lasvegas-indoor, test for handle left, force-overwrite arm-side, push, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "HRP2JSKNTS" :door-name :drc-lasvegas-indoor :push/pull :push :overwrite-stand-coords? :force :overwrite-arm-side? :force)
  )

(defun test-door-motion-hrp2jsknts-pattern4
  (&key (ri-test? nil))
  (print "drc-lasvegas-indoor, test for handle left, larm, pull, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "HRP2JSKNTS" :door-name :drc-lasvegas-indoor :arm :larm :push/pull :pull :overwrite-stand-coords? :force)
  )

(defun test-door-motion-hrp2jsknts-pattern5
  (&key (ri-test? nil))
  (print "test for handle left, larm, push, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "HRP2JSKNTS" :door-name :drc-sagami :arm :larm :push/pull :push :overwrite-stand-coords? :force)
  )

(defun test-door-motion-hrp2jsknts-pattern6
  (&key (ri-test? nil))
  (print "test for handle left, larm, pull, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "HRP2JSKNTS" :door-name :drc-sagami :arm :larm :push/pull :pull :overwrite-stand-coords? :force)
  )

(defun test-door-motion-hrp2jsknts-pattern7
  (&key (ri-test? nil))
  (print "test for handle right, rarm, pull, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "HRP2JSKNTS" :door-name :drc-sagami-mirror :arm :rarm :push/pull :push :overwrite-stand-coords? :force)
  )

(defun test-door-motion-hrp2jsknts-pattern8
  (&key (ri-test? nil))
  (print "test for handle right, rarm, pull, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "HRP2JSKNTS" :door-name :drc-sagami-mirror :arm :rarm :push/pull :pull :overwrite-stand-coords? :force)
  )

(defun test-door-motion-jaxon-with-ri
  ()
  (setq *robot* nil)
  (test-door-motion-jaxon-pattern1 :ri-test? t)
  (test-door-motion-jaxon-pattern2 :ri-test? t)
  (test-door-motion-jaxon-pattern3 :ri-test? t)
  (test-door-motion-jaxon-pattern4 :ri-test? t)
  )

(defun test-door-motion-jaxon-pattern1
  (&key (ri-test? nil))
  (print "drc-lasvegas-outdoor, test for handle left, force-overwrite arm-side, push, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "JAXON" :door-name :drc-lasvegas-outdoor :push/pull :push :overwrite-stand-coords? :force :overwrite-arm-side? :force)
  )

(defun test-door-motion-jaxon-pattern2
  (&key (ri-test? nil))
  (print "drc-lasvegas-outdoor, test for handle left, rarm, pull, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "JAXON" :door-name :drc-lasvegas-outdoor :arm :rarm :push/pull :pull :overwrite-stand-coords? :force)
  )

(defun test-door-motion-jaxon-pattern3
  (&key (ri-test? nil))
  (print "drc-lasvegas-indoor, test for handle left, force-overwrite arm-side, push, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "JAXON" :door-name :drc-lasvegas-indoor :push/pull :push :overwrite-stand-coords? :force :overwrite-arm-side? :force)
  )

(defun test-door-motion-jaxon-pattern4
  (&key (ri-test? nil))
  (print "drc-lasvegas-indoor, test for handle left, rarm, pull, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "JAXON" :door-name :drc-lasvegas-indoor :arm :rarm :push/pull :pull :overwrite-stand-coords? :force)
  )

(defun test-door-motion-jaxon-pattern5
  (&key (ri-test? nil))
  (print "test for handle left, larm, push, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "JAXON" :door-name :drc-sagami :arm :larm :push/pull :push :overwrite-stand-coords? :force)
  )

(defun test-door-motion-jaxon-pattern6
  (&key (ri-test? nil))
  (print "test for handle left, rarm, pull, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "JAXON" :door-name :drc-sagami :arm :rarm :push/pull :pull :overwrite-stand-coords? :force)
  )

(defun test-door-motion-jaxon-pattern7
  (&key (ri-test? nil))
  (print "test for handle right, rarm, pull, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "JAXON" :door-name :drc-sagami-mirror :arm :rarm :push/pull :push :overwrite-stand-coords? :force)
  )

(defun test-door-motion-jaxon-pattern8
  (&key (ri-test? nil))
  (print "test for handle right, rarm, pull, overwrite stand-coords")
  (test-door-motion :ri-test? ri-test? :robot-name "JAXON" :door-name :drc-sagami-mirror :arm :larm :push/pull :pull :overwrite-stand-coords? :force)
  )


;; misc

(defun do-drcbox2-door-motion-for-jaxon
  (&key (real? nil) (draw? t) (wait? t))
  ;; generate model
  (setup-door-open :generate-robot? t :generate-ri? nil :robot-name "JAXON")
  (make-drc-box2-door)
  (objects (list *robot* *door*))
  (setq *av-list* nil)
  (send *robot* :reset-pose)
  (push (send *robot* :angle-vector) *av-list*)
  (setq *init-standing-coords* (make-coords :pos (float-vector -800 -450 0) :rpy (list (deg2rad -80) 0 0)))
  ;; attach handle to robot
  (let* (larm-door-translate-offset
         rarm-door-translate-offset
         larm-door-z-rotate-offset
         rarm-door-z-rotate-offset)
    (cond ((or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
           (setq larm-door-translate-offset (float-vector 40.0 40.0 0.0))
           (setq rarm-door-translate-offset (float-vector 40.0 -40.0 0.0))
           (setq larm-door-z-rotate-offset (deg2rad 0))
           (setq rarm-door-z-rotate-offset (deg2rad -0))
           )
          (t ;; (or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
           (setq larm-door-translate-offset (float-vector 0 0 0))
           (setq rarm-door-translate-offset (float-vector 0 0 0))
           (setq larm-door-z-rotate-offset 0)
           (setq rarm-door-z-rotate-offset 0)))
    (send *robot* :put :larm-door-coords
          (make-cascoords
           :coords (send (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate larm-door-translate-offset :local) :rotate larm-door-z-rotate-offset :z :local)
           :parent (send (send *robot* :larm :end-coords) :parent)))
    (send *robot* :put :rarm-door-coords
          (make-cascoords
           :coords (send (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate rarm-door-translate-offset :local) :rotate rarm-door-z-rotate-offset :z :local)
           :parent (send (send *robot* :rarm :end-coords) :parent))))
  ;; generate reaching motion
  (let* ((trans-list
          (list (list (float-vector -100 200 -150) -60 0 0)
                (list (float-vector 0 50 -100) -90 0 0)
                (list (float-vector 0 -25 -50) -90 0 0)
                (list (float-vector 0 -25 -50) -90 0 -15)
                (list (float-vector 0 -25 -50) -90 0 -30)
                (list (float-vector 0 -25 -50) -90 -10 -30)
                (list (float-vector 0 -25 -50) -90 -20 -30)
                (list (float-vector 0 0 -50) -90 -20 0)
                (list (float-vector 0 50 -50) -90 -20 0)
                )))
    (dolist (trans trans-list)
      (send *robot* :reset-pose)
      (send *robot* :fix-leg-to-coords *init-standing-coords*)
      (send (send *door* :joint :door-hinge-joint) :joint-angle (elt trans 2))
      (send (send *door* :joint :door-knob-joint) :joint-angle (elt trans 3))
      (with-move-target-link-list
       (mt ll *robot* '(:larm :rleg :lleg))
       (setf (elt mt 0) (send *robot* :get :larm-door-coords))
       (let* ((tc (list (send (send (send (send *door* :handle :larm-knob-handle) :copy-worldcoords)
                                    :translate (elt trans 0) :local)
                              :rotate (deg2rad (elt trans 1)) :z :local)
                        (send *robot* :rleg :end-coords :copy-worldcoords)
                        (send *robot* :lleg :end-coords :copy-worldcoords))))
         (solve-ik-for-do-drcbox2-door-motion-for-jaxon tc mt ll))
       )
      )
    )
  (setq *av-list* (reverse *av-list*))
  ;; send motion to real robot
  (dolist (av *av-list*)
    (send *robot* :angle-vector av)
    (when draw? (send *irtviewer* :draw-objects))
    (when wait? (read-line))
    (when real?
      (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
      (send *ri* :wait-interpolation)))
  ;; (send *robot* :reset-pose)
  ;; (play-motion-on-eus-viewer *av-list* :interpolation? t)
  )

(defun solve-ik-for-do-drcbox2-door-motion-for-jaxon
  (tc mt ll &key (draw? nil))
  (send *robot*
        :fullbody-inverse-kinematics tc
        :move-target mt :link-list ll
        :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
        :translation-axis (list t t t)
        :rotation-axis (list t t t)
        :root-link-virtual-joint-weight (float-vector 0.15 0.15 0.01 0.15 0.5 0.5)
        :additional-weight-list
        (list (list (send *robot* :torso :waist-r :child-link) 0.1)
              (list (send *robot* :torso :waist-y :child-link) 0.1)
              (list (send *robot* :torso :waist-p :child-link) 0.1))
        :max (float-vector  500  200  25  10  20  10)
        :min (float-vector  -500  -200  -200  -10  -20  -10)
        :stop 50
        :debug-view nil
        :avoid-collision-distance 100
        :avoid-collision-null-gain 5.0
        :avoid-collision-joint-gain 0.8
        :collision-avoidance-link-pair
        (list (list (elt (send *robot* :larm :links) 2) (send *robot* :link "CHEST_LINK2")))
        )
  (push (send *robot* :angle-vector) *av-list*)
  )


(warning-message 2 ";; test without ri~%")
(warning-message 2 "(test-door-motion-without-ri)~%")
(warning-message 2 ";; test with ri~%")
(warning-message 2 "(test-door-motion-hrp2jsknt-with-ri)~%")
(warning-message 2 "(test-door-motion-hrp2jsknts-with-ri)~%")
(warning-message 2 "(test-door-motion-jaxon-with-ri)~%")
