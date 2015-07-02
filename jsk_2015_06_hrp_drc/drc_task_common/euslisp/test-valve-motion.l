(load "package://drc_task_common/euslisp/generate-valve-motion.l")

(ros::roseus "test")

(defun test-valve-motion
  (&key (robot-name "HRP2JSKNT")
        (valve-r (/ 360 2))
        (x-pos 500) (y-pos -120) (z-pos 1260) (yaw-angle 0)
        (valve-coords (make-coords :pos (float-vector x-pos y-pos z-pos) :rpy (list yaw-angle pi/2 0)))
        (stand-coords nil)
        (grasp-center? nil) (arm :rarm)
        (overwrite-stand-coords? nil) (overwrite-arm-side? nil)
        (ri-test? nil)
        )
  (cond ((not ri-test?)
         (get-valve-motion-full
          valve-r valve-coords
          :stand-coords stand-coords
          :valve-coords-system :world :arm arm :use-ri? nil :robot-name robot-name :draw-environment? t
          :grasp-center? grasp-center?
          :overwrite-stand-coords? overwrite-stand-coords? :overwrite-arm-side? overwrite-arm-side?)
         (play-motion-on-eus-viewer *valve-motion* :interpolation? t)
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
         (let* ((valve-coords-relative-to-body
                 (send (send (car (send *robot* :links)) :copy-worldcoords) :transformation valve-coords :local)))
           (publish-valve-motion
            valve-r valve-coords-relative-to-body
            :stand-coords stand-coords
            :step-time 0 :test-mode? t
            :arm arm :grasp-center? grasp-center?
            :overwrite-stand-coords? overwrite-stand-coords? :overwrite-arm-side? overwrite-arm-side?)
           (send-valve-motion
            valve-r valve-coords-relative-to-body
            :stand-coords stand-coords
            :arm arm :grasp-center? grasp-center?
            :overwrite-stand-coords? overwrite-stand-coords? :overwrite-arm-side? overwrite-arm-side?
            :test-mode? t))
         (cond ((check-coords-distance valve-coords (send *valve* :worldcoords))
                (warning-message 2 "transformed coords is correct. target: ~a  actual:~a~%" valve-coords (send *valve* :worldcoords)))
               (t
                (error "transformed coords is not correct. target: ~a  actual:~a~%" valve-coords (send *valve* :worldcoords))))
         )
        )
  )

(defun test-valve-motion-without-ri
  ()
  (setq *robot* nil)
  (test-valve-motion-hrp2jsknt-lasvegas :ri-test? nil)
  (test-valve-motion-hrp2jsknt-pattern1 :ri-test? nil)
  (test-valve-motion-hrp2jsknt-pattern2 :ri-test? nil)
  (test-valve-motion-hrp2jsknt-pattern3 :ri-test? nil)
  (setq *robot* nil)
  (test-valve-motion-hrp2jsknts-lasvegas :ri-test? nil)
  (test-valve-motion-hrp2jsknts-pattern4 :ri-test? nil)
  (test-valve-motion-hrp2jsknts-pattern5 :ri-test? nil)
  (test-valve-motion-hrp2jsknts-pattern6 :ri-test? nil)
  (setq *robot* nil)
  (test-valve-motion-jaxon-lasvegas :ri-test? nil)
  (test-valve-motion-jaxon-pattern1 :ri-test? nil)
  (test-valve-motion-jaxon-pattern2 :ri-test? nil)
  (test-valve-motion-jaxon-pattern3 :ri-test? nil)
  (test-valve-motion-jaxon-pattern4 :ri-test? nil)
  (setq *robot* nil)
  (test-valve-motion-staro-lasvegas :ri-test? nil)
  (test-valve-motion-staro-pattern1 :ri-test? nil)
  (test-valve-motion-staro-pattern2 :ri-test? nil)
  (test-valve-motion-staro-pattern3 :ri-test? nil)
  (test-valve-motion-staro-pattern4 :ri-test? nil)
  )

(defun test-valve-motion-hrp2jsknt-with-ri
  ()
  (setq *robot* nil)
  (test-valve-motion-hrp2jsknt-pattern2 :ri-test? t)
  (test-valve-motion-hrp2jsknt-pattern4 :ri-test? t)
  (test-valve-motion-hrp2jsknt-pattern5 :ri-test? t)
  )

(defun test-valve-motion-hrp2jsknt-pattern1
  (&key (ri-test? nil))
  (print "rarm test")
  (test-valve-motion :ri-test? ri-test? :robot-name "HRP2JSKNT" :x-pos 500 :y-pos -120 :z-pos 1260 :arm :rarm)
  )

(defun test-valve-motion-hrp2jsknt-pattern2
  (&key (ri-test? nil))
  (print "larm test")
  (test-valve-motion :ri-test? ri-test? :robot-name "HRP2JSKNT" :x-pos 500 :y-pos 120 :z-pos 1260 :arm :larm)
  )

(defun test-valve-motion-hrp2jsknt-pattern3
  (&key (ri-test? nil))
  (print "overwrite arm-side test")
  (test-valve-motion :ri-test? ri-test? :robot-name "HRP2JSKNT" :x-pos 500 :y-pos 120 :z-pos 1260 :arm :rarm :overwrite-arm-side? t)
  )
(defun test-valve-motion-hrp2jsknt-pattern4
  (&key (ri-test? nil))
  (print "overwrite stand-coords test")
  (test-valve-motion :ri-test? ri-test? :robot-name "HRP2JSKNT" :x-pos 500 :y-pos 120 :z-pos 1260 :arm :rarm :overwrite-stand-coords? t)
  )

(defun test-valve-motion-hrp2jsknt-pattern5
  (&key (ri-test? nil))
  (print "grasp center test")
  (test-valve-motion :ri-test? ri-test? :robot-name "HRP2JSKNT" :x-pos 400 :y-pos -500 :z-pos 1260 :arm :rarm :grasp-center? t)
  )

(defun test-valve-motion-hrp2jsknt-pattern6
  (&key (ri-test? nil))
  (print "set stand-coords test")
  (test-valve-motion :ri-test? ri-test? :robot-name "HRP2JSKNT" :x-pos 1500 :y-pos 1120 :z-pos 1260 :stand-coords (make-coords :pos (float-vector 1000 1000 0)) :overwrite-arm-side? t :arm :rarm)
  )

(defun test-valve-motion-hrp2jsknt-lasvegas
  (&key (ri-test? nil))
  (print "drc environment in las vegas")
  (test-valve-motion :ri-test? ri-test? :robot-name "HRP2JSKNT" :valve-r 130 :x-pos 500 :y-pos -120 :z-pos 1200 :overwrite-stand-coords? :force)
  )

(defun test-valve-motion-hrp2jsknts-with-ri
  ()
  (setq *robot* nil)
  (test-valve-motion-hrp2jsknts-pattern2 :ri-test? t)
  (test-valve-motion-hrp2jsknts-pattern4 :ri-test? t)
  (test-valve-motion-hrp2jsknts-pattern5 :ri-test? t)
  )

(defun test-valve-motion-hrp2jsknts-pattern1
  (&key (ri-test? nil))
  (print "rarm test")
  (test-valve-motion :ri-test? ri-test? :robot-name "HRP2JSKNTS" :x-pos 500 :y-pos -120 :z-pos 1260 :arm :rarm)
  )

(defun test-valve-motion-hrp2jsknts-pattern2
  (&key (ri-test? nil))
  (print "larm test")
  (test-valve-motion :ri-test? ri-test? :robot-name "HRP2JSKNTS" :x-pos 500 :y-pos 120 :z-pos 1260 :arm :larm)
  )

(defun test-valve-motion-hrp2jsknts-pattern3
  (&key (ri-test? nil))
  (print "overwrite arm-side test")
  (test-valve-motion :ri-test? ri-test? :robot-name "HRP2JSKNTS" :x-pos 500 :y-pos 120 :z-pos 1260 :arm :rarm :overwrite-arm-side? t)
  )
(defun test-valve-motion-hrp2jsknts-pattern4
  (&key (ri-test? nil))
  (print "overwrite stand-coords test")
  (test-valve-motion :ri-test? ri-test? :robot-name "HRP2JSKNTS" :x-pos 500 :y-pos 120 :z-pos 1260 :arm :rarm :overwrite-stand-coords? t)
  )

(defun test-valve-motion-hrp2jsknts-pattern5
  (&key (ri-test? nil))
  (print "grasp center test")
  (test-valve-motion :ri-test? ri-test? :robot-name "HRP2JSKNTS" :x-pos 400 :y-pos -500 :z-pos 1260 :arm :rarm :grasp-center? t)
  )

(defun test-valve-motion-hrp2jsknts-pattern6
  (&key (ri-test? nil))
  (print "set stand-coords")
  (test-valve-motion :ri-test? ri-test? :robot-name "HRP2JSKNTS" :x-pos 1500 :y-pos 1120 :z-pos 1260 :stand-coords (make-coords :pos (float-vector 1000 1000 0)) :overwrite-arm-side? t :arm :rarm)
  )

(defun test-valve-motion-hrp2jsknts-lasvegas
  (&key (ri-test? nil))
  (print "drc environment in las vegas")
  (test-valve-motion :ri-test? ri-test? :robot-name "HRP2JSKNTS" :valve-r 130 :x-pos 500 :y-pos -120 :z-pos 1200 :overwrite-stand-coords? :force)
  )

(defun test-valve-motion-jaxon-with-ri
  ()
  (setq *robot* nil)
  (test-valve-motion-jaxon-pattern1 :ri-test? t)
  (test-valve-motion-jaxon-pattern2 :ri-test? t)
  (test-valve-motion-jaxon-pattern3 :ri-test? t)
  )

(defun test-valve-motion-jaxon-pattern1 ;; for continuous test
  (&key (ri-test? nil))
  (print "grasp edge test")
  (test-valve-motion :ri-test? ri-test? :robot-name "JAXON" :yaw-angle (deg2rad 20))
  ;; (2 1 0 0 4 3 2 1 4 3)
  )

(defun test-valve-motion-jaxon-pattern2 ;; for continuous test
  (&key (ri-test? nil))
  (print "grasp center test")
  (test-valve-motion :ri-test? ri-test? :robot-name "JAXON" :x-pos 420 :y-pos -600 :z-pos 1240 :yaw-angle (deg2rad 20) :grasp-center? t)
  ;; (0 0 6 5 4 3 2 1 1 0)
  )

(defun test-valve-motion-jaxon-pattern3
  (&key (ri-test? nil))
  (print "overwrite arm-side test")
  (test-valve-motion :ri-test? ri-test? :robot-name "JAXON" :x-pos 420 :y-pos 600 :z-pos 1240 :overwrite-arm-side? t)
  )

(defun test-valve-motion-jaxon-pattern4
  (&key (ri-test? nil))
  (print "overwrite stand-coords test")
  (test-valve-motion :ri-test? ri-test? :robot-name "JAXON" :x-pos 420 :y-pos 600 :z-pos 1240 :overwrite-stand-coords? t)
  )

(defun test-valve-motion-jaxon-lasvegas
  (&key (ri-test? nil))
  (print "drc environment in las vegas")
  (test-valve-motion :ri-test? ri-test? :robot-name "JAXON" :valve-r 130 :x-pos 600 :y-pos -500 :z-pos 1200 :overwrite-stand-coords? :force)
  )

(defun test-valve-motion-staro-with-ri
  ()
  (setq *robot* nil)
  (test-valve-motion-staro-pattern1 :ri-test? t)
  (test-valve-motion-staro-pattern2 :ri-test? t)
  (test-valve-motion-staro-pattern3 :ri-test? t)
  )

(defun test-valve-motion-staro-pattern1 ;; for continuous test
  (&key (ri-test? nil))
  (print "grasp edge test")
  (test-valve-motion :ri-test? ri-test? :robot-name "STARO" :x-pos 600 :y-pos -500 :yaw-angle (deg2rad 20))
  ;; (2 1 0 0 4 3 2 1 4 3)
  )

(defun test-valve-motion-staro-pattern2 ;; for continuous test
  (&key (ri-test? nil))
  (print "grasp center test")
  (test-valve-motion :ri-test? ri-test? :robot-name "STARO" :x-pos 600 :y-pos -600 :z-pos 1240 :yaw-angle (deg2rad 20) :grasp-center? t)
  ;; (0 0 6 5 4 3 2 1 1 0)
  )

(defun test-valve-motion-staro-pattern3
  (&key (ri-test? nil))
  (print "overwrite arm-side test")
  (test-valve-motion :ri-test? ri-test? :robot-name "STARO" :x-pos 420 :y-pos 600 :z-pos 1240 :overwrite-arm-side? t)
  )

(defun test-valve-motion-staro-pattern4
  (&key (ri-test? nil))
  (print "overwrite stand-coords test")
  (test-valve-motion :ri-test? ri-test? :robot-name "STARO" :x-pos 420 :y-pos 600 :z-pos 1240 :overwrite-stand-coords? t)
  )

(defun test-valve-motion-staro-lasvegas
  (&key (ri-test? nil))
  (print "drc environment in las vegas")
  (test-valve-motion :ri-test? ri-test? :robot-name "STARO" :valve-r 130 :x-pos 600 :y-pos -500 :z-pos 1200 :overwrite-stand-coords? :force)
  )

(warning-message 2 ";; test without ri~%")
(warning-message 2 "(test-valve-motion-without-ri)~%")
(warning-message 2 ";; test with ri~%")
(warning-message 2 "(test-valve-motion-hrp2jsknt-with-ri)~%")
(warning-message 2 "(test-valve-motion-hrp2jsknts-with-ri)~%")
(warning-message 2 "(test-valve-motion-jaxon-with-ri)~%")
(warning-message 2 "(test-valve-motion-staro-with-ri)~%")
