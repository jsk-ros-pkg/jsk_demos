(load "package://drc_task_common/euslisp/generate-valve-motion.l")

(ros::roseus "test")

(defun test-valve-motion
  (&key (robot-name "HRP2JSKNT")
        (valve-r (/ 360 2))
        (x-pos 500) (y-pos -120) (z-pos 1260) (yaw-angle 0)
        (valve-coords (make-coords :pos (float-vector x-pos y-pos z-pos) :rpy (list yaw-angle pi/2 0)))
        (grasp-center? nil) (arm :rarm)
        (overwrite-stand-coords? nil) (overwrite-arm-side? nil)
        (ri-test? nil)
        )
  (cond ((not ri-test?)
         (get-valve-motion-full
          valve-r valve-coords
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
         (send *robot* :fix-leg-to-coords (make-coords))
         (let* ((valve-coords-relative-to-body
                 (send (send (car (send *robot* :links)) :copy-worldcoords) :transformation valve-coords :local)))
           (send-valve-motion
            valve-r valve-coords-relative-to-body
            :arm arm :grasp-center? grasp-center?
            :overwrite-stand-coords? overwrite-stand-coords? :overwrite-arm-side? overwrite-arm-side?))
         )
        )
  )

(defun test-valve-motion-without-ri
  ()
  (setq *robot* nil)
  (test-valve-motion-hrp2jsknt-pattern1 :ri-test? nil)
  (test-valve-motion-hrp2jsknt-pattern2 :ri-test? nil)
  (test-valve-motion-hrp2jsknt-pattern3 :ri-test? nil)
  (test-valve-motion-hrp2jsknt-pattern4 :ri-test? nil)
  (test-valve-motion-hrp2jsknt-pattern5 :ri-test? nil)
  (setq *robot* nil)
  (test-valve-motion-jaxon-pattern1 :ri-test? nil)
  (test-valve-motion-jaxon-pattern2 :ri-test? nil)
  (test-valve-motion-jaxon-pattern3 :ri-test? nil)
  (test-valve-motion-jaxon-pattern4 :ri-test? nil)
  )

(defun test-valve-motion-hrp2jsknt-with-ri
  ()
  (setq *robot* nil)
  (test-valve-motion-hrp2jsknt-pattern1 :ri-test? t)
  (test-valve-motion-hrp2jsknt-pattern2 :ri-test? t)
  (test-valve-motion-hrp2jsknt-pattern5 :ri-test? t)
  )

(defun test-valve-motion-hrp2jsknt-pattern1
  (&key (ri-test? nil))
  (print "rarm test")
  (test-valve-motion :ri-test? ri-test? :x-pos 500 :y-pos -120 :z-pos 1260 :arm :rarm)
  )

(defun test-valve-motion-hrp2jsknt-pattern2
  (&key (ri-test? nil))
  (print "larm test")
  (test-valve-motion :ri-test? ri-test? :x-pos 500 :y-pos 120 :z-pos 1260 :arm :larm)
  )

(defun test-valve-motion-hrp2jsknt-pattern3
  (&key (ri-test? nil))
  (print "overwrite arm-side test")
  (test-valve-motion :ri-test? ri-test? :x-pos 500 :y-pos 120 :z-pos 1260 :arm :rarm :overwrite-arm-side? t)
  )
(defun test-valve-motion-hrp2jsknt-pattern4
  (&key (ri-test? nil))
  (print "overwrite stand-coords test")
  (test-valve-motion :ri-test? ri-test? :x-pos 500 :y-pos 120 :z-pos 1260 :arm :rarm :overwrite-stand-coords? t)
  )

(defun test-valve-motion-hrp2jsknt-pattern5
  (&key (ri-test? nil))
  (print "grasp center test")
  (test-valve-motion :ri-test? ri-test? :x-pos 400 :y-pos -500 :z-pos 1260 :arm :rarm :grasp-center? t)
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

(warning-message 2 ";; test without ri~%")
(warning-message 2 "(test-valve-motion-without-ri)~%")
(warning-message 2 ";; test with ri~%")
(warning-message 2 "(test-valve-motion-hrp2jsknt-with-ri)~%")
(warning-message 2 "(test-valve-motion-jaxon-with-ri)~%")
