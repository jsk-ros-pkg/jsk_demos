;; pick-tabletop-object.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :detection-interface "package://jsk_perception/euslisp/detection_interface.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(require :pr2-pose "package://jsk_demo_common/euslisp/pr2-pose.l")

(defun pr2-pick-tray-pose (&key (arm :arms) (torso))
  (when torso
    (send *pr2* :torso :waist-z :joint-angle torso))
  (case arm
    (:larm
     (send *pr2* :larm :shoulder-p :joint-angle -20)
     (send *pr2* :larm :shoulder-r :joint-angle 90)
     (send *pr2* :larm :collar-y :joint-angle 60)
     (send *pr2* :larm :elbow-p :joint-angle -117)
     (send *pr2* :larm :elbow-r :joint-angle 0)
     (send *pr2* :larm :wrist-p :joint-angle -6)
     (send *pr2* :larm :wrist-r :joint-angle 0))
    (:rarm
     (send *pr2* :rarm :shoulder-p :joint-angle -20)
     (send *pr2* :rarm :shoulder-r :joint-angle -90)
     (send *pr2* :rarm :collar-y :joint-angle -60)
     (send *pr2* :rarm :elbow-p :joint-angle -117)
     (send *pr2* :rarm :elbow-r :joint-angle 0)
     (send *pr2* :rarm :wrist-p :joint-angle -6)
     (send *pr2* :rarm :wrist-r :joint-angle 0))
    (:arms
     (pr2-pick-tray-pose :arm :larm :torso torso)
     (pr2-pick-tray-pose :arm :rarm :torso torso)))
  (send *pr2* :angle-vector))

(defun pick-tabletop-object (obj &key (offset 100) (arm :larm) (flip-z-if-fail t) (rotation-axis t) (parent-grasp ""))
  (let ((cds (send obj :copy-worldcoords))
        pregrasp-av grasp-av pregrasp-task-name-local grasp-trial-task-local)
    (if *remotelogging*
     (progn (setq pregrasp-task-name-local (start-task "http://knowrob.org/kb/knowrob.owl#ArmMovement" "PreGraspPose"))
     (generate-subtask-relation parent-grasp pregrasp-task-name-local))
    )
    (send cds :translate (float-vector (- offset) 0 0))
    (setq pregrasp-av
          (send *pr2* :inverse-kinematics cds
                      :move-target (send *pr2* arm :end-coords)
                      :rotation-axis rotation-axis
                      :link-list (send *pr2* :link-list
                                       (send *pr2* arm :end-coords :parent)
                                       (send *pr2* :link "toarso_lift_link"))
                      :debug-view nil))
    (unless pregrasp-av
      (unless flip-z-if-fail (return-from pick-tabletop-object nil))
      (send cds :translate (float-vector offset 0 0))
      (send cds :rotate pi :z)
      (send cds :translate (float-vector (- offset) 0 0))
      (setq pregrasp-av
            (send *pr2* :inverse-kinematics cds
                        :move-target (send *pr2* arm :end-coords)
                        :rotation-axis rotation-axis
                        :link-list (send *pr2* :link-list
                                         (send *pr2* arm :end-coords :parent)
                                         (send *pr2* :link "toarso_lift_link"))
                        :debug-view nil))
      (unless pregrasp-av (return-from pick-tabletop-object nil)))
    (if *remotelogging*
      (let (query-arm id1 res1)
            (setq query-arm (concatenate string "send_prolog_assert_query('rdf_assert(\\'" pregrasp-task-name-local "\\', knowrob:bodyPartUsed, literal(type(xsd:string,\\'" (string arm) "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-arm))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
            (finish-task pregrasp-task-name-local)
      )
    )
    ;;
    (if *remotelogging*
       (progn (defvar grasp-trial-task-local (start-task "http://knowrob.org/kb/knowrob.owl#CRAMAchieve" "GraspTrial"))
              (generate-subtask-relation parent-grasp grasp-trial-task-local)))
    (setq cds (send *pr2* arm :end-coords :copy-worldcoords))
    (send cds :translate (float-vector offset 0 0))
    (setq grasp-av
          (send *pr2* :inverse-kinematics cds
                        :move-target (send *pr2* arm :end-coords)
                        :link-list (send *pr2* :link-list
                                         (send *pr2* arm :end-coords :parent)
                                         (send *pr2* :link "toarso_lift_link"))
                        :debug-view nil))
    (unless grasp-av
       (if *remotelogging* 
          (let (query-s id1 res1)
             (setq query-s (concatenate string "send_prolog_assert_query('rdf_assert(\\'" grasp-trial-task-local "\\', knowrob:taskSuccess, literal(type(xsd:boolean,\\'false\\')), \\'LoggingGraph\\'), rdf_assert(\\'" grasp-task-name "\\', knowrob:taskSuccess, literal(type(xsd:boolean,\\'true\\')), \\'LoggingGraph\\')', @(false), Result)."))
             (setq id1 (send-json-prolog-query query-s))
             (setq res1 (read-json-next-solution id1))
             (finish-json-query id1)
             (finish-task grasp-trial-task-local)))
      (return-from pick-tabletop-object nil))
    ;;
    (send *ri* :stop-grasp arm :wait t)
    (send *ri* :angle-vector-sequence
          (list pregrasp-av grasp-av) (list 5000 2000))
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp arm)
    (if *remotelogging* 
       (let (query-s id1 res1)
          (setq query-s (concatenate string "send_prolog_assert_query('rdf_assert(\\'" grasp-trial-task-local "\\', knowrob:taskSuccess, literal(type(xsd:boolean,\\'true\\')), \\'LoggingGraph\\'), rdf_assert(\\'" grasp-task-name "\\', knowrob:taskSuccess, literal(type(xsd:boolean,\\'true\\')), \\'LoggingGraph\\')', @(false), Result)."))
          (setq id1 (send-json-prolog-query query-s))
          (setq res1 (read-json-next-solution id1))
          (finish-json-query id1)
          (finish-task grasp-trial-task-local)))))
    
;;
(defun pick-serial (&optional (arm :larm))
  (if *remotelogging*
     (progn (defvar pregrasp-cornflakes-task-name (start-task "http://knowrob.org/kb/knowrob.owl#ArmMovement" "PreGraspPose"))
     (generate-subtask-relation grasp-cornflakes-task-name pregrasp-cornflakes-task-name))
  )
  (pr2-pick-tray-pose :arm arm :torso 180)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (if *remotelogging*
      (let (query-arm id1 res1)
            (setq query-arm (concatenate string "send_prolog_assert_query('rdf_assert(\\'" pregrasp-cornflakes-task-name "\\', knowrob:bodyPartUsed, literal(type(xsd:string,\\'" (string arm) "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-arm))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
            (finish-task pregrasp-cornflakes-task-name)
            (defvar cornflakes-perception-task (start-task "http://knowrob.org/kb/knowrob.owl#CRAMPerceive" "PerceiveCornflakes"))
            (generate-subtask-relation grasp-cornflakes-task-name cornflakes-perception-task)
      )
  )
  (let ((serial (check-detection :type "serial" :speak nil :timeout 70
                                 :detection-topic "/kinect_head/depth_registered/tabletop/ObjectDetection")))
    (if *remotelogging*
       (let (query-s id1 res1 query-position id2 res2 query-rotation id4 res4 query-object-acted id3 res3 quat)
           (setq query-s (concatenate string "send_prolog_assert_query('rdf_assert(\\'" cornflakes-perception-task "\\', knowrob:taskSuccess, literal(type(xsd:boolean,\\'true\\')), \\'LoggingGraph\\')', @(false), Result)."))
           (setq id1 (send-json-prolog-query query-s))
           (setq res1 (read-json-next-solution id1))
           (finish-json-query id1)
           (setq quat (ros::rot->tf-quaternion (send serial :worldrot))) 
           (setq query-position (concatenate string "send_prolog_assert_query('rdf_assert(\\'" cornflakes-perception-task "\\', knowrob:translation, literal(type(xsd:string,\\'" (format nil "~a"  (/ (elt (send serial :worldpos) 0) 1000.0)) " " (format nil "~a"  (/ (elt (send serial :worldpos) 1) 1000.0)) " " (format nil "~a"  (/ (elt (send serial :worldpos) 2) 1000.0))  "\\')), \\'LoggingGraph\\')', @(false), Result)."))
           (setq id2 (send-json-prolog-query query-position))
           (setq res2 (read-json-next-solution id2))
           (finish-json-query id2)
           (setq query-rotation (concatenate string "send_prolog_assert_query('rdf_assert(\\'" cornflakes-perception-task "\\', knowrob:quaternion, literal(type(xsd:string,\\'" (format nil "~a" (ros::quaternion-_x quat)) " " (format nil "~a" (ros::quaternion-_y quat)) " " (format nil "~a" (ros::quaternion-_z quat)) " " (format nil "~a" (ros::quaternion-_w quat)) "\\')), \\'LoggingGraph\\')', @(false), Result)."))
           (setq id4 (send-json-prolog-query query-rotation))
           (setq res4 (read-json-next-solution id4))
           (finish-json-query id4)
           (setq query-object-acted (concatenate string "send_prolog_assert_query('cram_set_object_acted_on(\\'" cornflakes-perception-task "\\',\\'" cornflakes-name "\\')', @(false), Result)."))
           (setq id3 (send-json-prolog-query query-object-acted))
           (setq res3 (read-json-next-solution id3))
           (finish-task cornflakes-perception-task)))
    (pick-tabletop-object serial :offset 150 :parent-grasp grasp-cornflakes-task-name)))
;;
(defun pick-bowl (&optional (arm :rarm))
  (if *remotelogging*
     (progn (defvar pregrasp-bowl-task-name (start-task "http://knowrob.org/kb/knowrob.owl#ArmMovement" "PreGraspPose"))
     (generate-subtask-relation grasp-bowl-task-name pregrasp-bowl-task-name))
  )
  (pr2-pick-tray-pose :arm arm :torso 180)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (if *remotelogging*
      (let (query-arm id1 res1)
            (setq query-arm (concatenate string "send_prolog_assert_query('rdf_assert(\\'" pregrasp-bowl-task-name "\\', knowrob:bodyPartUsed, literal(type(xsd:string,\\'" (string arm) "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-arm))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
            (finish-task pregrasp-bowl-task-name)
            (defvar bowl-perception-task (start-task "http://knowrob.org/kb/knowrob.owl#CRAMPerceive" "PerceiveBowl"))
            (generate-subtask-relation grasp-bowl-task-name bowl-perception-task)
      )
  )
  (let ((bowl (check-detection :type "red_bowl" :speak nil :timeout 70
                               :detection-topic "/kinect_head/depth_registered/tabletop/ObjectDetection")))
    (if *remotelogging*
       (let (query-s id1 res1 query-position id2 res2 query-rotation id4 res4 query-object-acted id3 res3 quat)
           (setq query-s (concatenate string "send_prolog_assert_query('rdf_assert(\\'" bowl-perception-task "\\', knowrob:taskSuccess, literal(type(xsd:boolean,\\'true\\')), \\'LoggingGraph\\')', @(false), Result)."))
           (setq id1 (send-json-prolog-query query-s))
           (setq res1 (read-json-next-solution id1))
           (finish-json-query id1)
           (setq quat (ros::rot->tf-quaternion (send bowl :worldrot))) 
           (setq query-position (concatenate string "send_prolog_assert_query('rdf_assert(\\'" bowl-perception-task "\\', knowrob:translation, literal(type(xsd:string,\\'" (format nil "~a"  (/ (elt (send bowl :worldpos) 0) 1000.0)) " " (format nil "~a"  (/ (elt (send bowl :worldpos) 1) 1000.0)) " " (format nil "~a"  (/ (elt (send bowl :worldpos) 2) 1000.0))  "\\')), \\'LoggingGraph\\')', @(false), Result)."))
           (setq id2 (send-json-prolog-query query-position))
           (setq res2 (read-json-next-solution id2))
           (finish-json-query id2)
           (setq query-rotation (concatenate string "send_prolog_assert_query('rdf_assert(\\'" bowl-perception-task "\\', knowrob:quaternion, literal(type(xsd:string,\\'" (format nil "~a" (ros::quaternion-_x quat)) " " (format nil "~a" (ros::quaternion-_y quat)) " " (format nil "~a" (ros::quaternion-_z quat)) " " (format nil "~a" (ros::quaternion-_w quat)) "\\')), \\'LoggingGraph\\')', @(false), Result)."))
           (setq id4 (send-json-prolog-query query-rotation))
           (setq res4 (read-json-next-solution id4))
           (finish-json-query id4)
           (setq query-object-acted (concatenate string "send_prolog_assert_query('cram_set_object_acted_on(\\'" bowl-perception-task "\\',\\'" bowl-name "\\')', @(false), Result)."))
           (setq id3 (send-json-prolog-query query-object-acted))
           (setq res3 (read-json-next-solution id3))
           (finish-task bowl-perception-task)))
    (send bowl :rotate -pi/2 :y)
    (send bowl :translate (float-vector -70 0 0) :world)
    (pick-tabletop-object bowl :arm arm :offset 100 parent-grasp grasp-bowl-task-name)))

(provide :pick-tabletop-object) ;; end of pick-tabletop-object.l
