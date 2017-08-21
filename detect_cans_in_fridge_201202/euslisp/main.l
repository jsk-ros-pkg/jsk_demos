;; open fridge and pick a can (and close the door)
(ros::roseus "demo_main")
(ros::roseus-add-msgs "pr2_gripper_sensor_msgs")
(ros::load-ros-manifest "json_prolog_msgs")

(defvar *use-voicetext* t)
(defvar *use-english* t)
(defvar *logging* nil)
(defvar *remotelogging* nil)
(defvar *debug-view* (ros::get-param "~debug_view" t))

(setq *use-arm-navigation* nil)

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :mongo-client "package://roseus_mongo/euslisp/mongo-client.l")
;;(load "package://pr2eus_openrave/pr2eus-openrave.l")
;;(load "package://pr2eus_armnavigation/pr2eus-arm-navigation.l")
;;(load "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")


(if *logging*
    (progn
      (load "package://jsk_demo_common/euslisp/attention-observation.l")
      (require :pr2-attention-move "package://jsk_demo_common/euslisp/pr2-attention-move.l")
      (require :pr2-attention-action "package://jsk_demo_common/euslisp/pr2-attention-action.l")
      (setq *current-context* :fridge))
    (progn
      (require :pr2-move "package://jsk_demo_common/euslisp/pr2-move.l")
      (require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")))


(defun init-for-perception ()
  (unless (boundp '*scene*) (setq *scene* (make-eng2-scene)))
  (unless (boundp '*tfb*)
    (setq *tfb* (instance ros::transform-broadcaster :init)))
  (if (and x::*display* (/= x::*display* 0) *debug-view*) (make-irtviewer))
  (setq *obj* (make-sphere 100)) ;; dummy object
  (setq *target-name* nil)
  (ros::spin-once)
  )

(defun init-for-robot ()
  #|
  (unless (boundp '*pr2*) (pr2))
  (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))
  (send *ri* :spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  |#
  (pr2-init *debug-view*)
  (when *use-arm-navigation*
    (setq *plan-env* (instance arm_planning_environment :init :robot *pr2*)))
  )

(defun init ()
  (init-for-perception)
  (init-for-robot)
  (defparameter *detection-topic* "/kinect_head/rgb/ObjectDetection")
  ;;(defparameter *base-frame-id* "/base_footprint")
  (require :detection-interface "package://jsk_perception/euslisp/detection_interface.l");;
  ;;(send *pr2* :move-to (send *ri* :state :worldcoords) :world)
  ;;(objects (list *pr2*))
  (if (send *ri* :simulation-modep)
      (progn
        (load "models/room73b2-scene.l")
        (room73b2)
        (send *ri* :objects (send *room73b2* :objects))
        ))
  )

(defun goto-initial-position (&key (return-to-initial-position t)
                                   (use-arm :rarm)
                                   (turnp t) (finish-type :pass-to-human))
  (when turnp
    (send *ri* :go-pos-unsafe 0 0 -90)
    (send *ri* :wait-interpolation))

  (when return-to-initial-position
    (if (and (boundp '*use-voicetext*) *use-voicetext*)
        (cond
         ((and (boundp '*use-english*) *use-english*)
          (speak-en (format nil "I will bring  ~a." *type*)))
         (t (speak-jp (format nil "~aを持って行きます。" *type*))))
      (speak-jp (format nil "~a を もって いきます" *type*)))
    (send *ri* :move-to *opose*))

  (if *remotelogging*
     (progn (defvar achieve-putdown-task-name (start-task "http://knowrob.org/kb/knowrob.owl#CRAMAchieve" (string finish-type)))
     (generate-subtask-relation go-to-start-task-name achieve-putdown-task-name))
    )
  (case finish-type
    (:put-on-turtlebot
     (put-can-on-turtlebot))
    ((:pass-to-human t)
     (if (and (boundp '*use-voicetext*) *use-voicetext*)
         (cond
          ((and (boundp '*use-english*) *use-english*)
           (speak-en (format nil "Please take ~a." *type*)))
          (t (speak-jp (format nil "~aをどうぞ。" *type*))))
       (speak-jp (format nil "~a を どうぞ" *type*)))
     (hand-over use-arm :wait-shock t))
    )
   (if *remotelogging*
     (finish-task achieve-putdown-task-name))
  t
  )

(defun fill-at-location-task (task vect)
  (let (query-pv id1 res1 query-f id2 res2
           (current-pos (apply #'format nil "~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A ~A" (coerce vect cons))))
            (setq query-pv (concatenate string "send_prolog_assert_query('rdf_assert(\\'" task "\\', knowrob:designator, literal(type(xsd:string,\\'" current-pos "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-pv))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
            (setq query-f (concatenate string "send_prolog_assert_query('rdf_assert(\\'" task "\\', knowrob:designatorFormat, literal(type(xsd:string,\\'torso_lift_joint l_shoulder_pan_joint l_shoulder_lift_joint l_upper_arm_roll_joint l_elbow_flex_joint l_forearm_roll_joint l_wrist_flex_joint l_wrist_roll_joint r_shoulder_pan_joint r_shoulder_lift_joint r_upper_arm_roll_joint r_elbow_flex_joint r_forearm_roll_joint r_wrist_flex_joint r_wrist_roll_joint head_pan_joint head_tilt_joint\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id2 (send-json-prolog-query query-f))
            (setq res2 (read-json-next-solution id2))
            (finish-json-query id2)
     
  )
)

(defun likely-location-to-open-fridge (positive-file negative-file pos)
  (let* (query-sent id1 res1 x-max y-max (pos-string "") (lst ()))
            (dotimes (i (length pos))
             (cond ((= i 0)
               (setq pos-string (format nil "~a" (elt pos i))))
              ((or (= i 3) (= i 7) (= i 11)) 
             (setq pos-string (format nil "~a, ~a" pos-string  (/ (elt pos i) 1000.0))))
              (t
             (setq pos-string (format nil "~a, ~a" pos-string  (elt pos i))))
             ))
            (setq query-sent (concatenate string "send_prolog_assert_query('register_ros_package(\\'knowrob_learning\\'), owl_parse(\\'package://knowrob_cloud_logger/owl/room73b2.owl\\'), owl_individual_of(_Door, knowrob:\\'IAIFridgeDoor\\'), object_pose_at_time(_Door, 1, mat(_DoorPose)), _CurrentPose = [" pos-string "], pose_into_relative_coord(_CurrentPose, _DoorPose, _RelativeCurrentPose), get_likely_pose(\\'" positive-file  "\\', 1, \\'" negative-file "\\', 1, _RelativeCurrentPose, _PosLst,_), pose_into_global_coord(_PosLst, _DoorPose, _GlobalPose), matrix_translation(_GlobalPose, [X,Y,_])', @(false), ['X', 'Y'], [X,Y])."))
            (setq id1 (send-json-prolog-query query-sent))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
            (print (send res1 :solution))
            (setq y-max (cdr (first (with-input-from-string (is (send res1 :solution)) (json::parse-object is)))))
            (setq x-max (cdr (second (with-input-from-string (is (send res1 :solution)) (json::parse-object is)))))
            (setf lst (cons y-max lst))
            (setf lst (cons x-max lst))
            lst
  )
)

(defun start-task (task-type task-context)
  (let (time-string query-sent id1 res1 task-name task-name-length)
            (setq time-string (get-current-time-str))
            (setq query-sent (concatenate string "send_prolog_assert_query('cram_start_action(\\'" task-type "\\', \\'" task-context "\\'," time-string ", PA, ActionInst)', @(false), 'ActionInst', Result)."))
            (setq id1 (send-json-prolog-query query-sent))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
            (setq task-name (cdr (first (with-input-from-string (is (send res1 :solution)) (json::parse-object is)))))
            (setq task-name-length (- (length task-name) 1))
            (subseq task-name 1 task-name-length)
  )
)

(defun finish-task (task-id)
 (let (time-string query-sent id1 res1)
            (setq time-string (get-current-time-str))
            (setq query-sent (concatenate string "send_prolog_assert_query('cram_finish_action(\\'" task-id "\\'," time-string ")', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-sent))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
  )
)

(defun generate-subtask-relation (task subtask)
 (let (time-string query-sent id1 res1)
            (setq time-string (get-current-time-str))
            (setq query-sent (concatenate string "send_prolog_assert_query('cram_set_subaction(\\'" task "\\',\\'" subtask "\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-sent))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
  )
)

(defun send-json-prolog-query (query)
  (let ((retinitial "empty")
        (retsolution "empty")
        (id (format nil "~a" (random 10000)))) 
    (if (ros::wait-for-service "/json_prolog/simple_query" 1)
      (let ((req (instance json_prolog_msgs::PrologQueryRequest :init)) querysent)
        (send req :id id)
        (setq querysent "connect_to_user_container")
        (send req :query query)
        (setq retinitial (ros::service-call "/json_prolog/simple_query" req))
        (ros::ros-warn "[prolog-sender] Querying openEASE via local-knowrob:json-prolog service.")
        id)
    (ros::ros-warn "json_prolog is not available"))
))

(defun read-json-next-solution (id)
  (if (ros::wait-for-service "/json_prolog/next_solution" 1)
      (let ((req (instance json_prolog_msgs::PrologNextSolutionRequest :init)) res)
        (send req :id id)
        (setq res (ros::service-call "/json_prolog/next_solution" req))
        res)
    (ros::ros-warn "[prolog-sender] json_prolog disappeared"))
)

(defun finish-json-query (id)
  (if (ros::wait-for-service "/json_prolog/finish" 1)
      (let ((req (instance json_prolog_msgs::PrologFinishRequest :init)))
        (send req :id id)
        (setq res (ros::service-call "/json_prolog/finish" req)))
    (ros::ros-warn "[prolog-sender] json_prolog disappeared"))
)


(defun authenticate-openease (token ip keypath)
  (let ((query-sent (concatenate string "cloud_interface(" ip "," keypath "," token "), start_user_container" )) id)
    (setq id (send-json-prolog-query query-sent))
    (read-json-next-solution id)
    (finish-json-query id) 
))

(defun connect-user-container ()
  (let ((query-sent "connect_to_user_container") id) 
    (setq id (send-json-prolog-query query-sent))
    (read-json-next-solution id)
    (finish-json-query id) 
))


(defun enable-behavior-server ()
  (if (ros::wait-for-service "/interactive_behavior_enable" 1)
      (let ((req (instance std_srvs::EmptyRequest :init)))
        (ros::service-call "/interactive_behavior_enable" req)
        (ros::ros-warn "ENABLED behavior server"))
    (ros::ros-warn "behavior server is not available")))

(defun disable-behavior-server ()
  (if (ros::wait-for-service "/interactive_behavior_disable" 1)
      (let ((req (instance std_srvs::EmptyRequest :init)))
        (ros::service-call "/interactive_behavior_disable" req)
        (ros::ros-warn "DISABLED behavior server"))
    (ros::ros-warn "behavior server is not available")))
  

(defun initialize-demo (atype)
  (setq *type* atype)
  (ros::ros-info "start fridge demo / ~A" *type*)
  (if *remotelogging*
      (defvar experiment-name (start-task "http://knowrob.org/kb/knowrob.owl#RobotExperiment" "DetectCan"))
  )
  (if (and (boundp '*use-voicetext*) *use-voicetext*)
      (cond
       ((and (boundp '*use-english*) *use-english*)
        (speak-en (format nil "I will look for ~a and bring it. Please wait a minute." *type*)))
       (t (speak-jp (format nil "~aを持って行きます。少々、お待ちください。" *type*))))
    (speak-jp (format nil "~a を もってきます  しょうしょう おまち ください" *type*)))
  (send *ri* :start-grasp :arms)
  (pr2-tuckarm-pose :rarm)

  (setq *opose* (send *ri* :state :worldcoords))
  )

(defun demo (&key ((:type atype) "georgia")
                  (demo-type :map)  ;; :map, :short, ...
                  (app-manager :false)
                  (finish-type :pass-to-human) ;; :pass-to-human, :put-on-turtlebot, ...
                  ;;(use-arm :rarm)
                  (use-arm :larm)
                  )
  (disable-behavior-server)
  (case app-manager
    (:true
     ;; get type from parameter server
     (if (ros::has-param "/app_execute/target")
         (initialize-demo
          (ros::get-param "/app_execute/target"))
       (initialize-demo atype)
     ))
    (t
     (initialize-demo atype)
     ))

  (if *remotelogging*
      (let (query-object-acted id1 res1)
            (defvar go-to-fridge-task-name (start-task "http://knowrob.org/kb/knowrob.owl#Navigate" "GoNearByFridge"))
            (generate-subtask-relation experiment-name go-to-fridge-task-name)
            (defvar fridge-name "http://knowrob.org/kb/room73b2.owl#room73b2_1_room73b2-hitachi-fiesta-refrigerator_0_ROOT_link")
            (setq query-object-acted (concatenate string "send_prolog_assert_query('cram_set_object_acted_on(\\'" go-to-fridge-task-name "\\',\\'" fridge-name "\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-object-acted))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
      )
  )
  (case demo-type
    (:map
     (unless (goto-front-of-fridge)
       (return-from demo)))
    (t
     ;; do nothing
     ))
  (if *remotelogging*
      (finish-task go-to-fridge-task-name)
  )


  (if *remotelogging*
      (let (query-object-acted query-arm id1 res1 id2 res2 reverse-arm)
            (defvar open-fridge-task-name (start-task "http://knowrob.org/kb/knowrob.owl#CRAMPerform" "OpenFridge"))
            (generate-subtask-relation experiment-name open-fridge-task-name)
            (setq query-object-acted (concatenate string "send_prolog_assert_query('cram_set_object_acted_on(\\'" open-fridge-task-name "\\',\\'" fridge-name "\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-object-acted))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
            (if (eq (string use-arm) "LARM")
                (setq reverse-arm "RARM")
                (setq reverse-arm "LARM")) 
            (setq query-arm (concatenate string "send_prolog_assert_query('rdf_assert(\\'" open-fridge-task-name "\\', knowrob:bodyPartUsed, literal(type(xsd:string,\\'" reverse-arm "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id2 (send-json-prolog-query query-arm))
            (setq res2 (read-json-next-solution id2))
            (finish-json-query id2)
      )
  )
  (unless (open-fridge-door :use-arm use-arm)
    (return-from demo))
  (if *remotelogging*
      (finish-task open-fridge-task-name)
  )

  ;; move
  (if *remotelogging*
      (let (query-object-acted query-arm id1 res1 id2 res2)
            (defvar grasp-task-name (start-task "http://knowrob.org/kb/knowrob.owl#Grasp" "GraspObject"))
            (generate-subtask-relation experiment-name grasp-task-name)
            (defvar milk-box-name "http://knowrob.org/kb/room73b2.owl#room73b2_1_milkbox_76Gf1Q")
            (setq query-object-acted (concatenate string "send_prolog_assert_query('cram_set_object_acted_on(\\'" grasp-task-name "\\',\\'" milk-box-name "\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-object-acted))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
            (setq query-arm (concatenate string "send_prolog_assert_query('rdf_assert(\\'" grasp-task-name "\\', knowrob:bodyPartUsed, literal(type(xsd:string,\\'" (string use-arm) "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id2 (send-json-prolog-query query-arm))
            (setq res2 (read-json-next-solution id2))
            (finish-json-query id2)
      )
  )
  (unless (grasp-can :use-arm use-arm)
    (return-from demo))
  (if *remotelogging*
      (finish-task grasp-task-name)
  )

  ;; move

  (if *remotelogging*
      (let (query-object-acted query-arm id1 res1)
            (defvar close-fridge-task-name (start-task "http://knowrob.org/kb/knowrob.owl#ArmMovement" "CloseFridge"))
            (generate-subtask-relation experiment-name close-fridge-task-name)
            (setq query-object-acted (concatenate string "send_prolog_assert_query('cram_set_object_acted_on(\\'" close-fridge-task-name "\\',\\'" fridge-name "\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-object-acted))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)))

  (unless (close-fridge :use-arm use-arm)
    (return-from demo))
  (if *remotelogging*
      (finish-task close-fridge-task-name)
  )

  (if *remotelogging*
      (defvar go-to-start-task-name (start-task "http://knowrob.org/kb/knowrob.owl#BaseMovement" "GoBackToStartPose"))
      (generate-subtask-relation experiment-name go-to-start-task-name)
  )
  (case demo-type
    (:map
     (goto-initial-position :finish-type finish-type :use-arm use-arm)
     )
    (t
     (unless (goto-initial-position
              :return-to-initial-position nil :turnp t
              :use-arm use-arm
              :finish-type finish-type)
       (return-from demo)))
    )
  (if *remotelogging*
      (finish-task go-to-start-task-name)
  )
  (enable-behavior-server)
  (if *remotelogging*
      (let (query-owl id-owl res-owl)
            (finish-task experiment-name)
            (setq query-owl (concatenate string "send_prolog_assert_query(' rdf_save(\\'/home/ros/user_data/euslisp.owl\\', [graph(\\'LoggingGraph\\')])', @(false), Result)."))
            (setq id-owl (send-json-prolog-query query-owl))
            (setq res-owl (read-json-next-solution id-owl))
            (finish-json-query id-owl)
      )
  )
  (if (eq app-manager :true) (ros::exit))
  )

(defun get-current-time-str ()
  (let ((current-sec (format nil "~a" (send (ros::time-now) :sec))) (current-nsec (format nil "~a" (send (ros::time-now) :nsec))) time-string)    
     (setq time-string (concatenate string current-sec "." current-nsec))   
     time-string 
))

(defun demo-openease (&key ((:type atype) "georgia")
                  (demo-type :map)  ;; :map, :short, ...
                  (app-manager :false)
                  (finish-type :pass-to-human) ;; :pass-to-human, :put-on-turtlebot, ...
                  ;;(use-arm :rarm)
                  (use-arm :larm)
                  apikey address certificate)
  (setq *logging* nil)
  (setq *remotelogging* t)
  (ros::ros-info "authenticate start")
  (speak-en "I am connecting to open ease, my social network")
  (authenticate-openease apikey address certificate)
  (ros::ros-info "connecting to user container")
  (connect-user-container)
  (ros::ros-info "starting demo")
  (demo :type atype :demo-type demo-type :app-manager app-manager :finish-type finish-type :use-arm use-arm)
  )

(defun demo-breakfast-openease (&key ((:type atype) "georgia")
                  (demo-type :map)  ;; :map, :short, ...
                  (app-manager :false)
                  (finish-type :pass-to-human) ;; :pass-to-human, :put-on-turtlebot, ...
                  ;;(use-arm :rarm)
                  (use-arm :larm)
                  apikey address certificate)
  (setq *logging* nil)
  (setq *remotelogging* t)
  (ros::ros-info "authenticate start")
  (speak-en "I am connecting to open ease, my social network")
  (authenticate-openease apikey address certificate)
  (ros::ros-info "connecting to user container")
  (connect-user-container)
  (ros::ros-info "starting demo")


  ;;initialize demo
  (setq *type* atype)
  (ros::ros-info "start breakfast demo / ~A" atype)
  (if *remotelogging*
      (defvar experiment-name (start-task "http://knowrob.org/kb/knowrob.owl#RobotExperiment" "PrepareBreakfast"))
  )
  (if (and (boundp '*use-voicetext*) *use-voicetext*)
      (cond
       ((and (boundp '*use-english*) *use-english*)
        (speak-en (format nil "Good morning. I presume you are hungry! I will prepare a breakfast for you! Please wait a minute." *type*)))
       (t (speak-jp (format nil "~aを持って行きます。少々、お待ちください。" *type*))))
    (speak-jp (format nil "~a を もってきます  しょうしょう おまち ください" *type*)))
  (send *ri* :start-grasp :arms)
  (pr2-tuckarm-pose :rarm)

  (setq *opose* (send *ri* :state :worldcoords))

  (if *remotelogging*
      (let (query-object-acted id1 res1)
            (defvar go-to-fridge-task-name (start-task "http://knowrob.org/kb/knowrob.owl#Navigate" "GoNearByFridge"))
            (generate-subtask-relation experiment-name go-to-fridge-task-name)
            (defvar fridge-name "http://knowrob.org/kb/room73b2.owl#room73b2_1_room73b2-hitachi-fiesta-refrigerator_0_ROOT_link")
            (setq query-object-acted (concatenate string "send_prolog_assert_query('cram_set_object_acted_on(\\'" go-to-fridge-task-name "\\',\\'" fridge-name "\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-object-acted))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
      )
  )
  (case demo-type
    (:map
     (unless (goto-front-of-fridge)
       (return-from demo)))
    (t
     ;; do nothing
     ))
  (if *remotelogging*
      (finish-task go-to-fridge-task-name)
  )

  (if *remotelogging*
      (let (query-object-acted query-arm id1 res1 id2 res2 reverse-arm)
            (defvar open-fridge-task-name (start-task "http://knowrob.org/kb/knowrob.owl#CRAMPerform" "OpenFridge"))
            (generate-subtask-relation experiment-name open-fridge-task-name)
            (setq query-object-acted (concatenate string "send_prolog_assert_query('cram_set_object_acted_on(\\'" open-fridge-task-name "\\',\\'" fridge-name "\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-object-acted))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
            (if (eq (string use-arm) "LARM")
                (setq reverse-arm "RARM")
                (setq reverse-arm "LARM")) 
            (setq query-arm (concatenate string "send_prolog_assert_query('rdf_assert(\\'" open-fridge-task-name "\\', knowrob:bodyPartUsed, literal(type(xsd:string,\\'" reverse-arm "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id2 (send-json-prolog-query query-arm))
            (setq res2 (read-json-next-solution id2))
            (finish-json-query id2)
      )
  )
  (unless (open-fridge-door :use-arm use-arm)
    (return-from demo))
  (if *remotelogging*
      (finish-task open-fridge-task-name)
  )

  ;; move
  (if *remotelogging*
      (let (query-object-acted query-arm id1 res1 id2 res2)
            (defvar grasp-task-name (start-task "http://knowrob.org/kb/knowrob.owl#Grasp" "GraspObject"))
            (generate-subtask-relation experiment-name grasp-task-name)
            (defvar milk-box-name "http://knowrob.org/kb/room73b2.owl#room73b2_1_milkbox_76Gf1Q")
            (setq query-object-acted (concatenate string "send_prolog_assert_query('cram_set_object_acted_on(\\'" grasp-task-name "\\',\\'" milk-box-name "\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-object-acted))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
            (setq query-arm (concatenate string "send_prolog_assert_query('rdf_assert(\\'" grasp-task-name "\\', knowrob:bodyPartUsed, literal(type(xsd:string,\\'" (string use-arm) "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id2 (send-json-prolog-query query-arm))
            (setq res2 (read-json-next-solution id2))
            (finish-json-query id2)
      )
  )
  (unless (grasp-can :use-arm use-arm)
    (return-from demo))
  (if *remotelogging*
      (finish-task grasp-task-name)
  )

  ;; move

  (if *remotelogging*
      (let (query-object-acted query-arm id1 res1)
            (defvar close-fridge-task-name (start-task "http://knowrob.org/kb/knowrob.owl#ArmMovement" "CloseFridge"))
            (generate-subtask-relation experiment-name close-fridge-task-name)
            (setq query-object-acted (concatenate string "send_prolog_assert_query('cram_set_object_acted_on(\\'" close-fridge-task-name "\\',\\'" fridge-name "\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-object-acted))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)))

  (unless (close-fridge :use-arm use-arm)
    (return-from demo))
  (if *remotelogging*
      (finish-task close-fridge-task-name)
  )

  (if *remotelogging*
      (let (query-object-acted id1 res1)
            (defvar go-to-dining-table-task-name (start-task "http://knowrob.org/kb/knowrob.owl#Navigate" "GoNearByFridge"))
            (generate-subtask-relation experiment-name go-to-dining-table-task-name)
            (defvar dining-table-name "http://knowrob.org/kb/room73b2.owl#room73b2_1_room73b2-karimoku-table_0_nil_link")
            (setq query-object-acted (concatenate string "send_prolog_assert_query('cram_set_object_acted_on(\\'" go-to-dining-table-task-name "\\',\\'" dining-table-name "\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-object-acted))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
      )
  )
  (case demo-type
    (:map
     (unless (goto-front-of-dining-table)
       (return-from demo)))
    (t
     ;; do nothing
     ))
  (if *remotelogging*
      (finish-task go-to-dining-table-task-name)
  )
  
)

(defun wait-android-query (&optional (demo-type :map))
  (ros::ros-info "Begin Waiting .... ~A" demo-type)
  (let (atype from)
    (ros::subscribe "/Tablet/StartDemo" roseus::StringStamped
                    #'(lambda(m)
                        (print (list :subscribe (send m :data)))
                        (cond
                         ((string= (send m :data) "/Tablet/other/GetGeorgia")
                          (setq from :tablet)
                          (setq atype "georgia"))
                         (t ))))
    (ros::subscribe "/Murase/Demo" std_msgs::String
                    #'(lambda(m)
                        (print (list :subscribe (send m :data)))
                        (cond
                         ((string= (send m :data) "georgia")
                          (setq from :app)
                          (setq atype "georgia"))
                         (t ))))
    (ros::rate 2)
    (do-until-key
     (ros::ros-info " Waiting ....")
     (ros::spin-once)
     (send *ri* :spin-once)
     (when atype
       (case from
         (:tablet
          (ros::ros-info "detect_cans: starting demo with ~A" atype)
          (demo :type atype :demo-type demo-type))
         (:app
          (ros::ros-info "detect_cans: starting app demo with ~A" atype)
          ;;(demo :type type)
          (demo :type atype :demo-type demo-type :finish-type :put-on-turtlebot))
         )
       (ros::spin-once)
       (setq atype nil from nil)
       )
     (ros::sleep))
    (ros::unsubscribe "/Tablet/StartDemo")
    (ros::unsubscribe "/Murase/Demo")
    ))

(defun demo-openease-short ()
  (defvar *use-english* t)
  ;(demo-openease :type "morinaga" :apikey "'wStdxHsWLm2QMUV4OepDkwwqLqGdYH43SE71iCqNGxnPGsDbKCjfXrBu9q1CcsOv'" :address "'https://133.11.216.20'" :certificate "'/home/asil/cert/localhost.pem'"))
  (demo-breakfast-openease :type "morinaga" :apikey "'wStdxHsWLm2QMUV4OepDkwwqLqGdYH43SE71iCqNGxnPGsDbKCjfXrBu9q1CcsOv'" :address "'https://133.11.216.20'" :certificate "'/home/asil/cert/localhost.pem'"))

(init)
;;(wait-android-query)

(warn ";; (init) ;; initialize demo~%")
(warn ";; (demo) ;; start demo~%")
(warn ";; (demo :type \"georgia\")~%") ;; type georgia wanda iemon
(warn ";; (demo :type \"georgia\" :demo-type :short)~%")
(warn ";; ~%")
(warn ";; (check-detection :type \"fridge\")~%")
(warn ";; ~%")
(warn ";; (goto-front-of-fridge)~%")
(warn ";; ~%")
(warn ";; (open-fridge-door)~%")
(warn ";; ~%")
(warn ";; (grasp-can :move nil)~%")

;;(demo :type "georgia")

;; (progn (check-detection :type "wonda")(grasp-can *obj*))
;;(check-detection :type "fridge")
;;(setq *type* "georgia")
;;(check-detection :type *type* :tf-force t :timeout 30)
;;(open-fridge-door :type)
;;(grasp-can :move nil)
;; (open-fridge-door :door-type :slide1 :look-around nil)
;;
;; (load "package://ee_cart_imped_tutorial/scripts/imped_motion_util.l") ;; override move-arm
;; (open-fridge-door :open-fridge-func #'open-fridge-traj-imped)
;; (open-fridge-traj-imped :rarm *last-fridge-handle* 1.7 :radius 490 :use-torso nil :time-tick 400 :path-div 32 :wait-time 0.1 :grasp-check nil :rotation-asix t :door-type :circle)
;; (open-fridge-traj-imped :rarm bcds 200 :use-torso nil :time-tick 400 :path-div 32 :wait-time 0.1 :grasp-check nil :rotation-asix t :door-type :slide)
;; (setq cds (make-coords :pos (float-vector 775.472 97.99 1137.752) :rpy (list 0.054 0 0)))
;; (setq bcds (send (send *last-fridge-handle* :copy-worldcoords) :translate #f(0 -230 -355)))
;; (setq ccds (send (send *last-fridge-handle* :copy-worldcoords) :translate #f(0 -230 -675)))
;;
(provide :fridge-main)
