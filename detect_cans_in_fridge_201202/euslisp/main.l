(defvar *use-voicetext* t)
(defvar *use-english* nil)
(defvar *logging* t)

;; open fridge and pick a can (and close the door)
(require :pr2-interface "package://pr2eus/pr2-interface.l")
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
(ros::roseus-add-msgs "pr2_gripper_sensor_msgs")

(ros::roseus "demo_main")
(setq *use-arm-navigation* nil)

(defun init-for-perception ()
  (unless (boundp '*scene*) (setq *scene* (make-eng2-scene)))
  (unless (boundp '*tfb*)
    (setq *tfb* (instance ros::transform-broadcaster :init)))
  (if (and x::*display* (/= x::*display* 0)) (make-irtviewer))
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
  (pr2-init)
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
          (speak-jp (format nil "I will bring  ~a." *type*)))
         (t (speak-jp (format nil "~aを持って行きます。" *type*))))
      (speak-jp (format nil "~a を もって いきます" *type*)))
    (send *ri* :move-to *opose*))

  (case finish-type
    (:put-on-turtlebot
     (put-can-on-turtlebot))
    ((:pass-to-human t)
     (if (and (boundp '*use-voicetext*) *use-voicetext*)
         (cond
          ((and (boundp '*use-english*) *use-english*)
           (speak-jp (format nil "Please take ~a." *type*)))
          (t (speak-jp (format nil "~aをどうぞ。" *type*))))
       (speak-jp (format nil "~a を どうぞ" *type*)))
     (hand-over use-arm :wait-shock t))
    )
  t
  )

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
  (if (and (boundp '*use-voicetext*) *use-voicetext*)
      (cond
       ((and (boundp '*use-english*) *use-english*)
        (speak-jp (format nil "I will look for ~a and bring it. Please wait a minute." *type*)))
       (t (speak-jp (format nil "~aを持って行きます。少々、お待ちください。" *type*))))
    (speak-jp (format nil "~a を もってきます  しょうしょう おまち ください" *type*)))
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

  (case demo-type
    (:map
     (unless (goto-front-of-fridge)
       (return-from demo)))
    (t
     ;; do nothing
     ))

  (unless (open-fridge-door :use-arm use-arm)
    (return-from demo))

  ;; move

  (unless (grasp-can :use-arm use-arm)
    (return-from demo))

  ;; move

  (unless (close-fridge :use-arm use-arm)
    (return-from demo))

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
  (if (eq app-manager :true) (ros::exit))
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
     (unless atype
       (enable-behavior-server))
     (ros::sleep))
    (ros::unsubscribe "/Tablet/StartDemo")
    (ros::unsubscribe "/Murase/Demo")
    ))

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
