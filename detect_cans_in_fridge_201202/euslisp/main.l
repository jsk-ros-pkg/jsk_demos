;; open fridge and pick a can (and close the door)
(load "package://jsk_2011_07_pr2_semantic/euslisp/actions.l")
(load "package://pr2eus/pr2-interface.l")

(load "package://jsk_demo_common/euslisp/pr2-move.l")
(load "package://jsk_demo_common/euslisp/pr2-action.l")
;;(load "package://pr2eus_openrave/pr2eus-openrave.l")
;;(load "package://pr2eus_armnavigation/pr2eus-arm-navigation.l")

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
  (unless (boundp '*pr2*) (pr2))
  (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))
  (send *ri* :spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (when *use-arm-navigation*
    (setq *plan-env* (instance arm_planning_environment :init :robot *pr2*)))
  )

(defun init ()
  (init-for-perception)
  (init-for-robot)
  (defparameter *detection-topic* "/openni/rgb/ObjectDetection")
  ;;(defparameter *base-frame-id* "/base_footprint")
  (load "package://jsk_perception/euslisp/detection_interface.l");;
  ;;(send *pr2* :move-to (send *ri* :state :worldcoords) :world)
  ;;(objects (list *pr2*))
  )

(defun goto-initial-position (&key (return-to-initial-position t) (turnp t))
  (when turnp
    (send *ri* :go-pos-unsafe 0 0 -90)
    (send *ri* :wait-interpolation))
  (when return-to-initial-position
    (speak-jp (format nil "~a を もって いきます" *type*))
    (send *ri* :move-to *opose*))

  (speak-jp (format nil "~a を どうぞ" (send *obj* :name)))
  (hand-over :rarm :wait-shock t)
  t
  )

(defun demo (&key (type "georgia"))
  ;;(init)
  (setq *type* type)
  (speak-jp (format nil "~a を もってきます  しょうしょう おまち ください" *type*))

  (pr2-tuckarm-pose :rarm)

  (setq *opose* (send *ri* :state :worldcoords))

  (unless (goto-front-of-fridge)
    (return-from demo))

  (unless (open-fridge-door)
    (return-from demo))

  (unless (grasp-can)
    (return-from demo))

  (unless (close-fridge)
    (return-from demo))

  (unless (goto-initial-position)
    (return-from demo))
  )

(defun demo-short (&key (type "georgia"))
  (setq *type* type)
  (speak-jp (format nil "~a を もってきます  しょうしょう おまち ください" *type*))

  (pr2-tuckarm-pose :rarm)
  (setq *opose* (send *ri* :state :worldcoords))

  ;;do not use map
  ;;(unless (goto-front-of-fridge)
  ;;(return-from demo))

  (unless (open-fridge-door)
    (return-from demo-short))

  (unless (grasp-can)
    (return-from demo-short))

  (unless (close-fridge)
    (return-from demo-short))

  (unless (goto-initial-position
           :return-to-initial-position nil :turnp t)
    (return-from demo-short))
  )
#|
(defun wait-query ()
  (setq *type* nil)
  (ros::subscribe "/query" std_msgs::String
          #'(lambda(m) (setq *type* (send m :data))))
  (do-until-key
   (ros::spin-once)
   (send *ri* :spin-once)
   (when *type*
     (ros::unsubscribe "/query")
     (demo *type*)
     (return-from wait-query t))
   (ros::sleep))
  )
|#
;; this is just an another sample
(defun wait-android-query-test ()
  (ros::ros-info "Begin Waiting ....")
  (let (type)
    (ros::subscribe "/Tablet/StartDemo" roseus::StringStamped
                    #'(lambda(m)
                        (print (list :subscribe (send m :data)))
                        (cond
                         ((string= (send m :data) "/Tablet/other/GetGeorgia")
                          (setq type "georgia"))
                         (t ))))
    (ros::rate 4)
    (do-until-key
     (ros::ros-info " Waiting ....")
     (ros::spin-once)
     (send *ri* :spin-once)
     (when type
       (ros::unsubscribe "/Tablet/StartDemo")
       (ros::ros-info "detect_cans: starting demo with ~A" type)
       ;;(demo :type type)
       (demo-short :type type)
       (setq type nil))
     (ros::sleep))
    ))

(init)
(wait-android-query-test)

(warn ";; (init) ;; initialize demo~%")
(warn ";; (demo) ;; start demo~%")
(warn ";; (demo :type \"georgia\")~%")
(warn ";; (demo :type \"iemon\")~%")
(warn ";; (demo :type \"wonda\")~%")
(warn ";; ~%")
(warn ";; (check-detection :type \"fridge\")~%")

(warn ";; ~%")
(warn ";; (goto-front-of-fridge)~%")


;;(demo :type "georgia")

;; (progn (check-detection :type "wonda")(grasp-can *obj*))
;;(check-detection :type "fridge")
;;(setq *type* "georgia")
;;(check-detection :type *type* :tf-force t :timeout 30)
;;(open-fridge-door)
;;(grasp-can :move nil)