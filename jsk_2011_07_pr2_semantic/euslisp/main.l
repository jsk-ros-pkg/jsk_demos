#!/usr/bin/env roseus

(load "package://jsk_maps/src/eng2-scene.l")
(load "package://pr2eus/pr2-interface.l")
(load "knowrob-interface.l") ;; -> json_prolog/src
(load "actions.l")

(ros::roseus "demo_main")

(defun init ()
  ;; env
  ;;  (setq *scene* (make-eng2-scene))
  (unless (boundp '*pr2*) (pr2))
  (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))
  (unless (boundp '*tfl*)(setq *tfl* (instance ros::transform-listener :init)))
  (setq *prolog* (instance json_prolog :init "json_prolog"))
  ;; callbacks
  (ros::subscribe "/query" std_msgs::string #'identity)
  )

(defun insert-robot-pose ()
  (let (pose po ret
	     (type "'http://ias.cs.tum.edu/kb/knowrob.owl#Robot'")
		 (robot "'http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#pr2'"))
    (ros::spin-once)
    (setq pose (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)))
    (send pose :locate (scale 0.001 (send pose :worldpos)) :world)
    (setq po (coerce (array-entity (send pose :4x4)) cons))
    (setq po (mapcan #'(lambda(x)(coerce (format nil ",~a" x) cons)) po))
    (setq po (coerce (cdr po) string))
    (send *prolog* :query
	  (list->prolog `((create_object_instance ,type ,robot)))) ;; always success
    (setq ret (send *prolog* :query
		    (list->prolog `((update_pose ,robot ,(format nil "[~a]" po))))))
    ret ))


;; ask to knowrob
(defun ask-query-and-move (&optional (type "Cup"))
  (let* (solutions pose opose)
	(ros::spin-once)
	(setq opose (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)))
    (insert-robot-pose)
    (setq solutions (similar-objects-with-info-by-distance type))
    (dolist (solution solutions)
      (setq obj (cadr (assoc "OBJ" solution :test #'equal)))
      (setq robot-pose (cadr (assoc "POSE_SPOT" solution :test #'equal)))
      (ros::spin-once)
      (send *ri* :move-to robot-pose) ;; robot moves
      (ros::spin-once)
      (send *pr2* :move-to (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)) :world)
      (setq obj-pose (cadr (assoc "POSE_OBJ" solution :test #'equal)))
      (send *pr2* :head :look-at (send obj-pose :worldpos))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      ;; grasp cup here
      (when (check :timeout 20) ;; detection success
	(pick :larm)
	;; go to somewhere
	(pr2-tuckarm-pose :larm)
	(send *ri* :move-to opose)
	(hand-over)
	(return))
      )))

(defun demo (&rest args)
  (when (eq args nil)
    (warn ":home :pick~%")
    (return-from demo t))
  ;;
  (when (member :home args)
    (send *ri* :move-to
	  (cadr (assoc "POSE_SPOT" (car (find-knowrob-objects-with-info "OfficeRoom")) :test #'equal))))
  ;;
  (when (member :pick args)
    (ask-query-and-move "Cup"))
  ;;
)

;;
(progn
  (init)  ;; for pr2 setup
  (init2) ;; for cup detection
)
;(ask-query-and-move "Cup")
;(ros::spin)
;(exit)

