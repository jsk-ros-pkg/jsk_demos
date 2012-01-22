#!/usr/bin/env roseus

(load "package://jsk_maps/src/eng2-scene.l")
(load "package://pr2eus/pr2-interface.l")
(load "knowrob-interface.l") ;; -> json_prolog/src
(load "actions.l")

(ros::roseus "demo_main")

(defun init ()
  ;; env
  (unless (boundp '*scene*) (setq *scene* (make-eng2-scene)))
  (unless (boundp '*pr2*) (pr2))
;;  (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init :move-base-action-name "elevator_move_base")))
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
    (setq pose (send *ri* :state :worldcoords))
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
(defun ask-query-and-move (&optional (type "Cup") (name nil))
  (let* (solutions pose opose flag)
    (ros::spin-once)
    (setq opose (send *ri* :state :worldcoords))
    (setq *opose* opose)
    ;;(setq opose *opose*)
    (insert-robot-pose)
    ;;(setq solutions (similar-objects-with-info-by-distance type))
    (setq solutions (find-knowrob-objects-with-info type))
    (speak-jp "ごようけんはなんですか") (unix::sleep 4)
    (dolist (solution solutions)
      (setq obj (cadr (assoc "OBJ" solution :test #'equal)))
      (setq robot-pose (cadr (assoc "POSE_SPOT" solution :test #'equal)))
      (ros::spin-once)
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (send *pr2* :head :angle-vector #f(0 0))
      (send *pr2* :torso :angle-vector #f(0))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
      (clear-costmap)
      (send *ri* :wait-interpolation)
      (send *ri* :move-to robot-pose) ;; robot moves
      (ros::spin-once)
      (send *pr2* :move-to (send *ri* :state :worldcoords) :world)
      (setq obj-pose (cadr (assoc "POSE_OBJ" solution :test #'equal)))
      (send *pr2* :head :look-at (send obj-pose :worldpos))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      ;; grasp cup here
      (speak-jp "こっぷをさがしています")
      (let ((neck-av (send *pr2* :head :angle-vector)))
	(setq flag
	      (or (check :timeout 6 :type name)
		  (progn (send *pr2* :head :angle-vector (v+ #f(20 0) neck-av))
			 (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
			 (send *ri* :wait-interpolation)
			 (check :timeout 6 :type name))
		  (progn (send *pr2* :head :angle-vector (v+ #f(-20 0) neck-av))
			 (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
			 (send *ri* :wait-interpolation)
			 (check :timeout 6 :type name)))
	      ))
      (when flag
	(speak-jp "こっぷがみつかりました")
	(send *pr2* :head :look-at (send *obj* :worldpos))
	(send *ri* :head-angle-vector (send *pr2* :angle-vector) 500)
	(or (pick *obj* :larm)
	    (progn
	      (check :timeout 4 :type name)
	      (pick *obj* :larm)))
	;; go to somewhere
	(pr2-tuckarm-pose :larm)
	(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
	(send *pr2* :head :angle-vector #f(0 0))
	(send *pr2* :torso :angle-vector #f(0))
	(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
	(send *ri* :wait-interpolation)
	(clear-costmap)
	(send *ri* :move-to opose)
	(send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
	(send *ri* :head-angle-vector (send *pr2* :angle-vector) 1000)
	(speak-jp "こっぷをもってきました")
	(hand-over :larm) ;; larm
	(return-from ask-query-and-move t))
      (speak-jp "こっぷはみつかりませんでした")
      (pr2-tuckarm-pose :larm)
      )
    (send *ri* :move-to opose)
    nil))



(defun ask-query-and-pick (&optional (type "Sandwich"))
  (let (spose gpose av0)
    (setq av0 (copy-object (send *ri* :state :potentio-vector)))
    (setq spose (send *ri* :state :worldcoords))
    (setq gpose (cadr (assoc "POSE_SPOT" (car (typical-place-to-buy-omics type)) :test #'equal)))
    ;;(setq gpose (send *scene* :spot "/eng2/2f/subway-lunchset"))
    ;;
    (send *ri* :move-to gpose)
    ;;
    (pickup-sandwich) ;; larm
    ;;
    (send *ri* :move-to spose)
    ;;
    (hand-over :larm)
  ))

(defun tmp-demo-0920 ()
  (speak-jp "ごようけんはなんですか")
  (unix::sleep 3)
  (let (spose gpose av0 (type "Sandwich"))
    (setq av0 (copy-object (send *ri* :state :potentio-vector)))
    (setq spose (send *ri* :state :worldcoords))
    (setq *opose* spose)
    ;; go to fridge, and check inside
    (demo-open-fridge (send *scene* :spot "/eng2/7f/room73B2-fridge-front")
		      :go-back nil :pickup t)
    ;;
    (setq gpose (cadr (assoc "POSE_SPOT" (car (typical-place-to-buy-omics type)) :test #'equal)))
    ;; rarm
    (send *ri* :move-to gpose)
    ;; rarm
    (pickup-sandwich) ;; use larm to pickup
    ;; rarm
    (send *ri* :move-to spose)
    ;; rarm
    (hand-over :larm)
    ))

(defun get-place-to-search (&key (type "Cup"))
  (let ((info (find-knowrob-objects-with-info "Cup")))
    (mapcar #'(lambda(x)(list (cadr (assoc "POSE_OBJ" x :test #'string=))
			      (cadr (assoc "POSE_SPOT" x :test #'string=))))
	    info))
  )

(defun demo (&rest args)
  (when (eq args nil)
    (warn "(:pick name)~%")
    (return-from demo t))
  ;;
  (when (member :pick args)
    (pr2-tuckarm-pose :larm)
    (change-inflation-range 0.35)
    (ask-query-and-move "Cup" (cadr (member :pick args))))
  ;;
  (when (member :sandwich args)
    (pr2-tuckarm-pose :rarm)
    (speak-jp "めいあいへるぷゆう") (unix::sleep 8)
    (ask-query-and-pick "Sandwich"))
  )

;;
(progn
  (init)  ;; for pr2 setup
  (init2) ;; for cup detection
)
(warn "
;;(demo :pick \"cmu\") ;; to start demo
;;(demo :sandwich) ;; to start demo
;;
;; to debug ...
;;(check :type \"cmu\")  ;; to start recognize
;;(pick *obj* :larm) ;; to pickup using openrave
;;(demo-open-fridge fridge-front) ;; to go and open the fridge
")


(defun daa ()
  (setq spose (send *ri* :state :worldcoords))
;;  (speak-jp "ごようけんはなんですか")
  (unix::sleep 3)
  ;;
;  (setq cand1 (send *scene* :spot "/eng2/7f/room73B2-front-kitchen-table"))
;  (send *ri* :move-to cand1)
;  (if (check :type "sandwich")
;      (pick :larm))
  ;;
  (demo-open-fridge (send *scene* :spot "/eng2/7f/room73B2-fridge-front")
		    :go-back nil :pickup "box")
  ;;
  (send *ri* :move-to spose)
  (hand-over :larm)
  )
