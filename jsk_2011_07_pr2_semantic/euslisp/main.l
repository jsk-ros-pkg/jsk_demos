#!/usr/bin/env roseus

(load "package://jsk_maps/src/eng2-scene.l")
(load "package://pr2eus/pr2-interface.l")
(load "eus_json_prolog.l") ;; -> json_prolog/src
(load "actions.l")

(ros::roseus "demo_main")

(defun init ()
  ;; env
  ;;  (setq *scene* (make-eng2-scene))
  (unless (boundp '*pr2*) (pr2))
  (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))
  (unless (boundp '*tfl*)(setq *tfl* (instance ros::transform-listener :init)))
  (setq *knowrob* (instance json_prolog :init "json_prolog"))
  ;; callbacks
  (ros::subscribe "/query" std_msgs::string #'identity)
  )

;; callback
(defun recieve-task (msg)
  )

(defun insert-robot-pose ()
  (let (pose po ret)
	(ros::spin-once)
	(setq pose (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)))
	(send pose :locate (scale 0.001 (send pose :worldpos)) :world)
	(setq po (coerce (array-entity (send pose :4x4)) cons))
	(setq po (mapcan #'(lambda(x)(coerce (format nil ",~a" x) cons)) po))
	(setq po (coerce (cdr po) string))
	(send *knowrob* :query "create_object_instance('http://ias.cs.tum.edu/kb/knowrob.owl#Robot',http://jsk.t.u-tokyo.ac.jp/knowrob.owl#'pr2')") ;; always success
	(setq ret (send *knowrob* :query
					(format nil "update_pose(pr2,[~a])" po)))
	ret ))

;; ask to knowrob
(defun ask-query-and-move (type)
  (let* (;(type (send msg :data))
		 (infront-of "'http://ias.cs.tum.edu/kb/knowrob.owl#inFrontOf-Generally'")
		 (rdf-type (format nil "'http://ias.cs.tum.edu/kb/knowrob.owl#~a'" type))
		 (linktype "'http://ias.cs.tum.edu/kb/knowrob.owl#linkToImageFile'")
		 solutions pose)
	(setq solutions
		  (send *knowrob* :query
				(list->prolog
				 (list `(rdfs_individual_of :obj ,rdf-type)
					   `(owl_has :spot ,infront-of :obj)
					   (format nil "(owl_has(OBJ,~a,URL);URL='')" linktype)
					   ))
				:pose "SPOT"))
	(when solutions
	  (setq solutions (list (elt solutions 1)))
	  (setq obj (cadr (assoc "OBJ" (car solutions) :test #'string=)))
	  (setq robot-pose (cadr (assoc "POSE" (car solutions) :test #'string=)))
	  (send *ri* :move-to robot-pose) ;; robot moves
	  (ros::spin-once)
	  (send *pr2* :move-to (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)) :world)
	  (send *knowrob* :query (format nil "OBJ='~a'" obj) :pose "OBJ" :mode :incremental)
	  (setq obj-pose (cadr (assoc "POSE" (car (send *knowrob* :solutions)) :test #'string=)))
	  (send *pr2* :head :look-at (send obj-pose :worldpos))
	  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
	  (send *ri* :wait-interpolation)
	  ;; grasp cup here
	  ;; go to somewhere
	  )
  ))

;;
(init)  ;; for pr2 setup
(init2) ;; for cup detection
;(ask-query-and-move "Cup")
;(ros::spin)
;(exit)

