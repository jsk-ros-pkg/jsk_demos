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
  (setq *knowrob* (instance json_prolog :init "json_prolog"))
  ;; callbacks
  (ros::subscribe "/query" std_msgs::string #'identity)
  )

;; callback
(defun recieve-task (msg)
  )

;; ask to knowrob
(defun ask-query-and-move (type)
  (let* (;(type (send msg :data))
		 (infront-of "'http://ias.cs.tum.edu/kb/knowrob.owl#inFrontOf-Generally'")
		 (rdf-type (format nil "'http://ias.cs.tum.edu/kb/knowrob.owl#~a'" type))
		 solutions pose)
	(send *knowrob* :query
		  (list->prolog
		   (list `(rdfs_individual_of :obj ,rdf-type)
				 `(owl_has :spot ,infront-of :obj)))
		  :pose "SPOT")
	(setq solutions (send *knowrob* :solutions))
	(when solutions
	  (setq pose (cadr (assoc "POSE" (car solutions) :test #'string=)))
	  (send *ri* :move-to pose) ;; robot moves
	  )
  ))

;;
(init)  ;; for pr2 setup
(init2) ;; for cup detection
;(ask-query-and-move "Cup")
;(ros::spin)
;(exit)

