#!/usr/bin/env roseus

(load "package://jsk_maps/src/eng2-scene.l")
(load "package://pr2eus/pr2-interface.l")
(load "eus_json_prolog.l") ;; -> json_prolog/src

(ros::roseus "demo_main")

(defun init ()
  ;; env
  (setq *scene* (make-eng2-scene))
  (setq *pr2* (pr2))
  (setq *ri* (instance pr2-interface :init))
  (setq *knowrob* (instance json_prolog :init))
  ;; callbacks
  (ros::subscribe "/query" std_msgs::string #'ask-query)
  )

;; callback
(defun recieve-task (msg)
  )

;; ask to knowrob
(defun ask-query-and-move (msg)
  (let* ((objtype (send msg :data))
		 (rdf-type (format nil "'http://ias.cs.tum.edu/kb/knowrob.owl#~a'" objtype))
		 solutions pose)
	(send *knowrob* :query
		  (list->prolog
		   `(rdfs_individual_of :obj ,rdf-type)) :pose "OBJ")
	(setq solutions (send *knowrob* :solutions))
	(when solutions
	  (setq pose (cadr (assoc (car solutions) "POSE" sol :test #'string=)))
	  (send *ri* :move-to pose) ;; robot moves
	  )
  ))

;;
(init)
(ros::spin)
(exit)
