#!/usr/bin/env roseus

(load "package://jsk_maps/src/eng2-scene.l")
(load "package://pr2eus/pr2-interface.l")
(load "eus_json_prolog.l") ;; -> json_prolog/src

(ros::roseus "demo_main")

(defun init ()
  ;; env
;  (setq *scene* (make-eng2-scene))
  (setq *pr2* (pr2))
  (setq *ri* (instance pr2-interface :init))
  (setq *knowrob* (instance json_prolog :init "json_prolog"))
  ;; callbacks
  (ros::subscribe "/query" std_msgs::string #'identity)
  )

;; callback
(defun recieve-task (msg)
  )

;; ask to knowrob
(defun ask-query-and-move (msg)
  (let* (;(objtype (send msg :data))
		 ;(rdf-type (format nil "'http://ias.cs.tum.edu/kb/knowrob.owl#~a'" objtype))
		 solutions pose)
	(send *knowrob* :query
		  "owl_has(SPOT, 'http://ias.cs.tum.edu/kb/knowrob.owl#inFrontOf-Generally', 'http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#room73b2-sharp-52-aquostv')" :pose "SPOT")
;	(send *knowrob* :query
;		  "owl_has(SPOT, knowrob:'inFrontOf-Generally', jsk_map:'room73b2-sharp-52-aquostv')" :pose "SPOT")
	(setq solutions (send *knowrob* :solutions))
	(when solutions
	  (setq pose (cadr (assoc "POSE" (car solutions) :test #'string=)))
	  (send *ri* :move-to pose) ;; robot moves
	  )
  ))

;;
(init)
(ask-query-and-move nil)
;(ros::spin)
;(exit)

