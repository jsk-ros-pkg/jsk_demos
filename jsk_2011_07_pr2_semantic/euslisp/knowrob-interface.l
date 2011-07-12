;;
;; euslisp interface for json_prolog
;;
;; Author: Manabu Saito
;;
(load "package://json_prolog/src/euslisp/eus_json_prolog.l")

(defun find-knowrob-objects-with-info (&optional (objtype "Refrigerator"))
  (let ((rdf-type (format nil "'http://ias.cs.tum.edu/kb/knowrob.owl#~a'" objtype))
		(infront-of "'http://ias.cs.tum.edu/kb/knowrob.owl#inFrontOf-Generally'"))
	(send *prolog* :query
		  (list->prolog (list
		   `(setof :_o ("lookForObjT" ,rdf-type :_o) :_objs)
		   `(member :obj :_objs)
		   `(owl_has :spot ,infront-of :obj)
		   `(object_info :obj "[IMAGE,ROOM,LEVEL|_]")))
		  :pose '("OBJ" "SPOT"))
	))


(defun find-similar-objects-with-info (&optional (objtype "Refrigerator") (k 5))
  (let ((rdf-type (format nil "'http://ias.cs.tum.edu/kb/knowrob.owl#~a'" objtype))
		(infront-of "'http://ias.cs.tum.edu/kb/knowrob.owl#inFrontOf-Generally'"))
	(send *prolog* :query
		  (list->prolog (list
		   `(setof :_o ("lookForKMostSimilarObjT" ,rdf-type ,k :_o) :_objs)
		   `(member :obj :_objs)
		   `(owl_has :spot ,infront-of :obj)
		   `(object_info :obj "[IMAGE,ROOM,LEVEL|_]")))
		  :pose '("OBJ" "SPOT"))
	))

;; initialize and print usage
(ros::roseus "eus_prolog_sample")
;(setq *prolog* (instance json_prolog :init "knowrob_server"))
(setq *prolog* (instance json_prolog :init "json_prolog"))
(format t ";;(prolog-sample1) (prolog-sample2) (prolog-sample3)")
(format t ";;(find-knowrob-objects \"DrinkingMug\")~%")
(format t ";;(find-knowrob-objects-with-info \"Cup\")~%")
(format t ";;(find-knowrob-objects-with-info \"Table-PieceOfFurniture\")~%")
(format t ";;(find-similar-objects-with-info \"Pitcher\")~%")