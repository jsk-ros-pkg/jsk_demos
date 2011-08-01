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

;; call after inserting the robot position
(defun similar-objects-with-info-by-distance
  (&optional (objtype "Refrigerator") (k 5))
  (let ((rdf-type (format nil "'http://ias.cs.tum.edu/kb/knowrob.owl#~a'" objtype))
	(infront-of "'http://ias.cs.tum.edu/kb/knowrob.owl#inFrontOf-Generally'")
	(robot "'http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#pr2'"))
    (send *prolog* :query
	  (list->prolog
	   `(,(format nil "_R=~a" robot)
	     (setof :_O ("lookForKMostSimilarObjT" ,rdf-type ,k :_O) :_OBJS)
	     (get_sorted_path_costs ,robot :_OBJS :_SORTED)
	     (member #(:COST :OBJ)  :_SORTED)
	     (owl_has :spot ,infront-of :obj)
	     (object_info :obj "[IMAGE,ROOM,LEVEL|_]")))
	  :pose '("OBJ" "SPOT"))
    ))

(defun find-typical-place-omics
  (&optional (objtype "Cup"))
  (let* ((knowrob "http://ias.cs.tum.edu/kb/knowrob.owl")
	 (rdftype  (format nil "'~a#~a'" knowrob objtype))
	 (omicsloc (format nil "'~a#OmicsLocations'" knowrob))
	 (incont  (format nil "'~a#in-ContGeneric'" knowrob))
	 (infront-of (format nil "'~a#inFrontOf-Generally'" knowrob))
	 (robot "'http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#pr2'")
	 (place  (format nil "'~a#Place'" knowrob)))
    (send *prolog* :query
      (list->prolog
      `((findall #(:_P :_X) (bayes_probability_given ,omicsloc :_X ,rdftype :_P) :_Ps)
        (sort :_Ps :_Sorted)
        (reverse :_Sorted :_Rev)
        (nth0 0 :_Rev #(:Prob :Type))
        (findall :_R (owl_individual_of :_R :Type) :_Rs)
	(get_sorted_path_costs ,robot :_Rs :_SortRs)
        (member :Room :_SortRs)
        (rdf_triple ,incont :Spot :Room)
        (rdf_has :Spot "rdf:type" ,place)
        (owl_has :Spot ,infront-of :obj)
	("lookForObjT" ,rdftype :obj)
        (object_info :obj "[IMAGE|_]")))
      :pose '("OBJ" "SPOT"))
    ))

;; initialize and print usage
(ros::roseus "eus_prolog_sample")
;(setq *prolog* (instance json_prolog :init "knowrob_server"))
(setq *prolog* (instance json_prolog :init "json_prolog"))
(send *prolog* :query "scene1.") ;; for loading scene1.owl
(format t ";;(prolog-sample1) (prolog-sample2) (prolog-sample3)")
(format t ";;(find-knowrob-objects \"DrinkingMug\")~%")
(format t ";;(find-knowrob-objects-with-info \"Cup\")~%")
(format t ";;(find-knowrob-objects-with-info \"Table-PieceOfFurniture\")~%")
(format t ";;(find-similar-objects-with-info \"Pitcher\")~%")