#!/usr/bin/env roseus

;; this a map convert script for jsk_maps to ias_semantic_map
;; rosrun mod_semantic_map SemanticMapToOWL

;; now only for jsk internal
#-:jsk
(jsk)
#-:rbrain
(rbrain)

;; call service
(ros::roseus-add-msgs "mod_semantic_map")
(ros::roseus-add-srvs "mod_semantic_map")
(setq *srvname* "/generate_owl_map")

(ros::roseus "hoge")
;(ros::wait-for-service *srvname*)

(defun convert-obj->semantic-bbox (id obj type parent)
  (let ((semobj (instance mod_semantic_map::SemMapObject :init))
	aabb tmp)
    (setq tmp (send obj :copy-worldcoords))
    (send obj :move-to (make-coords) :world)
    (setq aabb (send obj :box))
    (send obj :move-to tmp :world)
    ;;
    (send semobj :id id)
    (send semobj :type type)
    (setq tmp (scale 0.001 (v- (send aabb :maxpoint) (send aabb :minpoint))))
    (send semobj :depth  (elt tmp 0)) ;; x
    (send semobj :width  (elt tmp 1)) ;; y
    (send semobj :height (elt tmp 2)) ;; z
    (send semobj :pose ;; 4x4 matrix -> size =16
	  (mapcan #'(lambda(x)(coerce (matrix-row (send obj :4x4) x) cons))
		  (list 0 1 2 3)))
    (send semobj :partof parent) ;; parent id
    (format t ";; convert id:~a obj:~a type:~a par:~a~%" id obj type parent)
    semobj
    ))

(defun convert-objs->semobjs (objs &optional (pid 0))
  (let ((semobjs nil) id)
    (dolist (ob objs)
      (dolist (type (list "knob" "hinge" "handle" "table" "door" "cupboard"
			  "drawer" "oven" "refrigerator" "dishwasher"
			  "countertop" "sink"))
	;; this is rule for convert type from object name
	(when (substringp type (string-downcase (send ob :name)))
	  (setq id (incf *id*))
	  (push (convert-obj->semantic-bbox id ob type pid) semobjs)
	  (append semobjs (convert-objs->semobjs (send ob :descendants) id))
	  )))
    semobjs ))


(defun call-mod-semmap (objs)
  (setq *id* 0)
  (let ((req (instance mod_semantic_map::GenerateSemanticMapOWLRequest :init))
	res (offset 0))
	(send req :map :header :frame_id "/map")
	(send req :map :header :stamp (ros::time-now))
	(setq semobjs (convert-objs->semobjs objs)) ;; convert
	(send req :map :objects semobjs)
	(setq res (ros::service-call *srvname* req))
	(send res :owlmap) ;; output xml string
    ))

(defun test (&optional (filename "output.xml"))
  (setq *map* (room73B2))
  (setq text (call-mod-semmap (send *map* :all-objects)))
  (with-open-file
   (fd filename :direction :output)
   (format fd text))
  )
