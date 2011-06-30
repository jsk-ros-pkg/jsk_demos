#!/usr/bin/env roseus

;; this a map convert script for jsk_maps to ias_semantic_map
;; rosrun mod_semantic_map SemanticMapToOWL

;; now only for jsk internal
;#-:jsk
;(jsk)
;#-:rbrain
;(rbrain)

;; call service
(ros::roseus-add-msgs "mod_semantic_map")
(ros::roseus-add-srvs "mod_semantic_map")
(setq *srvname* "/generate_owl_map")

(defun conversion-rule (name)
  (cond
    ;; object types
   ((substringp "knob" name) "ControlKnob") ;; knobs turned with one's fingers to control devices
   ((substringp "hinge" name) "HingedJoint") ;; a hinged connection
   ((substringp "handle" name) "Handle")
   ((substringp "cupboard" name) "Cupboard")
   ((substringp "desk" name) "Table-PieceOfFurniture")  
   ((substringp "table" name) "Table-PieceOfFurniture")
   ((substringp "door" name) "Door")
   ((substringp "drawer" name) "Drawer")
   ((substringp "oven" name) "Oven")
   ((substringp "refrigerator" name) "Refrigerator")
   ((substringp "fridge" name) "Refrigerator")
   ((substringp "dishwasher" name) "Dishwasher")
   ((substringp "countertop" name) "CounterTop")
   ((substringp "sink" name) "Sink")
   ((substringp "stairs" name) "Stairway")
   ((substringp "washing-machine" name) "ClothesWasher")
   ((substringp "refrigerator" name) "Refridgerator")
   ((substringp "bed" name) "Bed-PieceOfFurniture")
   ((substringp "microwave-oven" name) "MicrowaveOven")
   ((substringp "trashbox" name) "TrashContainer") ;; for those in the hallway / WasteBasket small baskets
   ((substringp "sofa" name) "Sofa-PieceOfFurniture")
   ((substringp "book_shelf" name) "ShelfInABuilding")
   ((substringp "audio_shelf" name) "ShelfInABuilding")
   ((substringp "shelf" name) "ShelfInABuilding")
   ((substringp "bariera" name) "ShelfInABuilding")
   ((substringp "panel-wall" name) "WallOfAConstruction")
   ((substringp "dining-table" name) "DiningRoomTable") ;; or EatingTable?
   ((substringp "television" name) "TelevisionSet")
   ((substringp "cabinet" name) "Cabinet-PieceOfFurniture")

   ;; portable objects
   ((substringp "sushi-cup" name) "Bowl-Eating") ;; its important that XXX-cup or cup-XXX is before cup
   ((substringp "mug-cup" name) "DrinkingMug") 
   ((substringp "cup-saucer" name) "Sauce")
   ((substringp "cup" name) "Cup")
   ((substringp "book" name) "BookCopy")
   ((substringp "chair" name) "Chair-PieceOfFurniture")
   ((substringp "chessboard" name) "ChessBoard")
   ((substringp "cell-phone" name) "CellularTelephone")
   ((substringp "frying-pan" name) "OmelettePan")
   ((substringp "kettle" name) "Kettle")
   ((substringp "knife" name) "Knife")
   ((substringp "iron" name) "ClothesIron")
   ((substringp "pockey" name) "Refrigerator")
   ((substringp "sponge" name) "Sponge-CleaningImplement")
   ((substringp "tray" name) "Tray")
   ((substringp "ball" name) "Ball")

   ;; room types
   ;;((substringp "???" name) "RoomInAConstruction") ;; room without specific type
   ;;((substringp "???" name) "Restroom") 
   ;;((substringp "???" name) "PublicRestroom") 
   ;;((substringp "???" name) "MensRoom") 
   ;;((substringp "???" name) "UnisexRoom")    
   ;;((substringp "???" name) "WomensRoom") 
   ;;((substringp "???" name) "BathRoom") 
   ;;((substringp "???" name) "Kitchen") 
   ;;((substringp "???" name) "Nursery-BabysRoom")    
   ;;((substringp "???" name) "StudyRoom") 
   ;;((substringp "???" name) "Lobby-Room") 
   ;;((substringp "???" name) "Closet") 
   ;;((substringp "???" name) "HospitalRoom")    
   ;;((substringp "???" name) "DiningRoom") 
   ;;((substringp "???" name) "UtilityRoom") 
   ;;((substringp "???" name) "StorageRoom") 
   ;;((substringp "???" name) "BedRoom")    
   ;;((substringp "???" name) "OfficeRoom") 
   ;;((substringp "???" name) "ProfessorsOffice")
   ;;((substringp "???" name) "LaboratoryRoom") 
   ;;((substringp "???" name) "Workshop")    
   ;;((substringp "???" name) "ServerRoom") 
   ;;((substringp "???" name) "ClassRoom") 
   ;;((substringp "???" name) "LectureHall") 
   ;;((substringp "???" name) "LibraryRoom")    
   ;;((substringp "???" name) "PersonAssemblyRoom") 
   ;;((substringp "???" name) "AuditoriumRoom") 
   ;;((substringp "???" name) "MeetingSpace") 
   ;;((substringp "???" name) "ConferenceRoom")    
   ;;((substringp "???" name) "RestaurantSpace") 
   ;;((substringp "???" name) "FastFoodRestaurantSpace") 
   ;;((substringp "???" name) "CoffeeShopSpace") 

   ;; level / floor types
   ;;((substringp "???" name) "LevelOfAConstruction") ;; level without specific type
   ;;((substringp "???" name) "AboveGroundLevelInAConstruction") ;; 1st floor or above, ie 2,3, etc
   ;;((substringp "???" name) "BasementLevelInAConstruction") ;; basement level, eg b1, b2, etc
   
   ;; building type
   ;;((substringp "???" name) "Building") 

   (t "ConstructionArtifact") ;; no candidate -> will not convert
  ))

(defun convert-obj->semantic-bbox (id obj type parent)
  (let ((semobj (instance mod_semantic_map::SemMapObject :init))
		dummy aabb tmp size cpos pos vc)
    (setq tmp (send obj :copy-worldcoords))
    (send obj :orient 0 :z :world)
	(send obj :worldpos) ;; omajinai
	(if (find-method obj :box)
		(setq aabb (send obj :box))
	  (progn
		(print (send obj :bodies))
		(setq aabb (make-bounding-box (flatten (send-all (send obj :bodies) :vertices))))))
    (setq size (scale 0.001 (v- (send aabb :maxpoint) (send aabb :minpoint))))
	(setq cpos (scale 0.5 (v+ (send aabb :maxpoint) (send aabb :minpoint))))
	(setq vc (v- cpos (send tmp :worldpos)))
	(setq pos (scale 0.001 (v+ (send tmp :worldpos) (send tmp :rotate-vector vc))))
    (send obj :move-to (send tmp :copy-worldcoords) :world)
    ;;
    (send semobj :id id)
    (send semobj :type type)
	(send tmp :locate pos :world)
    (send semobj :pose ;; 4x4 matrix -> size =16
		  (mapcan #'(lambda(x)(coerce (matrix-row (send tmp :4x4) x) cons))
				  (list 0 1 2 3)))
    (send semobj :width  (elt size 0)) ;; x
    (send semobj :depth  (elt size 1)) ;; y
    (send semobj :height (elt size 2)) ;; z
    (send semobj :partof parent) ;; parent id
	;;
    (format t ";; convert id:~a obj:~a type:~a par:~a~%" id obj type parent)
    semobj
    ))



(defun convert-objs->semobjs (objs &optional (pid 0))
  (let ((semobjs nil) type (id 0))
    (dolist (ob objs)
	  (when (or (find-method ob :box) (find-method ob :bodies))
		(progn
		  (setq type (conversion-rule (string-downcase (send ob :name))))
		  (when type
			(incf id)
			(push (convert-obj->semantic-bbox id ob type pid) semobjs)
;;			(append semobjs (convert-objs->semobjs (send ob :descendants) id))
			))))
    semobjs ))

;; dump the information for ias_sem_map
(defun dump-obj->yaml (obj id fd &key (view nil)) ;; only objects
  (if (null (send obj :parent)) (return-from dump-obj->yaml))
  (if (not (or (find-method obj :box) (find-method obj :bodies)))
	  (return-from dump-obj->yaml))
  (when view (irt-objects (list obj)))
  (let* ((type (conversion-rule (string-downcase (send obj :name))))
		 (pos (scale 0.001 (send obj :pos)))
		 (qu (matrix2quaternion (send obj :rot)))
		 (sobj (convert-obj->semantic-bbox 0 obj type 0)))
	(format fd "    ~a~a:~%" (send obj :name) (if id id ""))
	(format fd "        frame_id: ~a~%" (send (send obj :parent) :name))
	(format fd "        type: [~a]~%" type)
	(format fd "        translation: {x: ~a, y: ~a, z: ~a}~%"
			(elt (send sobj :pose) 3) (elt (send sobj :pose) 7)	(elt (send sobj :pose) 11)) ;; global pose
;			(elt pos 0) (elt pos 1) (elt pos 2)) local pose
	(format fd "        rotation: {w: ~a, x: ~a, y: ~a, z: ~a}~%"
			(elt qu 0) (elt qu 1) (elt qu 2) (elt qu 3))
	(format fd "        width: ~a~%        depth: ~a~%        height: ~a~%"
			(send sobj :width) (send sobj :depth) (send sobj :height))
	))
(defun dump-room->yaml (room fd) ;; only rooms
  (let* ((type (conversion-rule (string-downcase (send room :name))))
		 (pos (scale 0.001 (send room :pos)))
		 (qu (matrix2quaternion (send room :rot)))
		 (sobj (convert-obj->semantic-bbox 0 room type 0)) vs w1 w2 d)
	(setq vs (mapcar #'(lambda(x)(scale 0.001 (v- x (send room :worldpos))))
					 (send room :vertices)))
	(setq w1 (apply #'max (mapcar #'(lambda(x)(elt x 1)) vs)))
	(setq w2 (apply #'min (mapcar #'(lambda(x)(elt x 1)) vs)))
	(setq d1 (apply #'max (mapcar #'(lambda(x)(elt x 0)) vs)))
	(setq d2 (apply #'min (mapcar #'(lambda(x)(elt x 0)) vs)))
	(format fd "    ~a:~%" (send room :name))
	(format fd "        frame_id: ~a~%" (send (send room :parent) :name))
	(format fd "        type: [~a]~%" type)
	(format fd "        translation: {x: ~a, y: ~a, z: ~a}~%"
			(elt (send sobj :pose) 3) (elt (send sobj :pose) 7)	(elt (send sobj :pose) 11)) ;; global pose
;			(elt pos 0) (elt pos 1) (elt pos 2)) ;local pose
	(format fd "        rotation: {w: ~a, x: ~a, y: ~a, z: ~a}~%"
			(elt qu 0) (elt qu 1) (elt qu 2) (elt qu 3))
	(format fd "        region: [[~a,~a],[~a,~a],[~a,~a],[~a,~a]]~%"
			d1 w2 d1 w1 d2 w1 d2 w2)
	))


(defun dump-building->yaml (filename scene type &key (view nil))
  (let* ((floors (send scene :floors)) (rooms (send scene :rooms)) (id 0)
		 (objs (set-difference (send scene :objects) (append floors rooms))))
	(dolist (rm rooms)
	  (when (derivedp rm scene-model)
		(setq objs (append objs (send rm :objects)))))
   	(setq *objs* objs)
	(if (eq type :yaml)
		;; to yaml
		(progn
		  (with-open-file
		   (fd filename :direction :output)
		   (format fd "map_frame: ~a~%" (send scene :name))
		   (format fd "floor:~%")
		   (dolist (flo floors)
;;			 (dump-room->yaml flo fd))
			 (dump-obj->yaml flo nil fd :view view))
		   (format fd "room:~%")
		   (dolist (room rooms)
;;			 (dump-room->yaml room fd))
			 (dump-obj->yaml room nil fd :view view))
		   (format fd "object:~%")
		   (dolist (obj objs)
			 (dump-obj->yaml obj (incf id) fd :view view))))
	  ;; to service
	  (call-mod-semmap (append floors rooms objs)))
	))

(defun call-mod-semmap (objs)
  (setq *id* 0)
  (let ((req (instance mod_semantic_map::GenerateSemanticMapOWLRequest :init))
	res (offset 0))
	(send req :map :header :frame_id "http://www.jsk.t.u-tokyo.ac.jp/jsk_msp.owl#")
	(send req :map :header :stamp (ros::time-now))
	(setq semobjs (convert-objs->semobjs objs)) ;; convert
	(send req :map :objects semobjs)
	(setq res (ros::service-call *srvname* req))
	(send res :owlmap) ;; output xml string
    ))



(load "package://jsk_maps/src/eng2-scene.l")
(unless (boundp '*scene*) (setq *scene* (make-eng2-scene)))
(defun srv ()
  (ros::roseus "hoge")
  (ros::wait-for-service *srvname*)
  (dump-building->yaml "dummy" *scene* :srv))

(defun yaml (&key (filename "eng2.yaml") (view nil))
  (dump-building->yaml filename *scene* :yaml :view view))

(format t "type (yaml) or (srv)~%")
