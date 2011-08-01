#!/usr/bin/env roseus

;; this a map convert script for jsk_maps to ias_semantic_map
;; rosrun mod_semantic_map SemanticMapToOWL

;; call service
(ros::roseus-add-msgs "mod_semantic_map")
(ros::roseus-add-srvs "mod_semantic_map")
(setq *srvname* "/generate_owl_map")

(defun conversion-rule (obj)
  (let ((name (string-downcase (send obj :name)))
		(type (get obj :knowrob-type))
		(classtype (class obj)) ret)
  ;; igrnore
  (when (eq type :ignore) (return-from conversion-rule nil))

  (push
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
	((substringp "ball" name) "Ball"))
   ret)

   ;; load :knowrob-type
   (when (stringp type) (push type ret)) ;; use if exists
   (when (listp type) (dolist (ty type) (if (stringp ty) (push ty ret))))
   ;; coords is "Place" (spot)
   (when (eq classtype cascaded-coords) (push "Place" ret))
   ;; no candidate -> will not convert
   (setq ret (remove-if #'null ret))
   (when (null ret) (push "ConstructionArtifact" ret))

   ;; output is csv
   (subseq (apply #'concatenate string
				  (mapcar #'(lambda(x)(format nil ",~a" x)) ret)) 1)
   ))


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

(defun make-name-unique (objs)
  (let ((sorted
		 (sort (copy-list objs) #'string< #'(lambda(x)(send x :name))))
		cur (id 0))
	(dolist (item sorted)
	  (if (string= (send item :name) cur)
		  (send item :name (format nil "~a-~a" (send item :name) (incf id)))
		(setq cur (send item :name))))
	;; TODO loop this rule for ("a" "a" "a-1") -> ("a" "a-1" "a-1") if needed
	))

(defun make-all-bbox (lst)
  (cond ((atom lst)
		 (make-bounding-box (send lst :vertices)))
		(t
		 (let* ((boxes (mapcar #'make-all-bbox lst))
				(ret (car boxes)))
		   (dolist (bb (cdr boxes))
			 (setq ret (send ret :union bb)))
		   ret))
		))

(defun convert-obj->semantic-bbox (id obj type parent)
  (let ((semobj (instance mod_semantic_map::SemMapObject :init))
		dummy aabb tmp size cpos pos vc)
    (setq tmp (send obj :copy-worldcoords))
    (send obj :orient 0 :z :world)
	(send obj :worldpos) ;; omajinai
	(cond
	 ((find-method obj :box)
	  (setq aabb (send obj :box)))
	 ((find-method obj :bodies)
	  (print (send obj :bodies)) ;; for update vertices
	  (setq aabb (make-all-bbox (send obj :bodies))))
	  ;(setq aabb (make-bounding-box (flatten1 (send-all (send obj :bodies) :vertices)))))
	 (t
	  (setq aabb (make-bounding-box (list #f(0 0 0))))))
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

(defun dump-properties (obj fd)
  (let* ((properties (get obj :knowrob-properties)) (prop-str "") prop)
	(when (setq prop (remove-if-not #'(lambda(x)(or (stringp x) (numberp x)))
								properties :key #'cdr))
	  ;; data properties
	  (setq prop-str (apply #'concatenate string
			  (mapcar #'(lambda(x)(format nil ",[~a,~a,~s]" (car x)
			    (if (stringp (cdr x)) "string" "float") (cdr x)))
					  prop)))
	  (format fd "        data-properties: [~a]~%" (subseq prop-str 1)))
	(when (setq prop (remove-if #'(lambda(x)(or (stringp x) (numberp x)))
										properties :key #'cdr))
	  ;; object properties
	  (setq prop-str (apply #'concatenate string
			  (mapcar #'(lambda(x)(format nil ",[~a,~a]"
										  (car x) (send (cdr x) :name)))
					  prop)))
	  (format fd "        object-properties: [~a]~%" (subseq prop-str 1)))
	))

;; dump the information for ias_sem_map
;; height is for rooms or floors
(defun dump-obj->yaml (obj fd &key (view) (height)) ;; only objects
;  (if (null (send obj :parent)) (return-from dump-obj->yaml))
  (if (not (or (find-method obj :box)
			   (find-method obj :bodies)))
	  (return-from dump-obj->yaml))
  (when (and view (find-method obj :bodies)) (objects (list obj)))
  (let* ((type (conversion-rule obj))
		 (qu (matrix2quaternion (send obj :worldrot)))
		 (sobj (convert-obj->semantic-bbox 0 obj type 0)))
	(if (not type) (return-from dump-obj->yaml))
	(format fd "    ~a:~%" (send obj :name))
	(format fd "        frame_id: ~a~%" (send (send obj :parent) :name))
	(format fd "        type: [~a]~%" type)
	(dump-properties obj fd)
	(format fd "        translation: {x: ~a, y: ~a, z: ~a}~%"
			(elt (send sobj :pose) 3) (elt (send sobj :pose) 7)
			(if height (+ (elt (send sobj :pose) 11)
						  (/ (- (* 0.001 height) (send sobj :height)) 2))
			  (elt (send sobj :pose) 11)))
	(format fd "        rotation: {w: ~a, x: ~a, y: ~a, z: ~a}~%"
			(elt qu 0) (elt qu 1) (elt qu 2) (elt qu 3))
	(format fd "        width: ~a~%        depth: ~a~%        height: ~a~%"
			(send sobj :width) (send sobj :depth)
			(if height (* 0.001 height) (send sobj :height)))
	))
(defun dump-spot->yaml (spot fd) ;; only cascoords
  (if (not (and (eq (class spot) cascaded-coords)
				(eps= 0.0 (elt (send spot :worldpos) 2) 50)))
	  (return-from dump-spot->yaml))
  (let* ((type (conversion-rule spot))
		 (pos (scale 0.001 (send spot :worldpos)))
		 (qu (matrix2quaternion (send spot :worldrot))))
	(if (not type) (return-from dump-spot->yaml))
	(format fd "    ~a:~%" (send spot :name))
	(format fd "        frame_id: ~a~%" (send (send spot :parent) :name))
	(dump-properties spot fd)
	(format fd "        translation: {x: ~a, y: ~a, z: ~a}~%"
			(elt pos 0) (elt pos 1)	0.0) ;; global pose
	(format fd "        rotation: {w: ~a, x: ~a, y: ~a, z: ~a}~%"
			(elt qu 0) (elt qu 1) (elt qu 2) (elt qu 3)))
  )

(defun dump-building->yaml (filename scene type &key (view nil))
  (let* ((floors (send scene :floors)) (rooms (send scene :rooms))
		 (objs (set-difference (send scene :objects) (append floors rooms)))
		 spots)
	(dolist (rm rooms)
	  (when (derivedp rm scene-model)
		(setq objs (append objs (send rm :objects)))))
	(setq spots (remove-if-not #'(lambda(x)(eq cascaded-coords (class x))) objs))
	(setq objs (remove-if #'(lambda(x)(eq cascaded-coords (class x))) objs))
	(make-name-unique (append floors rooms spots objs))
	;;
	(if (eq type :yaml)
		;; to yaml
		(progn
		  (with-open-file
		   (fd filename :direction :output)
		   (format fd "building:~%")
		   (dump-obj->yaml scene fd :view view)
		   (format fd "floor:~%")
		   (dolist (flo floors)
			 (dump-obj->yaml flo fd :view view :height 3000))
		   (format fd "room:~%")
		   (dolist (room rooms)
			 (dump-obj->yaml room fd :view view :height 3000))
		   (format fd "object:~%")
		   (dolist (obj objs)
			 (dump-obj->yaml obj fd :view view))
		   (format fd "spots:~%")
		   (dolist (spot spots)
			 (dump-spot->yaml spot fd))))
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
(defun srv ()
  (ros::roseus "hoge")
  (ros::wait-for-service *srvname*)
  (setq *scene* (make-eng2-scene))
  (dump-building->yaml "dummy" *scene* :srv))

(defun yaml (&optional (type :eng2) &key (filename "eng2.yaml") (view nil))
  (case type
	(:eng2
	 (make-default-scene :filename "eng2.yaml" :view view))
	(:scene1
	 (make-semantic-scene1 :filename "scene1.yaml" :view view))
	))

(load "models/cup-object.l")
(defun make-default-scene (&key (filename "scene.yaml") (view nil))
  (let ((scene (make-eng2-scene)))
	(dump-building->yaml filename scene :yaml :view view)))

(defun make-semantic-scene1 (&key (filename "scene.yaml") (view nil))
  (flet ((add-to-room
		  (obj room)
		  (send room :assoc obj)
		  (when (derivedp room scene-model)
            (nconc (send room :objects) (list obj)))
		  (send obj :move-to (send room :copy-worldcoords) :world)))
	(let ((scene (make-eng2-scene))
		  73b2 73a3 83b1
		  cup1 cup2 cup3 cup4 cup5 cupf1 cupf2 cupf3 cupf4 cupf5)
	  ;;
	  ;; add objects and relations here.
	  ;;
	  (setq 73b2 (car (send scene :find-object "/eng2/7f/73b2")))
	  (setq kyuto (car (send scene :find-object "/eng2/7f/7a-mini-kitchen")))
	  (setq 73a3 (car (send scene :find-object "/eng2/7f/73a3")))
	  (setq 83b1 (car (send scene :find-object "/eng2/8f/83b1")))
	  (setq cup1 (cup :name "cotesys") cup2 (cup :name "mit") cup3 (cup :name "tea") cup4 (cup :name "sumo") cup5 (cup :name "C") cup6 (cup :name "cmu"))
	  (setq cupf1 (make-cascoords :name "infront-of-cup")
                cupf2 (make-cascoords :name "infront-of-cup")
                cupf3 (make-cascoords :name "infront-of-cup")
                cupf4 (make-cascoords :name "infront-of-cup")
                cupf5 (make-cascoords :name "infront-of-cup")
		cupf6 (car (send scene :find-object "/eng2/7f/mini-kitchen-A-inside")))
	  ;;
	  (add-to-room cup1 73a3)
	  (add-to-room cupf1 73a3)
	  (send cup1 :translate (float-vector 4350 700 700))
	  (send cupf1 :translate (float-vector 4350 1400 0))
	  (send cupf1 :orient -pi/2 :z)
	  (push-knowrob-properties cupf1 "inFrontOf-Generally" cup1)
	  (push-knowrob-properties cup1 "linkToImageFile" "http://jsk-ros-pkg.svn.sourceforge.net/viewvc/jsk-ros-pkg/trunk/demos/jsk_2011_07_pr2_semantic/data/cup-cotesys.jpg")
	  ;;
	  (add-to-room cup2 73b2)
	  (add-to-room cup3 73b2)
	  (add-to-room cup4 73b2)
	  (add-to-room cupf2 73b2)
	  (add-to-room cupf3 73b2)
	  (add-to-room cupf4 73b2)
	  (send cup2 :translate (float-vector 5150 1550 700))
	  (send cup3 :translate (float-vector 1600 1850 800))
	  (send cup4 :translate (float-vector 4550 -2550 700))
	  (send cupf2 :translate (float-vector 5150 2000 0))
	  (send cupf2 :orient -pi/2 :z)
	  (send cupf3 :translate (float-vector 2200 1800 0))
	  (send cupf3 :orient -pi :z)
	  (send cupf4 :translate (float-vector 4550 -1900 0));;-150
	  (send cupf4 :orient -pi/2 :z)
	  (push-knowrob-properties cupf2 "inFrontOf-Generally" cup2)
	  (push-knowrob-properties cupf3 "inFrontOf-Generally" cup3)
	  (push-knowrob-properties cupf4 "inFrontOf-Generally" cup4)
	  (push-knowrob-properties cup2 "linkToImageFile" "http://jsk-ros-pkg.svn.sourceforge.net/viewvc/jsk-ros-pkg/trunk/demos/jsk_2011_07_pr2_semantic/data/mug-mit.jpg")
	  (push-knowrob-properties cup3 "linkToImageFile" "http://jsk-ros-pkg.svn.sourceforge.net/viewvc/jsk-ros-pkg/trunk/demos/jsk_2011_07_pr2_semantic/data/cup-tea.jpg")
	  (push-knowrob-properties cup4 "linkToImageFile" "http://jsk-ros-pkg.svn.sourceforge.net/viewvc/jsk-ros-pkg/trunk/demos/jsk_2011_07_pr2_semantic/data/cup-sumo.jpg")
	  ;;
	  (add-to-room cup5 83b1)
	  (add-to-room cupf5 83b1)
	  (send cup5 :translate (float-vector 3300 -300 700))
	  (send cupf5 :translate (float-vector 2800 -300 0))
	  (push-knowrob-properties cupf5 "inFrontOf-Generally" cup5)
	  (push-knowrob-properties cup5 "linkToImageFile" "http://jsk-ros-pkg.svn.sourceforge.net/viewvc/jsk-ros-pkg/trunk/demos/jsk_2011_07_pr2_semantic/data/cup-chen.jpg")
	  ;;
	  (add-to-room cup6 kyuto)
          (add-to-room cupf6 kyuto)
	  (send cup6 :translate (float-vector 1800 600 700))
	  (push-knowrob-properties cupf6 "inFrontOf-Generally" cup6)
	  (push-knowrob-properties cup6 "linkToImageFile" "http://jsk-ros-pkg.svn.sourceforge.net/viewvc/jsk-ros-pkg/trunk/demos/jsk_2011_07_pr2_semantic/data/cup-cmu.jpg")
	  ;;
	  (dump-building->yaml filename scene :yaml :view view)
	  scene
	  )))

(format t "type (yaml) or (srv)~%")
