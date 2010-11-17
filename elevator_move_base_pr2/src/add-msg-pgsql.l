;; postgresql Euslisp interface
(require :pqsql (ros::resolve-ros-path "package://euslisp/jskeus/eus/lib/llib/pgsql.l"))

(defun init-pgsql-safe (&key (host "localhost") (port 0) (options 0) (tty 0)
			     (dbname (cond ((unix:getenv "PGDATABASE"))
					   (t (unix:getenv "USER"))))
			     (user (cond ((unix:getenv "PGUSER"))
					 (t (unix:getenv "USER"))))
			     (pass 0))
  (let ((connection (pq::setdblogin host port options tty dbname user pass)))
    (when (zerop (pqstatus connection))
      (instance pgsql :init :host host :port port :options options
		:tty tty :dbname dbname :user user :pass pass)
      )))

;; initialize db connection
(unless (boundp '*db*)
  (setq *db* (init-pgsql-safe :dbname "pr2db" :user "pr2admin" :host "pr1012"))
  (if (null *db*) (warn "pgsql can't open.")))

;; overwrite function
(defun tables (db)
  (pq:query db nil "select tablename from pg_tables where schemaname='public';"))

;; insert-trial-table (db text rostime)
;; insert-tf-table (db msg)
;; insert-tf-table-by-coords (db stamp parent child coords)

(defun create-trial-table (db)
  (pq:query db nil "CREATE TABLE trial (id serial,type text,stamp bigint);"))
(defun insert-trial-table (db text rostime)
  (when db
    (pq:insert-record2 db "trial" '(type stamp)
		       (list text (format nil "~d~0,9d" (send rostime :sec)
					  (send rostime :nsec))))))

(defun _db-values-by-accessors (msg accessors)
  (mapcar #'(lambda(acc)
	      (let ((m msg))
		(dolist (a acc m)
		  (setq m (if (keywordp a) (send m a) (elt m a)))
		  (when (eq (class m) time)
		    (setq m (send m :to-nsec))))))
	  accessors))

(defun _create-msg-table (db table fields types)
  (let (definition str)
    (while (or fields types)
      (push "," definition)
      (push (car fields) definition)
      (push " " definition)
      (push (car types) definition)
      (setq fields (cdr fields) types (cdr types)))
    (setq str (apply #'concatenate string (mapcar #'string-downcase (reverse definition))))
    (pq:query db nil (format nil "CREATE TABLE ~a (id serial ~a);" table str))
    ))

(defun _insert-msg-table (db table msg fields accessors)
  (let ((values (_db-values-by-accessors msg accessors)))
    (when db
      (pq:insert-record2 db table fields values))
    ))

;;
(defun create-tf-table (db &key (table "tf"))
  (let ((fields
	 '(header_stamp header_frame_id child_frame_id
	   transform_translation_x transform_translation_y
	   transform_translation_z transform_rotation_x
	   transform_rotation_y transform_rotation_z transform_rotation_w))
	(types '(bigint text text real real real real real real real)))
    ;; create table
    (_create-msg-table db table fields types)
    ;; add index on timestamp
    (pq:query db nil (format nil "CREATE INDEX ~A_stamp_idx ON ~A(header_stamp);" table table))
    ))

(defun insert-tf-table (db msg &key (table "tf"))
  (let ((fields
	 '(header_stamp header_frame_id child_frame_id
	   transform_translation_x transform_translation_y
	   transform_translation_z transform_rotation_x
	   transform_rotation_y transform_rotation_z transform_rotation_w))
	(accessors
	 '((:header :stamp) (:header :frame_id) (:child_frame_id)
	   (:transform :translation :x) (:transform :translation :y)
	   (:transform :translation :z)
	   (:transform :rotation :x) (:transform :rotation :y)
	   (:transform :rotation :z) (:transform :rotation :w))))
    (_insert-msg-table db table msg fields accessors)
    ))


(defun insert-tf-table-by-coords (db stamp source-obj target-obj
				     &key source_id target_id (table "tf"))
  (let* ((coords (send (make-coords);;(send target-obj :copy-worldcoords)
		       :transform (send source-obj :transformation target-obj) :world))
	 (parent (or source_id (send source-obj :name)))
	 (child  (or target_id (send target-obj :name)))
	 (msg (ros::coords->tf-transform-stamped coords parent child)))
    (send msg :header :stamp stamp)
    (insert-tf-table db msg :table table)
    ))

(defun select-tf-from-table (db &key frame_id child_id after before (limit 1) (table "tf") (with-time nil))
  (when (derivedp after time) (setq after (send after :to-nsec)))
  (when (derivedp before time) (setq before (send before :to-nsec)))
  (let ((fields
	 '(header_stamp header_frame_id child_frame_id
	   transform_translation_x transform_translation_y
	   transform_translation_z transform_rotation_x
	   transform_rotation_y transform_rotation_z transform_rotation_w))
	(cond1 (if frame_id (list '= 'header_frame_id frame_id)))
	(cond2 (if child_id (list '= 'child_frame_id child_id)))
	(cond3 (if after (list '> 'header_stamp after)))
	(cond4 (if before (list '< 'header_stamp before)))
	conslist where-cond result qu co
	return-list)
    (setq condlist (delete nil (list cond1 cond2 cond3 cond4)))
    (setq where-cond (cond ((<= 2 (length condlist))
			    (append '(and) condlist))
			   ((car condlist) (car condlist))
			   (t "")))
    (setq result (select db fields table :where where-cond :order-by "header_stamp DESC" :limit limit))
    (when result
      (dolist (record result)
	(setq qu (subseq record 6))
	(setq co (make-coords :pos (scale 1000 (coerce (subseq record 3 6) float-vector))
			      :rot (quaternion2matrix (float-vector (elt qu 3) (elt qu 0) (elt qu 1) (elt qu 2)))))
	(if with-time
	    (push (cons co (send (ros::time) :from-nsec (elt record 0))) return-list)
	  (push co return-list)))
      return-list)
    ))

;; ros message types -> bool int8 uint8 int16 uint16 int32 uint32 int64 uint64
;;                      float32 float64 string time duration and []


;; publish posearray for "success" "fail"
(defun publish-posearray (topic-name type)
  (let ((frame_id "/eng8") tmlist tflist
	(msg (instance geometry_msgs::PoseArray :init)))

    (ros::advertise topic-name geometry_msgs::PoseArray 1)
    (unix::sleep 1)

    (setq tmlist (flatten (pq:query *db* nil (format nil "SELECT stamp FROM trial WHERE type LIKE '~A%';" type))))
    (setq tflist (mapcan #'(lambda(tm)(select-tf-from-table *db* :before tm :frame_id frame_id)) tmlist))

    (send msg :header :stamp (ros::time-now))
    (send msg :header :frame_id frame_id)
    (send msg :poses (mapcar #'ros::coords->tf-pose tflist))

    (ros::publish topic-name msg)
))

