(require :mongo-client "package://roseus_mongo/euslisp/mongo-client.l")

(defclass db-building-model
  :super building-model
  :slots (db-enabled-p))
(defmethod db-building-model
  (:init (&rest args)
   (send-super* :init args)
   (unless (boundp '*tfl*)
     (setq *tfl* (instance ros::transform-listener :init)))
   (setq db-enabled-p (mongo::wait-for-server 3)))
  (:split-floor-name (full-name)
   (when (or (not (stringp full-name))
             (null-string-p full-name))
     (ros::ros-error "invalid name for spot: ~A" name)
     (return-from :split-floor-name nil))
   (dolist (floor (send-all (send self :floors) :name))
     (when (string= floor (subseq full-name 0 (length floor)))
       (return-from :split-floor-name
         (cons floor (string-left-trim "/" (subseq full-name (length floor)))))))
   (ros::ros-error "spot name must contains floor: ~A"
                   (send-all (send self :floors) :name))
   (return-from :split-floor-name nil))
  (:db-remove-spot (spot-name)
   ;; just for assertion
   (unless (send self :split-floor-name spot-name)
     (return-from :db-remove-spot nil))
   (let ((latest (mongo::query geometry_msgs::PoseStamped
                               :meta `((:type . :spot)
                                       (:name . ,spot-name)
                                       (:latest . t)))))
     (unless latest
       (ros::ros-info "No data to be deleted")
       (return-from :db-remove-spot t))
     (ros::ros-info "~a documents found" (length latest))
     ;; dissoc
     (let ((s (send self :spot spot-name)))
       (send (send s :parent) :dissoc s)
       ;; update db
       (dolist (doc latest)
         (ros::ros-info "updated ~A" (cdadr (assoc :_id (cdr doc))))
         (ros::ros-info "updated meta: ~A" (remove :latest (cdr doc) :key #'car))
         (setf (cdr (assoc :latest (cdr doc))) nil)
         (mongo::update-by-id (cdadr (assoc :_id (cdr doc))) ;; _id
                              (car doc)  ;; msg
                              :meta (cdr doc)))
       s)))
  (:db-insert-spot (coords)
   "Insert spot coords. Coords must be transformation from /world frame"
   ;; invalidate previous data
   (unless (send self :db-remove-spot (send coords :name))
     (return-from :db-insert-spot nil))

   ;; insert the latest data
   (let* ((floor (car (send self :split-floor-name (send coords :name))))
          (spot-cds (send *tfl* :lookup-transform floor "world" (ros::time 0))))
     (unless spot-cds
       (ros::ros-error "Something wrong with TF. Did you launch jsk_maps?")
       (return-from :db-insert-spot nil))
     (send spot-cds :transform coords)
     (mongo::insert
      (ros::coords->tf-pose-stamped spot-cds floor)
      :meta `((:type . :spot)
              (:name . ,(send coords :name))
              (:latest . t)))))
  (:db-query-spot (spot-name)
   ;; just for assertion
   (unless (send self :split-floor-name spot-name)
     (return-from :db-query-spot nil))
   (let* ((doc (mongo::query geometry_msgs::PoseStamped
                             :msg-only t
                             :meta `((:type . :spot)
                                     (:name . ,spot-name)
                                     (:latest . t))
                             :sort '((:inserted_at . -1)))))
     (cond
       ((null doc)
        (ros::ros-info "No spot named '~A' registered in database" spot-name)
        (return-from :db-query-spot nil))
       ((> (length doc) 1)
        (ros::ros-warn "Detected ~A duplicated spot(s) named '~A'."
                       (length doc) spot-name)))
     (let* ((relcds (ros::tf-pose-stamped->coords (car doc)))
            (floor (send self :object (send relcds :name))))
       (send relcds :name spot-name)
       (dolist (oldcds (remove-if-not #'(lambda (s)
                                          (string= (send relcds :name)
                                                   (send s :name))) (send floor :descendants)))
         (send floor :dissoc oldcds))
       (send floor :assoc relcds)
       relcds)))
  (:db-query-latest-spots ()
   (let ((doc (mongo::query geometry_msgs::PoseStamped
                            :meta `((:type . :spot)
                                    (:latest . t)))))
     (mapcar #'(lambda (d)
                 (let* ((pose (car d))
                        (meta (cdr d))
                        (spot (ros::tf-pose-stamped->coords pose))
                        (name (cdr (assoc :name meta)))
                        (floor (car (send self :split-floor-name name))))
                   (send spot :name name)
                   (setq floor (send self :object floor))
                   (send floor :assoc spot)
                   spot)) doc)))
    ;; override functions
  (:add-spots (spots)
   (if db-enabled-p
       (every #'(lambda (s)
                  (send self :db-insert-spot s)) spots)
       (send-super :add-spots spots)))
  (:remove-spots (spots)
   (if db-enabled-p
       (mapcar #'(lambda (s)
                   (send self :db-remove-spot (send s :name))) spots)
       (send-super :remove-spots spots)))
  (:spot (name)
   (if db-enabled-p
       (let ((spot (send self :db-query-spot name)))
         (when spot (return-from :spot spot))))
   (send-super :spot name))
  (:spots (&optional name)
   (let (spots)
     (if db-enabled-p
         (setq spots (send self :db-query-latest-spots)))
     (append (send-super :spots) spots)))
)
