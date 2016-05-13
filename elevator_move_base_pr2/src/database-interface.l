(require :mongo-client "package://roseus_mongo/euslisp/mongo-client.l")

(when (and (ros::has-param "robot/name")
           (string= "pr1012" (ros::get-param "robot/name")))
  (setq json::*tzoffset* -8)
  (ros::ros-warn "time zone offset is changed to -8 (PST)"))

;;;;;; insert

(defclass mongo-logger
  :slots (task-id)
  :super propertied-object)
(defmethod mongo-logger
  (:init (&optional (name ""))
   (setq task-id (format nil "~A~A" name (gensym)))
   (ros::ros-warn "logger is initialized task-id: ~A" task-id)
   t)
  (:save-robot-pose (&optional comment)
   (when *logging*
     (let ((c (send *ri* :state :worldcoords)) docid)
       (ros::ros-warn "saving robot pose...")
       (setq docid
             (mongo::insert
              (ros::coords->tf-pose-stamped c (send c :name))
              :meta `((:comment . ,comment)
                      (:task-id . ,task-id))))
       (ros::ros-warn "saved robot pose ~A" docid)))
   t)
  (:save-obj-pose (obj &optional comment)
   (when *logging*
     (let ((robot->obj (send *pr2* :transformation obj))
           docid)
       (ros::ros-warn "saving obj pose ~A..." (send obj :name))
       (setq docid
             (mongo::insert
              (ros::coords->tf-pose-stamped robot->obj "/base_footprint")
              :meta `((:object . ,(send obj :name))
                      (:task-id . ,task-id)
                      (:comment . ,comment))))
       (ros::ros-warn "saved obj pose ~A: ~A" (send obj :name) docid)))
   t)
  (:save-brightness (value &optional comment)
   (when *logging*
     (ros::ros-warn "saving brightness ~A" value)
     (setq docid
           (mongo::insert
            (instance std_msgs::float32 :init :data value)
            :meta `((:comment . ,comment)
                    (:task-id . ,task-id)))))
   t)
  )

(provide :database-interface)

