(load "attention-observation.l")
(load "sound-attention-node.l")

(pr2-init)

(unless (ros::ok)
  (ros::roseus "tablet_attention"))

(setq *type* nil *from* nil)
(ros::subscribe "/Tablet/StartDemo" roseus::StringStamped
                #'(lambda(m)
                    (print (list :subscribe (send m :data)))
                    (cond
                     ((string= (send m :data)
                               "/Tablet/other/GetGeorgia")
                      (setq *from* :tablet)
                      (setq *type* "georgia"))
                     ((string= (send m :data)
                               "/Tablet/other/DoNothing")
                      (setq *from* :tablet)
                      (setq *type* "nothing"))
                     (t ))))

(defun wait-android-query ()
  (ros::ros-info "Begin Waiting")
  (let (type from (cntr 0))
    (ros::rate 100)
    (do-until-key
     (incf cntr)
     (when (> cntr 1000)
       (ros::ros-info " Waiting ....")
       (setq cntr 0))

     (ros::spin-once)
     (send *ri* :state)
     (let ((mtrq
            (apply
             #'max (mapcar
                    #'(lambda (j) (/ (abs (send j :joint-torque))
                                     (send j :max-joint-torque)))
                    (flatten (send *ri* :robot :arms :joint-list))))))
       ;;(pprint mtrq)
       (when (> mtrq 0.8)
         (ros::ros-info "TORQUE INTERRUPT ~A" mtrq)
         (send *ri* :cancel-all-motion t)
         ))
     (when (sound-attention)
       (ros::ros-info "SOUND INTERRUPT")
       ;;(send *ri* :cancel-all-motion "sound")
       (send *ri* :cancel-all-motion t)
       )
     (when *type*
       (case *from*
         (:tablet
          (cond
           ((string= *type* "nothing")
            (ros::ros-info "TABLET INTERRUPT")
            ;;(send *ri* :cancel-all-motion "tablet")
            (send *ri* :cancel-all-motion t)
            )
           (t )
           )
          ))
       (setq *type* nil *from* nil)
       )
     (ros::sleep)
     );; /do-until-key
    ))

(wait-android-query)
