(defclass action-plan :super propertied-object :slots ())
(defmethod action-plan
  (:init ()
         (mapcar #'(lambda (x)
                     (send self :put x nil))
                 (list 
                  ;;body
                  :pose-name
                  :ri-method
                  :angle-vector
                  :tm
                  :stand-point
                  :lhand-target-coords
                  ;;hand
                  :hand-angle-vector
                  :hand-state)
                 ))
    (:copy-this ()
                (let (tmp-ap)
                  (setq tmp-ap (make-instance action-plan))
                  (mapcar #'(lambda (x)
                              (send tmp-ap :put (car x) (send self :get (car x))))
                          (send self :plist))
                  tmp-ap
                  ))
    )

(defclass action-plan-server :super propertied-object :slots(pose-name-list))
(defmethod action-plan-server
  (:init ()
         (setq plist nil)
         (setq pose-name-list nil)
         )
  (:push (ap)
         (cond ((send ap :get :pose-name)
                (setq pose-name (send ap :get :pose-name))
                (send self :put  pose-name (send ap :copy-this))
                (send ap :init)
                (format t "Pose name ~A was pushed to actoin-plan-server.~%" pose-name))
               (t (warn "pose-name is not defined!~%"))
                ))
  (:make-pose-name-list ()
                        (setq pose-name-list (mapcar #'(lambda (x) (car x)) plist)))
  (:get-action-plan-index (pose-name)
                (let (pose-index)
                  (send self :make-pose-name-list)
                  (setq tmp-length (length (memq pose-name pose-name-list)))
                  (- (length pose-name-list) tmp-length)
                  ))
  (:get-action-plan (pose-name)
                    (cdr (elt plist (send self :get-action-plan-index pose-name))))
  )
(defun action-plan-init ()
  (setq *ap* (make-instance action-plan))
  (send *ap* :init)
  )
(defun action-plan-server-init ()
  (setq *ap-server* (make-instance action-plan-server))
  (send *ap-server* :init)
  )
