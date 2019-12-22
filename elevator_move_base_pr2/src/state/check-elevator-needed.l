(load "package://elevator_move_base_pr2/src/utils.l")


(defvar *floors*)
(defvar *scene*)
(defvar *tfl*)


(defun check-elevator-needed (userdata)
  (ros::ros-info "Checking if elevator is needed...")
  (update-robot-position)
  (let* ((goal (cdr (assoc :goal userdata)))
         (goal-pose (ros::tf-pose-stamped->coords
                     (send goal :target_pose)))
         (cur-floor (check-current-floor *scene* *tfl*))
         (target-floor (check-current-floor *scene* *tfl* goal-pose))
         target-floor-button
         (target-coords
          (transform-pose-to-target-frame goal-pose (send *scene* :name) *tfl*))
         (up/down (cond ((send *scene* :floor< target-floor cur-floor) "down")
                        ((send *scene* :floor< cur-floor target-floor) "up")
                        (t nil))))
    (ros::ros-info
     (format nil
             "cur-floor: ~A, target-floor: ~A, up/down: ~A, target-coords: ~A"
             cur-floor target-floor up/down target-coords))
    (setq target-floor-button
          (elt (assoc target-floor *floors* :test #'string=
                      :key #'(lambda (x)
                               (format nil "~A/~A" (send *scene* :name)
                                       (string-downcase (elt x 0)))))
               0)) ;; /eng8/1f -> 1F
    (set-alist 'inside-panel-name "/elevator_inside_panel" userdata)
    (set-alist 'outside-panel-name "/elevator_call_panel" userdata)
    ;; transform to scene frame
    (set-alist
     'elevator-front-coords
     (let ((coords (send *scene* :transformation
                         (car (send *scene* :find-object
                                    (format nil "~A/elevator-outside"
                                            cur-floor))))))
       (send coords :name (send *scene* :name))
       (send coords :rotate pi :z)
       coords)
     userdata)
    (set-alist
     'front-coords
     (let ((coords (send *scene* :transformation
                         (car (send *scene* :find-object
                                    (format nil "~A/elevator_call_panel-front"
                                            cur-floor))))))
       (send coords :name (send *scene* :name))
       coords)
     userdata)
    (set-alist
     'inside-coords
     (let ((coords (send *scene* :transformation
                         (car (send *scene* :find-object
                                    (format nil "~A/elevator_inside_panel-front"
                                            cur-floor))))))
       (send coords :name (send *scene* :name))
       coords)
     userdata)
    (set-alist
     'outside-coords
     (let ((coords (send *scene* :transformation
                         (car (send *scene* :find-object
                                    (format nil "~A/elevator-outside"
                                            target-floor))))))
       (send coords :name (send *scene* :name))
       coords)
     userdata)
    (set-alist 'target-floor target-floor-button userdata)
    (set-alist 'outside-button up/down userdata)
    (set-alist 'target-coords target-coords userdata)
    (not (null up/down))))
