;; dummy-action-for-pddl.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(defun publish-string (name)
  (unless (ros::get-topic-publisher name)
    (ros::advertise name std_msgs::String 1)
    (unix:sleep 1))
  (ros::publish name (instance std_msgs::String :init :data name)))

(defmacro deftestaction (name)
  `(defun ,name (&rest args)
     (ros::ros-info (format nil "(~A ~A)" (string ',name) args))
     (publish-string (string-downcase
                      (substitute #\_ #\- (string ',name))))
     :success))

(deftestaction move-to)
(deftestaction pick)
(deftestaction place)
(deftestaction put)
(deftestaction open-door)
(deftestaction close-door)
(deftestaction push-button)
(deftestaction sweep)
(deftestaction sweep-under)
(deftestaction pull-chair)
(deftestaction push-chair)

(provide :dummy-action-for-pddl) ;; end of dummy-action-for-pddl.l
