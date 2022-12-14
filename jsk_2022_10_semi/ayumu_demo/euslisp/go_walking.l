#! usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus "go_walking")

(defun sub_to_walking (msg)
  (when (= *umbrella_replace_done* nil)
    (return-from sub_to_walking))
    
  (setq data (send msg :data))

  (setq x (elt data 1))
  (setq y (elt data 2))

  (when (> (abs (- x *pre_x*)) 0.5)
    ;; 左右に動く
    (send *ri* :go-pos 0 (- x *pre_x*) 0)
    (setq *pre_x* x))

  (when (> (abs (- y *pre_y*)) 0.5)
    ;; 前後に動く
    (send *ri* :go-pos (- y *pre_y*) 0 0)
    (setq *pre_y* y))
  )

(ros::subscribe "/nearest_pos" std_msgs::float32multiarray #'sub_to_walking)
