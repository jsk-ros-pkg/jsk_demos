;; pick-cup.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :fetch-and-place-action "package://simple_fetch_and_place_pr2/euslisp/fetch-and-place-action.l")

(task-compiler/before-run-hook)

(move-to 'table)
(find-object 'cup)

#|
(ros::advertise "object_marker" visualization_msgs::Marker 1)
(unix:sleep 1)
(setq *cup* (car (send *scene* :find-object "room73b2-cup")))
(check-detection :type "room73b2-cup" :target-object *cup* :speak nil)
(setq m (object->marker-msg *cup* (instance std_msgs::header :init :stamp (ros::time-now) :frame_id "base_footprint")))
(ros::publish "object_marker" m)
|#

(pick 'cup 'rarm)
(move-to 'cook)
(place 'cup 'rarm)

(task-compiler/after-run-hook)
