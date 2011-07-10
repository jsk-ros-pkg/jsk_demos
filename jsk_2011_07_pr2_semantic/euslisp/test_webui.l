#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_pr2_gui")
(load "package://pr2eus/pr2-interface.l")
(load "package://jsk_2011_07_pr2_semantic/euslisp/knowrob-interface.l")

(ros::roseus "demo_webui")

(defun init-settings ()
  (pr2)
  (setq *ri* (instance pr2-interface :init))
  (setq *time-count* (ros::time-now))
  (setq *jp* (instance json_prolog :init "json_prolog"))
  (ros::ros-info "init-settings end")
  )

(defun str-srv (req)
  (let ((m (send req :response)))
    (ros::ros-info "str-srv:~A" (send req :str))
    (print (send req :str))
    (send m :str (format nil "~A" (find-knowrob-objects-with-info (send req :str))))
    m))

(defun str-cb (msg)
  (cond ((> 0 (send (ros::time- (send msg :header :stamp) *time-count*) :to-sec))
	 (return-from str-cb nil))
	((not (equal (read-from-string (send msg :header :frame_id)) 'knowrob))
	 (return-from str-cb nil)))
  (ros::ros-info "Got ~A" (send msg :selection))
  (let* ((infront-of "'http://ias.cs.tum.edu/kb/knowrob.owl#inFrontOf-Generally'")
	 (obj (format nil "http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#~A" (send msg :selection))) solutions robot-pose)
    (print obj)
    (setq solutions
	  (send *jp* :query
		(list->prolog
		 (list (format nil "OBJ='~a'" obj)
		       `(owl_has :spot ,infront-of :obj))) :pose "SPOT"))
    (setq solutions (list (elt solutions 0)))
    (setq obj (cadr (assoc "OBJ" (car solutions) :test #'string=)))
    (setq robot-pose (cadr (assoc "POSE_SPOT" (car solutions) :test #'string=)))
    (ros::ros-info "GO to ~A" robot-pose)
    (send *ri* :move-to robot-pose)
  ))

;;
(init-settings)
(ros::advertise-service "/ipad/service/knowrob" roseus::StringString #'str-srv)
(ros::subscribe "/ipad/event/select" jsk_pr2_gui::iPadSelect #'str-cb)

(do-until-key
 (ros::spin-once))

;;(find-knowrob-objects "Cup")