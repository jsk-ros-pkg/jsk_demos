#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(load "package://jsk_maps/src/eng2-scene.l")
(load "package://pr2eus/pr2-interface.l")
(load "eus_json_prolog.l") ;; -> json_prolog/src

(ros::roseus "demo_webui")

(defun init-settings ()
;;  (setq *pr2* (pr2))
;;  (setq *ri* (instance pr2-interface :init))
  (setq *jp* (instance json_prolog :init "json_prolog"))
  (ros::ros-info "init-settings end")
  )

(defun str-srv (req)
  (let ((m (send req :response)))
    (ros::ros-info "str-srv:~A" (send req :str))
    (print (send req :str))
    (send m :str (format nil "~A" (find-knowrob-objects (send req :str))))
;;    (send m :str "hoge1 hoge2")
    m))

(defun str-cb (msg)
  (ros::ros-info "Got ~A" (send msg :data))
  )

;;
(init-settings)
(ros::advertise-service "web_str_srv" roseus::StringString #'str-srv)
(ros::subscribe "/ipad/event/select" roseus::StringStamped #'str-cv)

(do-until-key
 (ros::spin-once))

;;(find-knowrob-objects "Cup")