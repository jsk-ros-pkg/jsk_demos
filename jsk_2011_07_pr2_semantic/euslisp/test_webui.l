#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(load "package://jsk_maps/src/eng2-scene.l")
(load "package://pr2eus/pr2-interface.l")
(load "eus_json_prolog.l") ;; -> json_prolog/src

(ros::roseus "demo_webui")

(defun init-settings ()
;;  (setq *pr2* (pr2))
;;  (setq *ri* (instance pr2-interface :init))
;;  (setq *knowrob* (instance json_prolog :init "json_prolog"))
  (ros::ros-info "init-settings end")
  )

(defun str-srv (req)
  (let ((m (send req :response)))
    (ros::ros-info "str-srv:~A" (send req :str))
    (send m :str "hoge1 hoge2")
    m))

;;
(init-settings)
(ros::advertise-service "web_str_srv" roseus::StringString #'str-srv)

(do-until-key
 (ros::spin-once))

