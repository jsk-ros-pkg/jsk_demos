#!/usr/bin/env roseus

(ros::roseus "rviz_menu_server")

(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_rviz_plugins")
(ros::roseus-add-msgs "jsk_recognition_msgs")
;(ros::roseus-add-srvs "jsk_recognition_msgs")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-msgs "drc_task_common")
(ros::roseus-add-srvs "drc_task_common")

(defun rviz-menu-init
  ()
  (setq *menu-index* 0)
  (setq *menu-length* 1)
  (setq *menu-title* "")
  (setq *menu-list* (list ""))
  (setq *menu-variable* 1)
  (setq *menu-variable-max* 0)
  (setq *menu-variable-min* 0)
  (setq *menu-bool* nil)
  (setq *menu-select-flag* nil)
  (ros::advertise "/rviz_menu_text" jsk_rviz_plugins::OverlayMenu 1)
  (ros::advertise-service "/rviz_menu_call" drc_task_common::RvizMenuCall #'rviz-menu-call)
  (ros::advertise-service "/rviz_menu_up" std_srvs::Empty #'rviz-menu-up)
  (ros::advertise-service "/rviz_menu_down" std_srvs::Empty #'rviz-menu-down)
  (ros::advertise-service "/rviz_menu_select" drc_task_common::RvizMenuSelect #'rviz-menu-select)
  (ros::advertise-service "/rviz_menu_cancel" std_srvs::Empty #'rviz-menu-cancel)
  (ros::subscribe "/rviz_menu_title" std_msgs::string #'set-rviz-menu-title)
  (ros::subscribe "/rviz_menu_list" drc_task_common::StringMultiArray #'set-rviz-menu-list)
  (ros::subscribe "/rviz_menu_variable" std_msgs::Float32 #'set-rviz-variable)
  (ros::subscribe "/rviz_menu_bool" std_msgs::Bool #'set-rviz-bool)

  (warning-message 2 "[rviz-menu-server] initialize rviz-menu-server.~%")
  )

(defun rviz-menu-call
  (req)
  (let* ((msg (instance jsk_rviz_plugins::OverlayMenu :init))
         (res (send req :response)) (menu-title-with-variable *menu-title*) (menu-list-with-variable *menu-list*))
    (setq *menu-index* 0)
    (setq *menu-title* (send req :title))
    (setq *menu-list* (send req :menu_list))
    (setq *menu-select-flag* nil)
    (setq *menu-length* (length *menu-list*))
    (setq *menu-variable-max* (send req :variable_max))
    (setq *menu-variable-min* (send req :variable_min))
    (setq *menu-variable* (send req :variable))
    (setq *menu-bool* (send req :boolean))
    (ros::rate 1000)
    (while (not *menu-select-flag*)
      (unless (and (equal *menu-variable-max* 0) (equal *menu-variable-min* 0))
        (setq menu-title-with-variable
              (format nil *menu-title* (+ (* *menu-variable* (- *menu-variable-max* *menu-variable-min*)) *menu-variable-min*))))
      (send msg :title menu-title-with-variable)
      (setq menu-list-with-variable nil)
      (dotimes (i *menu-length*)
        (cond ((not (substringp "~a" (elt *menu-list* i)))
               (push (elt *menu-list* i) menu-list-with-variable))
              ((= i *menu-index*)
               (push (format nil (elt *menu-list* i) (if *menu-bool* "  [SET]" "  [UNSET]")) menu-list-with-variable))
              (t
               (push (format nil (elt *menu-list* i) "  [---]") menu-list-with-variable))))
      (send msg :menus (reverse menu-list-with-variable))
      (send msg :current_index *menu-index*)
      (ros::publish "/rviz_menu_text" msg)
      (ros::sleep)
      (ros::spin-once))
    (send msg :action jsk_rviz_plugins::OverlayMenu::*ACTION_CLOSE*)
    (ros::publish "/rviz_menu_text" msg)
    (send res :index *menu-index*)
    (send res :variable_ret (+ (* *menu-variable* (- *menu-variable-max* *menu-variable-min*)) *menu-variable-min*))
    (send res :boolean_ret *menu-bool*)
    res))

(defun rviz-menu-up
  (req)
  (setq *menu-index* (+ *menu-index* 1))
  (when (= *menu-index* *menu-length*)
    (setq *menu-index* 0))
  (send req :response))

(defun rviz-menu-down
  (req)
  (setq *menu-index* (- *menu-index* 1))
  (when (= *menu-index* -1)
    (setq *menu-index* (- *menu-length* 1)))
  (send req :response))

(defun rviz-menu-select
  (req)
  (setq *menu-variable* (send req :variable))
  (setq *menu-select-flag* t)
  (send req :response))

(defun rviz-menu-cancel
  (req)
  (setq *menu-index* 0)
  (setq *menu-variable* 0.0)
  (setq *menu-select-flag* t)
  (send req :response))

(defun set-rviz-menu-title
  (msg)
  (setq *menu-title* (send msg :data)))

(defun set-rviz-menu-list
  (msg)
  (setq *menu-list* (send msg :data)))

(defun set-rviz-variable
  (msg)
  (setq *menu-variable* (send msg :data))
  )

(defun set-rviz-bool
  (msg)
  (setq *menu-bool* (not *menu-bool*))
  )


(rviz-menu-init)
(ros::rate 10)
(while t
  (ros::sleep)
  (ros::spin-once)
  )
