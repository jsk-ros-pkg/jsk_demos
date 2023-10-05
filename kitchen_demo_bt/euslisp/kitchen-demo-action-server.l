(ros::roseus "action_server")
(ros::load-ros-package "kitchen_demo_bt")
(load "package://roseus_bt/euslisp/nodes.l")
(require :fetch-interface "package://fetcheus/fetch-interface.l")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(defvar *tweet* t)
(defvar *n-dock-trial* 3)
(defvar *n-move-to-trial* 3)
(defvar *control-switchbot* :api)
(setq *initial-light-on* nil)
(setq *success-auto-dock* nil)
(setq *success-go-to-kitchen* nil)

(ros::advertise "/tweet" std_msgs::String 1)
(ros::advertise "/photo_taken" sensor_msgs::Image 1)
(setq *image* nil)


(defun image-cb (msg) (setq *image* msg))


;; define callbacks
(defun auto-dock-execute-cb (server goal)
  (report-auto-dock)
  (let ((success (auto-dock :n-trial *n-dock-trial* :clear-costmap nil)))
    (setq *success-auto-dock* success)
    success))

(defun finish-execute-cb (server goal)
  (let ((success-battery-charging
         (progn (wait-until-is-charging)
                (equal (get-battery-charging-state) :charging))))
    (restore-params)
    (and *success-auto-dock* *success-go-to-kitchen* success-battery-charging)))

(defun get-light-on-execute-cb (server goal)
  (let ((light-on (get-light-on)))
    (setq *initial-light-on* light-on)
    light-on))

(defun init-execute-cb (server goal)
  (fetch-init)
  (if (send *ri* :simulation-modep)
    (progn
      (load "models/room73b2-scene.l")
      (load "package://jsk_maps/src/eng2-scene.l")
      (room73b2)
      (setq *base-spot* "/eng2/7f/room73B2-base")
      (setq *scene* (make-eng2-7f-scene))
      (send *ri* :objects (send *room73b2* :objects))
      (send (send *ri* :robot)
            :newcoords (car (get-spot-coords "/eng2/7f/room73B2-center")))
      ))
  (undock)
  (send *ri* :clear-costmap)
  (store-params)
  (inflation-loose)
  (clear-app-notification)
  (ros::subscribe "/edgetpu_object_detector/output/image/compressed"
                  sensor_msgs::CompressedImage #'image-cb)
  t)

(defun inspect-spot-execute-cb (server goal)
  (let* ((spot-name (send goal :goal :spot_name))
         (inspect-func
           (read-from-string
             (format nil "inspect-~A" (substitute #\- #\_ spot-name)))))
    (eval `(,inspect-func :tweet ,*tweet*))
    t))

(defun move-to-spot-execute-cb (server goal)
  (let* ((spot-name (send goal :goal :spot_name))
         (move-to-func
           (read-from-string
             (format nil "move-to-~A" (substitute #\- #\_ spot-name)))))
    (eval `(,move-to-func :n-trial ,*n-move-to-trial*))))

(defun report-auto-dock-failure-execute-cb (server goal)
  (report-auto-dock-failure)
  t)

(defun report-failure-execute-cb (server goal)
  (let* ((spot-name (send goal :goal :spot_name))
         (report-failure-func
           (read-from-string
             (format nil "report-move-to-~A-failure" (substitute #\- #\_ spot-name)))))
    (eval `(,report-failure-func))
    t))

(defun report-light-on-execute-cb (server goal)
  (report-light-on)
  t)

(defun report-start-go-to-kitchen-execute-cb (server goal)
  (report-start-go-to-kitchen)
  t)

(defun report-success-execute-cb (server goal)
  (let* ((spot-name (send goal :goal :spot_name))
         (report-success-func
           (read-from-string
             (format nil "report-move-to-~A-success" (substitute #\- #\_ spot-name)))))
    (eval `(,report-success-func))
    t))

(defun room-light-off-execute-cb (server goal)
  (if (and *success-auto-dock* (not *initial-light-on*))
    (room-light-off :control-switchbot *control-switchbot*))
  t)

(defun room-light-on-execute-cb (server goal)
  (let ((label-names (notify-recognition :location "on the way to kitchen")))
     (report-light-off)
     (room-light-on :control-switchbot *control-switchbot*))
  t)


;; create server instances
(instance roseus_bt:action-node :init
          "kitchen_demo/auto_dock" kitchen_demo_bt::AutoDockAction
          :execute-cb 'auto-dock-execute-cb)

(instance roseus_bt:action-node :init
          "kitchen_demo/finish" kitchen_demo_bt::FinishAction
          :execute-cb 'finish-execute-cb)

(instance roseus_bt:action-node :init
          "kitchen_demo/get_light_on" kitchen_demo_bt::GetLightOnAction
          :execute-cb 'get-light-on-execute-cb)

(instance roseus_bt:action-node :init
          "kitchen_demo/init" kitchen_demo_bt::InitAction
          :execute-cb 'init-execute-cb)

(instance roseus_bt:action-node :init
          "kitchen_demo/inspect_spot" kitchen_demo_bt::InspectSpotAction
          :execute-cb 'inspect-spot-execute-cb)

(instance roseus_bt:action-node :init
          "kitchen_demo/move_to_spot" kitchen_demo_bt::MoveToSpotAction
          :execute-cb 'move-to-spot-execute-cb)

(instance roseus_bt:action-node :init
          "kitchen_demo/report_auto_dock_failure" kitchen_demo_bt::ReportAutoDockFailureAction
          :execute-cb 'report-auto-dock-failure-execute-cb)

(instance roseus_bt:action-node :init
          "kitchen_demo/report_failure" kitchen_demo_bt::ReportFailureAction
          :execute-cb 'report-failure-execute-cb)

(instance roseus_bt:action-node :init
          "kitchen_demo/report_light_on" kitchen_demo_bt::ReportLightOnAction
          :execute-cb 'report-light-on-execute-cb)

(instance roseus_bt:action-node :init
          "kitchen_demo/report_start_go_to_kitchen" kitchen_demo_bt::ReportStartGoToKitchenAction
          :execute-cb 'report-start-go-to-kitchen-execute-cb)

(instance roseus_bt:action-node :init
          "kitchen_demo/report_success" kitchen_demo_bt::ReportSuccessAction
          :execute-cb 'report-success-execute-cb)

(instance roseus_bt:action-node :init
          "kitchen_demo/room_light_off" kitchen_demo_bt::RoomLightOffAction
          :execute-cb 'room-light-off-execute-cb)

(instance roseus_bt:action-node :init
          "kitchen_demo/room_light_on" kitchen_demo_bt::RoomLightOnAction
          :execute-cb 'room-light-on-execute-cb)


;; set rate
(ros::rate 100)

;; spin
(roseus_bt:spin)
