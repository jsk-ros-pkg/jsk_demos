(ros::roseus "miraikan")

(ros::load-ros-package "std_msgs")
;; (ros::roseus-add-msgs "miraikan_demo")
(ros::load-ros-package "miraikan_demo")
(setq *mode-srv* "/demo_mode")

(defun call_demo (mode time_delay)
  (let* ((req (instance miraikan_demo::ModeRequest :init
			:mode mode :time_delay time_delay))
	 res)
    (format t "waiting service ...~%")
    (ros::wait-for-service *mode-srv*)
    (setq res (ros::service-call *mode-srv* req))
    (format t "called service~%")
    )
  )

(defun episode_deai_1 (req)
  (let ((m (send req :response)))
    (call_demo 0 7)  ;; こちさんと出会ってからもう8年経ったね
    m)
  )

(defun episode_deai_2 (req)
  (let ((m (send req :response)))
    (call_demo 1 12)  ;; 皆に振り向いてもらえなくて悲しかったよ
    (ros::duration-sleep 2.0)
    (call_demo 2 7)  ;; こちさんが見つけてくれたね
    m)
  )

;; funcs -> rossrv msg
(ros::load-ros-package "std_srvs")
(defvar *process-rate* 5)

(ros::advertise-service "deai_1" std_srvs::Empty #'episode_deai_1)
(ros::advertise-service "deai_2" std_srvs::Empty #'episode_deai_2)

(defun main ()
  (ros::rate *process-rate*)
  (format t "robot behavior server start~%")
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)
    )
  (format t "robot behavior server end~%")
  )

;; (main)
;; (ros::exit)

