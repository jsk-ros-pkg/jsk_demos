#!/usr/bin/env roseus

(load "package://decopin_hand/euslisp/decopin-interface.l")
(ros::load-ros-manifest "sensor_msgs")

(decopin-init)
(set-reset-pose)

(setq *set* nil)
(setq *initial-finger-rot* 0)
(setq *initial-finger1* 0)
(setq *initial-finger2* 0)
(setq *initial-finger3* 0)
(setq *av-initial* (send *robot* :angle-vector))
(setq *last-cb-time* (ros::time-now))

;; Usage
;; This robot hand has two mode: initial mode and working mode.
;; In initial mode, we decide initial position of robot hand. (*set* is nil)
;; In working mode, we move robot hand. (*set* is t)

;; We use several buttons to control this robot hand.
;; SET button: initial mode -> working mode
;; STOP button: working mode -> initial mode
;; START button: send control commands to real robot hand.
;; Sliders: selecting position of robot hand.

(defun joy-cb (msg)
  (let* ((axes (send msg :axes))
         (finger-rot (elt axes 8))
         (finger1 (elt axes 9))
         (finger2 (elt axes 10))
         (finger3 (elt axes 11))
         (buttons (send msg :buttons))
         (set (elt buttons 3))
         (stop (elt buttons 8))
         (start (elt buttons 9))
         (av (send *robot* :angle-vector)))
    (if (and (not (eq finger-rot 0.0))
             (not (eq finger1 0.0))
             (not (eq finger2 0.0))
             (not (eq finger3 0.0)))
      (progn
        ;; connect slider position and finger pose
        (when (eq set 1) ;; if SET button is pushed
          ;; if 3 left sliders are recognized
          (ros::warn "SET button is pushed. The initial pose of robot hand is set.~%")
          (setq *set* t)
          (setq *initial-finger-rot* finger-rot)
          (setq *initial-finger1* finger1)
          (setq *initial-finger2* finger2)
          (setq *initial-finger3* finger3)
          (set-reset-pose)
          (setq *av-initial* (send *robot* :angle-vector))
          (return-from joy-cb))
        (when (eq stop 1)
          (ros::warn "STOP button is pushed. You can move sliders freely.~%")
          (setq *set* nil)
          (return-from joy-cb))
        (when (eq start 1)
          (ros::warn "START button is pushed. The robot is going to move.~%")
          (send *ri* :angle-vector (send *robot* :angle-vector) 500)
          (unix:usleep (* 600 1000))
          (return-from joy-cb))
        ;; reflect joy topic to *robot*
        (if (eq *set* t)
          (progn
            (send *robot* :angle-vector
                  (v+ *av-initial*
                      (float-vector
                       (* 100 (- finger-rot *initial-finger-rot*))
                       (* 200 (- finger1 *initial-finger1*))
                       (* -200 (- finger2 *initial-finger2*))
                       (* 200 (- finger3 *initial-finger3*)))))
            (ros::warn (format nil "angle-vector: (~A ~A ~A ~A)~%~%"
                               (elt av 0) (elt av 1) (elt av 2) (elt av 3))))
          (ros::ros-error "Push SET button before moving hand~%")))
      (ros::ros-error "Please move 4 left sliders and then push SET button~%"))
    ))


(defun main ()
  (ros::subscribe "/nanokontrol/joy" sensor_msgs::Joy #'joy-cb)
  (ros::rate 100)
  (ros::warn "~%Please use nanoKONTROL2~%")
  (do-until-key
   (ros::spin-once)
   (ros::sleep))
  (exit))


(main)
