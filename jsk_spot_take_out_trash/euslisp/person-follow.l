#!/usr/bin/env roseusb

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus "person recognition")

(defclass person-recog-cb-class
  :super propertied-object
  :slots (label_name rect person_x person_y width_person person_height person_area))
(defmethod person-recog-cb-class
  (:init ()
   (ros::subscribe "/spot_recognition/class" jsk_recognition_msgs::ClassificationResult
                   #'send self :person-recog-class-cb 1)
   (ros::subscribe "/spot_recognition/rects"
                   jsk_recognition_msgs::RectArray #'send self :person-rect-cb 1))

  (:person-recog-class-cb (msg)
                         (setq label_name (send msg :label_names))
                         ;; (ros::ros-info "~A" (list 'person-cb self label_name))
                         )
  (:person-rect-cb (msg)
                   (when label_name
                     ;; (when (string= (elt label_name 0) "person")
                     (setq rect (send msg :rects))
                     (when rect
                       (setq person_x (send (elt rect 0) :x))
                       (setq person_y (send (elt rect 0) :y))
                       (setq person_width (send (elt rect 0) :width))
                       (setq person_height (send (elt rect 0) :height))
                       (setq person_area (* person_width person_height)))))
  (:print-loop () ;; for debug
         (ros::rate 1)
         (while (ros::ok)
           (when (and rect label_name)
             (ros::ros-info
              "label: ~A(x,y)=(~A,~A), width: ~A, height: ~A, area: ~A"
              (elt label_name 0) person_x person_y person_width person_height
              (* person_width person_height)))
           (ros::spin-once)
           (ros::sleep))
         (exit))
  (:person-follow-loop () ;; person follow
                       (ros::rate 1)
                       (while (ros::ok)
                         (when (and rect label_name)
                           (when (string= (elt label_name 0) "person")
                             (ros::ros-info (format nil "x:~A" (+ person_x (/ person_width 2))))
                             (cond ((< (+ person_x (/ person_width 2)) 800)
                                    (progn
                                      (ros::ros-info "left")
                                      (send *ri* :go-pos 0 0 20)))
                                   ((> (+ person_x (/ person_width 2)) 1000)
                                    (progn
                                      (ros::ros-info "right")
                                      (send *ri* :go-pos 0 0 -20)))
                                   (t
                                    (progn
                                      (ros::ros-info "center")
                                      (when (< person_area 70000)
                                        (send *ri* :go-velocity 0.5 0 0)))))))
                         (ros::spin-once)
                         (ros::sleep))
                       (exit))
  (:person-front () ;; for debug
                 (ros::rate 1)
                 (while (ros::ok)
                   (when (and rect label_name)
                     (when (string= (elt label_name 0) "person")
                       (cond ((< person_area 80000) (send *ri* :go-velocity 0.5 0 0)))))
                   (ros::spin-once)
                   (ros::sleep))
                 (exit)))

(setq r (instance person-recog-cb-class :init))
;; (send r :person-follow-loop)
