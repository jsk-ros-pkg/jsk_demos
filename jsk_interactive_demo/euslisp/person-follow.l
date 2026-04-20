#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus "person recognition")

(defclass person-recog-cb-class
  :super propertied-object
  :slots (label_name rect person_x person_y width_person person_height person_area pose))
(defmethod person-recog-cb-class
  (:init ()
   (load "package://fetcheus/fetch-interface.l")
   (fetch-init)
   (ros::subscribe "/edgetpu_panorama_object_detector/output/class" jsk_recognition_msgs::ClassificationResult
                   #'send self :person-recog-class-cb 1)
   (ros::subscribe "/edgetpu_panorama_object_detector/output/rects"
                   jsk_recognition_msgs::RectArray #'send self :person-rect-cb 1)
   (ros::subscribe "/edgetpu_panorama_object_detector/output/detected"
                  jsk_recognition_msgs::PeoplePoseArray #'send self :person-detected-cb 1)


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
  (:person-detected-cb (msg)
                       (setq pose (send msg :poses))
                       
                       )
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
                             (cond ((< (+ person_x (/ person_width 2)) 400)
                                    (progn
                                      (ros::ros-info "left")
                                      (send *ri* :go-pos 0 0 20)))
                                   ((> (+ person_x (/ person_width 2)) 600) ;; width 1000 height 500
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
   (:person-detected-loop ()
      (ros::rate 1)
      (while (ros::ok)
        (when )
       )
   (:person-front () ;; for debug
                 (ros::rate 1)
                 (while (ros::ok)
                   (when (and rect label_name)
                     (when (string= (elt label_name 0) "person")
                       (cond ((< person_area 80000) (send *ri* :go-pos 0.5 0 0)))))
                   (ros::spin-once)
                   (ros::sleep))
                 (exit)))

(setq r (instance person-recog-cb-class :init))
;; (send r :person-follow-loop)
(defun main ()
  (setq person-follow (instance person-recog-cb-class :init))
  (send person-follow :person-follow-loop))
