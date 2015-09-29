#!/usr/bin/env roseus

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :mongo-client "package://roseus_mongo/euslisp/mongo-client.l")
(defparameter json::*tzoffset* -8) ;; use PST for PR2 Robot

(ros::roseus-add-msgs "people_msgs")
(ros::roseus-add-msgs "face_recognition")
(ros::roseus-add-msgs "jsk_hark_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(unless (boundp '*ri*)
  (pr2-init))

(setq *tfl* (instance ros::transform-listener :init))

(defun look-at-pos-random ()
  (let ((theta (deg2rad (- (random 30.0) 15))))
    (ros::ros-info "theta is ~A" theta)
    (let ((local-pos (float-vector (* 3000.0 (cos theta))
                                   (* 3000.0 (sin theta))
                                   0)))
      (let ((cds (send *tfl* :lookup-transform "/base_footprint" 
                       "/head_pan_link"
                       (ros::time 0))))
        (if cds (look-at-pos (send cds :transform-vector local-pos)))))))

(defun look-at-pos (world-pos)
  (ros::ros-info "looking at ~A" world-pos)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let ((current-pitch (send *pr2* :head :neck-p :joint-angle))
        (current-yaw (send *pr2* :head :neck-y :joint-angle)))
    (send *pr2* :head :neck-y :joint-angle 0)
    (send *pr2* :head :neck-p :joint-angle 0)
    (send *pr2* :head :look-at world-pos) ;LOOK AT!!
    (let ((next-pitch (send *pr2* :head :neck-p :joint-angle))
          (next-yaw (send *pr2* :head :neck-y :joint-angle)))
      (let ((diff (max (abs (- next-pitch current-pitch))
                       (abs (- next-yaw current-yaw)))))
        (let ((tm (/ diff 40.0)))
          (send *ri* :angle-vector (send *pr2* :angle-vector) (* 1000.0 tm) :head-controller)
          (setq *behavior-timeout* (* 2 tm))
          )))))

(defun position-measurement->look-at-pos (msg)
  (let ((cds (send *tfl* :lookup-transform "/base_footprint" 
                   (send msg :header :frame_id)
                   (send msg :header :stamp))))
        (if cds
            (progn
              (let ((world-pos (send cds :transform-vector (ros::tf-point->pos (send msg :pos)))))
                (let ((distance-from-robot (distance
                                            (float-vector (elt world-pos 0)
                                                          (elt world-pos 1)
                                                          0)
                                            (float-vector 0 0 0))))
                  (if (< distance-from-robot 2000.0)
                      world-pos
                    (progn
                      (ros::ros-warn "distance-from-robot > 2000: ~A" world-pos)
                      nil)))))
          ;; else
          (progn
            (ros::ros-warn "Failed to transform /base_footprint to ~A" (send msg :header :frame_id))
            nil))))

(defun people->look-at-pos (msg &optional idx)
  "return coords of people[idx] for look-at. if idx is not specified, use person that has the highest reliability"
  (let ((cds (send *tfl* :lookup-transform "/base_footprint"
                   (send msg :header :frame_id)
                   (send msg :header :stamp))))
    (if cds
        (progn
          (let* ((people (send msg :people))
                 (person (if idx (elt people idx)
                           (car (sort people #'(lambda (a b)
                                                 (> (send a :reliability) (send b :reliability)))))))
                 (world-pos (send cds :transform-vector (ros::tf-point->pos (send person :position))))
                 (distance-from-robot (distance
                                       (float-vector (elt world-pos 0)
                                                     (elt world-pos 1)
                                                     0)
                                       (float-vector 0 0 0))))
            (when (< (elt world-pos 2) 700.0)
              (setf (elt world-pos 2) 1300.0))
            (if (< distance-from-robot 2000.0)
                world-pos
              (progn
                (ros::ros-warn "distance-from-robot > 2000: ~A" world-pos)
                nil))))
    ;; else
    (progn
      (ros::ros-warn "Failed to transform /base_footprint to ~A" (send msg :header :frame_id))
      nil))))



(defun sound-direction->look-at-pos (msg)
  (let ((cds (send *tfl* :lookup-transform "/base_footprint" 
                   "/head_pan_link"
                       (send msg :header :stamp))))
    (if cds
        (let ((local-pos (scale 3000 (float-vector (send msg :vector :x)
                                                   (send msg :vector :y)
                                                   (send msg :vector :z)))))
          (let ((world-pos (send cds :transform-vector local-pos)))
            world-pos))
          (progn
            (ros::ros-warn "Failed to transform /base_footprint to ~A" (send msg :header :frame_id))
            nil))))

(defun relative-time-string-to-now (tm &key (suffix "会いましたね"))
  (let* ((ntm (ros::time-now))
         (ttn-sec (send (ros::time- ntm tm) :sec))
         ret-str)
    (setq ret-str
          (cond
           ((< ttn-sec (* 60 5)) ;; < 5 mins
            "さっき")
           ((< ttn-sec (* 60 60)) ;; < 1 hour
            (format nil "~A分前に" (/ ttn-sec 60)))
           ((< ttn-sec (* 60 60 24)) ;; < 1 day
            (format nil "~A時間前に" (/ ttn-sec 60 60)))
           ((< ttn-sec (* 60 60 24 2)) ;; < 2 days
            "昨日")
           ((< ttn-sec (* 60 60 24 7)) ;; < 7 days
            (format nil "~A日前に" (/ ttn-sec 60 60 24)))
           (t
            (return-from relative-time-string-to-now "久しぶりですね"))))
          (format nil "~A~A" ret-str suffix)))


(defun reply-to-person (name)
  (let ((res (mongo::query face_recognition::FaceRecognitionActionFeedback
                            :query `((:feedback.names . (,name)))
                            :sort '((:$natural . -1))
                            :single t))
        last-seen-date-ros)
    (when res
      (setq last-seen-date-ros (send (send (caar res) :header) :stamp))
      (speak-jp (format nil "~A さんこんにちは。~A" name (relative-time-string-to-now last-seen-date-ros))))))
