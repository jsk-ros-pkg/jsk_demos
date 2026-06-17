#!/usr/bin/env roseus

(ros::roseus "debug-detic-handles")
(ros::load-ros-manifest "jsk_recognition_msgs")

(setq *boxes* nil)
(setq *bbox-center-1* nil)
(setq *bbox-center-2* nil)
(setq *last-label-msg* nil)
(setq *last-label-secs* nil)
(setq *last-label-nsecs* nil)
(setq *label-msg-buffer* nil)
(setq *label-buffer-size* 30)
(setq *label-match-threshold-nsecs* 100000000) ;; 100 ms

(defun stamp->nsecs (stamp)
  (let* ((secs (when stamp (send stamp :sec)))
         (nsecs (when stamp (send stamp :nsec))))
    (when (and secs nsecs)
      (+ (* secs 1000000000) nsecs))))

(defun trim-label-buffer ()
  (let ((trimmed nil)
        (i 0))
    (dolist (entry *label-msg-buffer*)
      (when (< i *label-buffer-size*)
        (setq trimmed (cons entry trimmed)))
      (setq i (+ i 1)))
    (setq *label-msg-buffer* (reverse trimmed))))

(defun nearest-label-entry (stamp-nsecs)
  (let ((best-entry nil)
        (best-diff nil))
    (dolist (entry *label-msg-buffer*)
      (let ((diff (abs (- stamp-nsecs (car entry)))))
        (when (or (null best-diff) (< diff best-diff))
          (setq best-entry entry)
          (setq best-diff diff))))
    (when (and best-entry (<= best-diff *label-match-threshold-nsecs*))
      (list best-diff best-entry))))

(defun class-cb (msg)
  (setq *class-list* msg)
  (let* ((hdr (send msg :header))
         (stamp (when hdr (send hdr :stamp)))
         (secs (when stamp (send stamp :sec)))
         (nsecs (when stamp (send stamp :nsec)))
         (stamp-nsecs (stamp->nsecs stamp)))
    (setq *last-label-msg* msg)
    (setq *last-label-secs* secs)
    (setq *last-label-nsecs* nsecs)
    (when stamp-nsecs
      (setq *label-msg-buffer*
            (cons (list stamp-nsecs msg secs nsecs) *label-msg-buffer*))
      (trim-label-buffer))
    (format t "received LabelArray stamp: ~A.~A~%" secs nsecs))
)


(defun boxes-cb (msg)
  (setq *boxes* msg)

  (let* ((hdrb (send msg :header))
         (bstamp (when hdrb (send hdrb :stamp)))
         (bsecs (when bstamp (send bstamp :sec)))
         (bnsecs (when bstamp (send bstamp :nsec)))
         (box-stamp-nsecs (stamp->nsecs bstamp))
         (matched-labels nil))
    (when box-stamp-nsecs
      (let ((match (nearest-label-entry box-stamp-nsecs)))
        (when match
          (let* ((diff-nsecs (car match))
                 (entry (cadr match))
                 (label-msg (cadr entry))
                 (label-secs (caddr entry))
                 (label-nsecs (cadddr entry)))
            (setq matched-labels (send label-msg :labels))
            (format t "Matched LabelArray within ~A ns: box=~A.~A label=~A.~A diff=~A ns~%"
                    *label-match-threshold-nsecs*
                    bsecs bnsecs
                    label-secs label-nsecs
                    diff-nsecs)))))
  (let* ((boxes (send msg :boxes))
         (n (length boxes)))
    (format t "received boxes: ~A~%" n)

    (if (>= n 2)
        (let* ((box0 (elt boxes 0))
               (box1 (elt boxes 1))
               (p0 (send box0 :pose :position))
               (p1 (send box1 :pose :position)))
          (setq *bbox-center-1*
                (float-vector (send p0 :x) (send p0 :y) (send p0 :z)))
          (setq *bbox-center-2*
                (float-vector (send p1 :x) (send p1 :y) (send p1 :z)))

          (format t "bbox center 1 [m]: x=~A y=~A z=~A~%"
                  (send p0 :x) (send p0 :y) (send p0 :z))
          (format t "bbox center 2 [m]: x=~A y=~A z=~A~%"
                  (send p1 :x) (send p1 :y) (send p1 :z)))
      (progn
        (setq *bbox-center-1* nil)
        (setq *bbox-center-2* nil)
        (format t "need at least 2 bboxes, current: ~A~%" n)))

    (dotimes (i n)
      (let* ((box (elt boxes i))
             (p (send box :pose :position))
             (d (send box :dimensions))
             (label (send box :label))
             (value (send box :value)))

        (let ((class-name nil))
          (when matched-labels
            (when (< i (length matched-labels))
              (let ((lab (elt matched-labels i)))
                (setq class-name (send lab :name)))))

        (format t "box[~A] label=~A value=~A class=~A~%" i label value class-name)
        (format t "  center [m]: x=~A y=~A z=~A~%"
                (send p :x)
                (send p :y)
                (send p :z))
        (format t "  size   [m]: x=~A y=~A z=~A~%"
                (send d :x)
                (send d :y)
                (send d :z))))))))
(ros::subscribe "/docker/detic_segmentor/output/boxes"
                jsk_recognition_msgs::BoundingBoxArray
                #'boxes-cb)

(ros::subscribe "/docker/detic_segmentor/detected_classes"
                jsk_recognition_msgs::LabelArray
                #'class-cb)

(format t "waiting for Detic BoundingBoxArray...~%")

(while (ros::ok)
  (ros::spin-once)
  (unix::usleep 100000))
