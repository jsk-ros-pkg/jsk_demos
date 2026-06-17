;; Pot/handle perception utilities.
;; This file subscribes to Detic results and stores matched detections.

(ros::load-ros-manifest "jsk_recognition_msgs")

(setq *pot-handle-box-topic* "/docker/detic_segmentor/output/boxes")
(setq *pot-handle-label-topic* "/docker/detic_segmentor/detected_classes")

(setq *pot-handle-label-match-threshold-nsecs* 100000000) ;; 100 ms
(setq *pot-handle-label-buffer-size* 30)
(setq *pot-handle-label-buffer* nil)

(setq *pot-handle-latest-box-msg* nil)
(setq *pot-handle-latest-label-msg* nil)
(setq *pot-handle-latest-detections* nil)
(setq *pot-handle-latest-pots* nil)
(setq *pot-handle-latest-handles* nil)
(setq *pot-handle-perception-started* nil)

(defun pot-handle-stamp->nsecs (stamp)
  (let* ((secs (when stamp (send stamp :sec)))
         (nsecs (when stamp (send stamp :nsec))))
    (when (and secs nsecs)
      (+ (* secs 1000000000) nsecs))))

(defun pot-handle-trim-label-buffer ()
  (let ((trimmed nil)
        (i 0))
    (dolist (entry *pot-handle-label-buffer*)
      (when (< i *pot-handle-label-buffer-size*)
        (setq trimmed (cons entry trimmed)))
      (setq i (+ i 1)))
    (setq *pot-handle-label-buffer* (reverse trimmed))))

(defun pot-handle-nearest-label-entry (stamp-nsecs)
  (let ((best-entry nil)
        (best-diff nil))
    (dolist (entry *pot-handle-label-buffer*)
      (let ((diff (abs (- stamp-nsecs (car entry)))))
        (when (or (null best-diff) (< diff best-diff))
          (setq best-entry entry)
          (setq best-diff diff))))
    (when (and best-entry
               (<= best-diff *pot-handle-label-match-threshold-nsecs*))
      (list best-diff best-entry))))

(defun pot-handle-box-center (box)
  (let ((p (send box :pose :position)))
    (float-vector (send p :x) (send p :y) (send p :z))))

(defun pot-handle-box-dimensions (box)
  (let ((d (send box :dimensions)))
    (float-vector (send d :x) (send d :y) (send d :z))))

(defun pot-handle-box-volume (box)
  (let ((d (send box :dimensions)))
    (* (send d :x) (send d :y) (send d :z))))

(defun make-pot-handle-detection (class-name box label-msg label-index)
  (list class-name
        (pot-handle-box-center box)
        (pot-handle-box-dimensions box)
        (pot-handle-box-volume box)
        box
        label-msg
        label-index))

(defun pot-handle-detection-class (detection)
  (car detection))

(defun pot-handle-detection-center (detection)
  (cadr detection))

(defun pot-handle-detection-dimensions (detection)
  (caddr detection))

(defun pot-handle-detection-volume (detection)
  (cadddr detection))

(defun pot-handle-detection-box (detection)
  (car (cddddr detection)))

(defun pot-handle-filter-detections-by-class (detections class-name)
  (let ((filtered nil))
    (dolist (detection detections)
      (when (string= (pot-handle-detection-class detection) class-name)
        (setq filtered (cons detection filtered))))
    (reverse filtered)))

(defun pot-handle-label-cb (msg)
  (let* ((hdr (send msg :header))
         (stamp (when hdr (send hdr :stamp)))
         (stamp-nsecs (pot-handle-stamp->nsecs stamp)))
    (setq *pot-handle-latest-label-msg* msg)
    (when stamp-nsecs
      (setq *pot-handle-label-buffer*
            (cons (list stamp-nsecs msg) *pot-handle-label-buffer*))
      (pot-handle-trim-label-buffer))))

(defun pot-handle-box-cb (msg)
  (let* ((hdr (send msg :header))
         (stamp (when hdr (send hdr :stamp)))
         (stamp-nsecs (pot-handle-stamp->nsecs stamp))
         (match (when stamp-nsecs
                  (pot-handle-nearest-label-entry stamp-nsecs)))
         (matched-label-msg nil)
         (matched-labels nil)
         (detections nil))
    (setq *pot-handle-latest-box-msg* msg)
    (when match
      (setq matched-label-msg (cadr (cadr match)))
      (setq matched-labels (send matched-label-msg :labels)))
    (let* ((boxes (send msg :boxes))
           (n (length boxes)))
      (dotimes (i n)
        (let ((class-name nil))
          (when (and matched-labels (< i (length matched-labels)))
            (setq class-name (send (elt matched-labels i) :name)))
          (when class-name
            (setq detections
                  (cons (make-pot-handle-detection
                         class-name
                         (elt boxes i)
                         matched-label-msg
                         i)
                        detections))))))
    (setq *pot-handle-latest-detections* (reverse detections))
    (setq *pot-handle-latest-pots*
          (pot-handle-filter-detections-by-class
           *pot-handle-latest-detections* "pot"))
    (setq *pot-handle-latest-handles*
          (pot-handle-filter-detections-by-class
           *pot-handle-latest-detections* "handle"))))

(defun start-pot-handle-perception ()
  (unless *pot-handle-perception-started*
    (ros::subscribe *pot-handle-box-topic*
                    jsk_recognition_msgs::BoundingBoxArray
                    #'pot-handle-box-cb)
    (ros::subscribe *pot-handle-label-topic*
                    jsk_recognition_msgs::LabelArray
                    #'pot-handle-label-cb)
    (setq *pot-handle-perception-started* t)))

(defun update-pot-handle-perception ()
  (ros::spin-once))

(defun get-latest-pot-handle-detections ()
  *pot-handle-latest-detections*)

(defun get-latest-pots ()
  *pot-handle-latest-pots*)

(defun get-latest-handles ()
  *pot-handle-latest-handles*)

(defun clear-pot-handle-perception ()
  (setq *pot-handle-label-buffer* nil)
  (setq *pot-handle-latest-box-msg* nil)
  (setq *pot-handle-latest-label-msg* nil)
  (setq *pot-handle-latest-detections* nil)
  (setq *pot-handle-latest-pots* nil)
  (setq *pot-handle-latest-handles* nil))
