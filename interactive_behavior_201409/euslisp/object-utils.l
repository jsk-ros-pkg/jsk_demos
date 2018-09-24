#!/usr/bin/env roseus
;; object-utils.l
;; Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(provide :object-utils)

(defparameter *object-labels*
  (list "person" "bicycle" "car" "motorcycle" "airplane" "bus" "train" "truck" "boat" "traffic light" "fire hydrant" "stop sign" "parking meter" "bench" "bird" "cat" "dog" "horse" "sheep" "cow" "elephant" "bear" "zebra" "giraffe" "backpack" "umbrella" "handbag" "tie" "suitcase" "frisbee" "skis" "snowboard" "sports ball" "kite" "baseball bat" "baseball glove" "skateboard" "surfboard" "tennis racket" "bottle" "wine glass" "cup" "fork" "knife" "spoon" "bowl" "banana" "apple" "sandwich" "orange" "broccoli" "carrot" "hot dog" "pizza" "donut" "cake" "chair" "couch" "potted plant" "bed" "dining table" "toilet" "tv" "laptop" "mouse" "remote" "keyboard" "cell phone" "microwave" "oven" "toaster" "sink" "refrigerator" "book" "clock" "vase" "scissors" "teddy bear" "hair drier" "toothbrush"))


(defun string-split (str &optional (delim #\Space) (start 0))
  (while (and (< start (length str))
              (eq (aref str start) delim))
    (incf start))
  (if (< start (1- (length str)))
      (let ((end (or (position delim str :start start) (length str))))
        (append (list (subseq str start end)) (string-split str delim end)))))

(defun get-object-label (target)
  (let ((name (string-downcase target)))
    (nsubstitute #\Space #\_ name)
    (nsubstitute #\Space #\- name)
    (setq name (string-split name))

    (dotimes (i (length *object-labels*))
      (when (every #'(lambda (p) (substringp p (elt *object-labels* i))) name)
        (return-from get-object-label i)))))
