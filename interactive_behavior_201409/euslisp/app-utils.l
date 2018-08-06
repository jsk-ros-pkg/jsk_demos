#!/usr/bin/env roseus
;; app-utils.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "run_app")
(setq *action-param* (ros::get-param "/action/parameters"))
(ros::ros-info "*action-param*: ~A" *action-param*)

(defun get-arg (key)
  (unless (stringp key)
    (setq key (string-downcase key)))
  (cdr (assoc key *action-param* :test #'string=)))

(defmacro with-timeout (timeout &rest body)
  (let ((gstart (gensym)))
  `(let ((,gstart (ros::time-now)))
     (ros::rate 5)
     (while (and (ros::ok)
                 (< (send (ros::time- (ros::time-now) ,gstart) :to-sec) ,timeout))
       (prog1
           (progn ,@body)
         (ros::spin-once)
         (ros::sleep))))))

(defmacro wait-until (timeout test-func)
  (let ((gblock (gensym)))
    `(block ,gblock
       (with-timeout
           ,timeout
           (if (funcall ,test-func)
               (return-from ,gblock t)))
       (return-from ,gblock nil))))

(defmacro wait-until-set (timeout var)
  (let ((gblock (gensym)))
    `(block ,gblock
       (with-timeout
           ,timeout
           (if ,var (return-from ,gblock t)))
       (return-from ,gblock nil))))

(defmacro wait-until-bound (timeout sym)
  (let ((gblock (gensym)))
    `(block ,gblock
       (with-timeout
           ,timeout
           (if (boundp ',var) (return-from ,gblock t)))
       (return-from ,gblock nil))))

(defmacro with-subscribe (sub &rest body)
  `(progn
     (ros::subscribe ,@sub)
     (unwind-protect (progn ,@body)
       (ros::unsubscribe ,(car sub)))))

(defun person->place (person)
  (let (place)
    (when (stringp person)
      (setq person (intern (string-upcase person))))
    (case person
      ('asano (setq place "73B1"))
      ('otsubo (setq place "73B2"))
      ('furushchev (setq place "73A3"))
      ('okada (setq place "73A2"))
      ('inaba (setq place "73A1"))
      (t t))
    place))

(defun get-spot (scene robot name)
  (let* ((front-spots (remove-if-not #'(lambda (x)
                                         (string= "-front"
                                                  (subseq (send x :name) (- (length (send x :name))
                                                                            (length "-front")))))
                                     (send scene :spots)))
         (cand-spots (remove-if-not #'(lambda (x)
                                        (substringp (string-downcase name)
                                                    (string-downcase (send x :name))))
                                    front-spots))
         (robot-pose (send *ri* :state :worldcoords))
         (near-pos 10000000000)
         near-spot diff-pos)
    (cond
      ((null cand-spots)
       (return-from get-spot nil))
      ((= (length cand-spots) 1)
       (return-from get-spot (car cand-spots)))
      (t
       (dolist (spot cand-spots)
         (setq diff-pos (norm (send robot-pose :difference-position spot)))
         (when (< diff-pos near-pos)
           (setq near-pos diff-pos near-spot spot)))
       (return-from get-spot near-spot)))))


(provide :app-utils)
