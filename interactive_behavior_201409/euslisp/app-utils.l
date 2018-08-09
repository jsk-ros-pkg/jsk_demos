#!/usr/bin/env roseus
;; app-utils.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(provide :app-utils)

(ros::roseus "run_app")
(defparameter *action-param* (ros::get-param "/action/parameters"))
(defparameter *start-spot-param-key* "/action/start_spot")
(ros::ros-info "*action-param*: ~A" *action-param*)
;;
(require :pr2-actionlib "package://interactive_behavior_201409/pddl/pr2-actionlib.l")
(require :room-domain "package://interactive_behavior_201409/pddl/room-domain.l")

(defun get-arg (key)
  (unless (stringp key)
    (setq key (string-downcase key)))
  (let ((val (cdr (assoc key *action-param* :test #'string=))))
    (if (or (null val) (null-string-p val)) nil (read-from-string val))))

(defmacro run-exit (&rest form)
  `(if (find-if #'(lambda (s) (substringp "__log:=" s)) lisp::*eustop-argument*)
     (if (progn ,@form) (exit 0) (exit 1))
     (warn ";; run ~A~%" '(progn ,@form))))

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
      ('asano (setq place '73B1))
      ('otsubo (setq place '73B2))
      ('furushchev (setq place '73A3))
      ('okada (setq place '73A2))
      ('inaba (setq place '73A1))
      (t t))
    place))

(defun get-spot (name &optional (scene *scene*))
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
     ((string= (string-downcase name) "start")
      (return-from get-spot (load-start-pose)))
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

(defun save-start-pose ()
  (let ((cur-cds (send *ri* :state :worldcoords))
        vec)
    (setq vec (coerce (send cur-cds :worldpos) cons))
    (setq vec (append vec (car (send cur-cds :rpy-angle))))
    (ros::set-param *start-spot-param-key* vec)))

(defun load-start-pose ()
  (if (and (ros::has-param *start-spot-param-key*)
           (ros::get-param *start-spot-param-key*))
      (let ((vec (ros::get-param *start-spot-param-key*))
            cds)
        (setq cds (make-cascoords))
        (send cds :locate (coerce (subseq vec 0 3) float-vector))
        (apply #'send cds :rpy (subseq vec 3))
        cds)
    (send *ri* :state :worldcoords)))
