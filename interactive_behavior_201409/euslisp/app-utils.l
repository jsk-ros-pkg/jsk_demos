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

(defun search (subs s &key (from-end))
  (declare (string subs) (string s))
  (if (string= subs s) 0
      (let (j)
        (dotimes (i (1+ (- (length s) (length subs))))
          (setq j (if from-end (- (length s) (length subs) i) i))
          (when (string= subs (subseq s j (+ j (length subs))))
            (return-from search j))))))

(defun get-spot (name &optional (scene *scene*))
  (let* ((front-name (format nil "~A-front" (string-downcase name)))
         (front-spots (remove-if-not #'(lambda (x) (substringp front-name (string-downcase (send x :name)))) (send scene :spots)))
         (near-pos 10000000000)
         robot-pose cand-spots near-spot diff-pos)

    (ros::ros-debug "~A spot candidates found ~A" (length front-spots) (send-all front-spots :name))
    ;; 1. find with number suffix
    (dolist (s front-spots)
      (let ((suffix (subseq (send s :name) (+ (search front-name (string-downcase (send s :name)) :from-end t)
                                              (length front-name)))))
        (when (and (not (null-string-p suffix))
                   (numberp (read-from-string suffix)))
          (push (cons (read-from-string suffix) s) cand-spots))))
    (when cand-spots
      (sort cand-spots #'>= #'car)
      (setq cand-spots (list (cdar cand-spots))))
    ;; 2. find without suffix
    (unless cand-spots
      (setq cand-spots (remove-if-not #'(lambda (s) (string= front-name
                                                             (subseq (string-downcase (send s :name))
                                                                     (- (length (send s :name))
                                                                        (length front-name)))))
                                      front-spots)))

    (ros::ros-info "~A -> ~A" name cand-spots)

    (cond
     ((string= (string-downcase name) "start")
      (return-from get-spot (load-start-pose)))
     ((null cand-spots)
      (return-from get-spot nil))
     ((= (length cand-spots) 1)
      (return-from get-spot (car cand-spots)))
     (t
      (setq robot-pose (if (and (boundp '*ri*) *ri*) (send *ri* :state :worldcoords)))
      (unless robot-pose
        (return-from get-spot (car cand-spots)))
      (dolist (spot cand-spots)
        (setq diff-pos (norm (send robot-pose :difference-position spot)))
        (ros::ros-debug "diff-pos: ~A" diff-pos)
        (when (< diff-pos near-pos)
          (setq near-pos diff-pos near-spot spot)))
      (return-from get-spot near-spot)))))

(defun get-all-spots (&optional (scene *scene*))
  (let ((front-names (mapcar #'(lambda (x)
                                 (subseq (send x :name)
                                         (1+ (or (search "/" (send x :name) :from-end t) -1))
                                         (search "-front" (send x :name) :from-end t)))
                             (remove-if-not #'(lambda (x) (substringp "-front" (send x :name)))
                                            (send scene :spots))))
        spots spot)
    (dolist (name front-names)
      (setq spot (get-spot name scene))
      (when spot
        (push (cons name spot) spots)))
    spots))

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
