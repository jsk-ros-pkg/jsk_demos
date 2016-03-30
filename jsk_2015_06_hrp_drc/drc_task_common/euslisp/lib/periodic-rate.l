;; class like rospy.Rate but works without ros
(defclass periodic-rate
  :super propertied-object
  :slots (rate started timer))

(defmethod periodic-rate
  (:init (r)
    (setq rate r)
    (setq timer (instance mtimer :init))
    )
  (:sleep ()
    (if started
        (progn
          (let ((processed-time (send timer :stop)))
            (let ((rest-time (- (/ 1.0 rate) processed-time)))
              (if (> rest-time 0)
                  (unix::usleep (floor (* 1000000.0 rest-time))))
              ))
          (send timer :start)
          )
      (progn
        (unix::usleep (floor (* 1000000.0 (/ 1.0 rate))))
        (send timer :start)
        (setq started t)
        )
      ))
  )

(defun test-periodic-rate ()
  (let ((r (instance periodic-rate :init 1)))
    (dotimes (i 10)
      (print '1hz)
      (send r :sleep))
    )
  (let ((r (instance periodic-rate :init 5)))
    (dotimes (i 100)
      (print '5hz)
      (send r :sleep))
    )
  )
;; (load "package://drc_task_common/euslisp/lib/periodic-rate.l")