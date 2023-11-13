#!/usr/bin/env roseus


(defun one-shot-subscribe-with-condition (topic-name mclass &key (timeout) (after-stamp) (unsubscribe t)
                                                     (condition))
  "Subscribe message, just for once"
  (let (lmsg)
    (unless (ros::get-num-publishers topic-name)
      (cond
       (after-stamp
        (ros::subscribe topic-name mclass
          #'(lambda (msg)
              (let ((st (send msg :header :stamp)))
                (when (> (send st :to-sec)
                         (send after-stamp :to-sec))
                  (setq lmsg msg))))))
       (t
        (ros::subscribe topic-name mclass
          #'(lambda (msg) (setq lmsg msg))))))
    (let ((finishtm (if timeout (ros::time-now))))
      (when finishtm
        (setq finishtm (ros::time+ finishtm (ros::time (/ timeout 1000.0)))))
      (while (not (and finishtm
                       (< (send (ros::time- finishtm (ros::time-now)) :to-Sec) 0)))
        (unix::usleep (* 50 1000))
        (ros::spin-once)
        (when condition
          (when (and lmsg (funcall condition lmsg))
            (return)))
        (unless condition
          (if lmsg (return)))))
    (if unsubscribe (ros::unsubscribe topic-name))
    lmsg))
