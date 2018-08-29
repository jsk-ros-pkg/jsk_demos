#!/usr/bin/env roseus
;; conv-utils.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(provide :conv-utils)
(ros::load-ros-manifest "mecab_ros")
(defparameter *mecab-service* "/mecab/parse")

(defun parse-speech (text)
  (unless (ros::wait-for-service *mecab-service* 1)
    (ros::ros-error "~A not advertised" *mecab-service*)
    (return-from parse-speech nil))
  (let ((req (instance mecab_ros::ParseRequest :init :text text))
        pos idx before after)
    (setq pos (send (ros::service-call *mecab-service* req) :parts))
    (setq idx (position-if
               #'(lambda (p)
                   (and (string= "助動詞" (send p :pos))
                        (string= "ない" (send p :original)))) pos))
    (when (numberp idx)
      (setq before (remove-if-not
                    #'(lambda (p)
                        (string= "名詞" (send p :pos))) (subseq pos 0 idx)))
      (setq after (remove-if-not
                   #'(lambda (p)
                       (string= "名詞" (send p :pos))) (subseq pos idx))))
    (when (and before after)
      (return-from parse-speech
        (list (cons :command :update)
              (cons :before (send (car (last before)) :query))
              (cons :after (send (car (last after)) :query))
              (cons :result pos))))
    (list (cons :command nil)
          (cons :result pos))))
