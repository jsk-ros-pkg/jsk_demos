#!/usr/bin/env roseus
;; conv-utils.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(provide :conv-utils)
(ros::load-ros-manifest "mecab_ros")
(ros::roseus "conv_utils")

(defparameter *mecab-service* "/mecab/parse")

(defparameter *word-remapping-table*
  (list (cons "コップ" "cup")
        (cons "ボウル" "bowl")
        (cons "ペットボトル" "bottle")
        ))

(defparameter *feature-remapping-table*
  (list (cons "小さい" "small")
        (cons "大きい" "big")
        (cons "長い" "tall")
        (cons "短い" "short")
        (cons "白い" "white")
        (cons "黒い" "black")
        (cons "赤い" "red")
        (cons "青い" "blue")
        (cons "黄色い" "yellow")
        (cons "丸い" "round")
        (cons "四角い" "box")
        (cons "平べったい" "flat")))


(defun remap-word (word)
  "this should be overriden by disambiguotizer?"
  (if (derivedp word mecab_ros::POS) (setq word (send word :query)))
  (let ((trans (cdr (assoc word *word-remapping-table* :test #'string=))))
    (if trans trans word)))


(defun parse-preempt-speech (text)
  (unless (ros::wait-for-service *mecab-service* 1)
    (ros::ros-error "~A not advertised" *mecab-service*)
    (return-from parse-preempt-speech nil))
  (let ((req (instance mecab_ros::ParseRequest :init :text text))
        pos)
    (setq pos (send (ros::service-call *mecab-service* req) :parts))
    ;; not A but B
    ;; not A
    (let (idx before after)
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
      (setq before (mapcar #'remap-word before))
      (setq after (mapcar #'remap-word after))
      (cond
       ((and before after)
        (return-from parse-preempt-speech
          (list (cons :command :update)
                (cons :before (car (last before)))
                (cons :after (car (last after)))
                (cons :result pos))))
       ((and before (null after))
        (return-from parse-preempt-speech
          (list (cons :command :remove)
                (cons :before (car (last before)))
                (cons :result pos))))))

    ;; also A
    (let (idx after)
      (setq idx (position-if
                 #'(lambda (p)
                     (and (string= "助詞" (send p :pos))
                          (string= "も" (send p :original)))) pos))
      (when (numberp idx)
        (setq after (remove-if-not
                      #'(lambda (p)
                          (string= "名詞" (send p :pos))) (subseq pos 0 idx))))
      (setq after (mapcar #'remap-word after))
      (when after
        (return-from parse-preempt-speech
          (list (cons :command :add)
                (cons :after (car (last after)))
                (cons :result pos)))))
    
    ;; no command detected
    (list (cons :command nil)
          (cons :result pos))))

(defun remap-feature (word)
  (if (derivedp word mecab_ros::POS) (setq word (send word :original)))
  (let ((trans (cdr (assoc word *feature-remapping-table* :test #'string=))))
    (if trans trans word)))

(defun parse-object-description-speech (text)
  (unless (ros::wait-for-service *mecab-service* 1)
    (ros::ros-error "~A not advertised" *mecab-service*)
    (return-from parse-preempt-speech nil))
  (let ((req (instance mecab_ros::ParseRequest :init :text text))
        pos features names)
    (setq pos (send (ros::service-call *mecab-service* req) :parts))

    (dolist (p pos)
      (cond
       ((string= (send p :pos) "形容詞")
        (push (remap-feature p) features))
       ((string= (send p :pos) "連体詞")
        (send p :pos "形容詞")
        (send p :original
              (concatenate string (subseq s 0 (- (length s) 3)) "い"))
        (push (remap-feature p) features))
       ((string= (send p :pos) "名詞")
        (push (send p :original) names))))

    (list (cons :names names)
          (cons :features features))))
