#!/usr/bin/env roseus
;; go.l

(ros::roseus "run_go")

(setq *param* (ros::get-param "/action/parameters"))

(ros::ros-info "*param*: ~A" *param*)

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")

(defun pr2-init (&optional (create-viewer))
  (setq *pr2* (pr2)
        *ri* (instance pr2-interface :init
                       :move-base-action-name "elevator_move_base"))
  (ros::spin-once)
  (send *ri* :spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))

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

(defun main (spot-name)
  (pr2-init)
  (setq *scene* (make-eng2-scene))

  (setq *spot* (get-spot *scene* *ri* spot-name))
  (unless *spot*
    (ros::ros-error "Destination '~A' not found" spot-name)
    (return-from main nil))
  (ros::ros-info "Going to ~A" (send *spot* :name))
  (speak-jp (format nil "~Aに行きます" (send *spot* :name)))

  (send *ri* :move-to *spot*)
  (return-from main t))



(if (main (cdr (assoc "place" *param* :test #'string=)))
    (exit 0) (exit 1))
