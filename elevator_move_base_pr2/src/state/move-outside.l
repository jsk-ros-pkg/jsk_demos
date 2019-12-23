(require :pr2-interface "package://pr2eus/pr2-interface.l")

(load "package://elevator_move_base_pr2/src/utils.l")


(defvar *scene*)
(defvar *tfl*)


(defun move-outside (userdata)
  (unix:sleep 3)  ;; Wait for door opening
  (speak-jp "エレベータから降ります。")
  (clear-costmap)
  (if (send *ri* :move-to (cdr (assoc 'outside-coords userdata)) :retry 2)
      t
    (progn
      (speak-jp "エレベータから降りられませんでした。")
      ;; Return to elevator_inside_panel-front
      (setq panel-front-coords
            (send *scene* :transformation
                  (car (send *scene* :find-object
                             (format nil "~A/elevator_inside_panel-front"
                                     (check-current-floor *scene* *tfl*))))))
      (send panel-front-coords :name (send *scene* :name))
      (send *ri* :move-to panel-front-coords :retry 2)
      nil)))
