(load "../models/door_with_wall.l")
(load "package://pr2eus/pr2.l")

(defun init ()
    (setq *pr2* (pr2))
    (setq *scene* (door_with_wall))
    (send *pr2* :move-to (make-coords :pos #f(-1500 0 0) :rpy #f(0 0 0)))
    (objects (list *pr2* *scene*))
)

;; このあとの方針
;; ドアを掴んで開けたあと，servo-offにしてそのまま肩で押していく
