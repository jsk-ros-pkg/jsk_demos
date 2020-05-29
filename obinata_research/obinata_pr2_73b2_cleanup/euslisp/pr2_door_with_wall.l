(load "../models/door_with_wall.l")
(load "package://pr2eus/pr2.l")

(defun init ()
    (setq *pr2* (pr2))
    (setq *scene* (door_with_wall))
    (send *pr2* :move-to (make-coords :pos #f(-1500 0 0) :rpy #f(0 0 0)))
    (setq *door* (send *door_with_wall* :object "room73b2-door-right"))
    ;; default jskeus
    ;; to assoc & dissoc
    (setq *rear-knob* (send *door* :handle-rear-knob))
    ;; 廊下側のノブ座標
    (setq *knob* (make-coords :pos #f(-115.0 -140.0 960.0) :rpy #f(-3.142 1.225e-16 1.571)))
    (objects (list *pr2* *scene*))
)

;; このあとの方針
;; ドアを掴んで開けたあと，servo-offにしてそのまま肩で押していく

(defun move-to-door()
    (send *pr2* :move-to (make-coords :pos #f(850 -400 0)))
    )


(defun grasp-knob()
    (send *pr2* :head :look-at (send *knob* :worldpos))
    ;; larm -> rarm?
    (send *pr2* :larm :inverse-kinematics
                *knob*
                :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
                :move-target (send *pr2* :larm :end-coords)
                :rotation-axis :z
                :debug-view t)
    )

(defun open-door())

(defun push-door())


