(load "package://pr2eus/pr2.l")
(load "models/room73b2-scene.l")
(load "package://pr2eus/pr2-interface.l")

(defun init ()
    (setq *pr2* (pr2))
    (setq *room* (room73b2))
    (setq *chair* (send *room* :object "room73b2-chair0"))
    (objects (list *pr2* *room*))
    (setq *ri* (instance pr2-interface :init :objects (send *room* :objects)))
    (send *ri* :angle-vector (send *pr2* :reset-pose) 5000)
)

(defun go-to-chair-spot ()
    ;; (setq *chair-spot* (send *chair* :worldpos))
    ;; kinematics-simulatorで斜めに移動するのを修正したほうがいいかも
    ;; 目標のイスの前まで移動
    (send *pr2* :move-to (make-coords :pos #f(4500 0 0) :rpy #f(0 0 0)) :world)
    ;; init ri is -x 0.5
    (send *ri* :go-pos-unsafe 4 0 0)
    (send *pr2* :move-to (make-coords :rpy (float-vector pi/2 0 0)))
    (send *ri* :go-pos-unsafe 0 0 90)
    (send *pr2* :move-to (make-coords :pos #f(1600 0 0)))
    (send *ri* :go-pos-unsafe 1.6 0 0)
    (send *pr2* :move-to (make-coords :rpy (float-vector pi/2 0 0)))
    (send *ri* :go-pos-unsafe 0 0 90)
    ;; (send *ri* :move-to (make-coords :pos #f(4500 1600 0) :rpy (float-vector pi 0 0)))
)

(defun )