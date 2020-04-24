(load "package://pr2eus/pr2.l")
(load "models/room73b2-scene.l")
(load "package://pr2eus/pr2-interface.l")

(defun init ()
    (setq *pr2* (pr2))
    (setq *room* (room73b2))
    (setq *chair* (send *room* :object "room73b2-chair0"))
    (objects (list *pr2* *room*))
    (setq *ri* (instance pr2-interface :init :objects (send *room* :objects)))
)

(defun go-to-chair-spot ()
    ;; (setq *chair-spot* (send *chair* :worldpos))
    ;; kinematics-simulatorで斜めに移動するのを修正したほうがいいかも
    ;; 目標のイスの前まで移動
    (send *pr2* :move-to (make-coords :pos #f(4500 0 0) :rpy #f(0 0 0)) :world)
    ;; (send *ri* :move-to (make-coords :pos #f(4500 0 0) :rpy #f(0 0 0)))
    (send *pr2* :move-to (make-coords :rpy (float-vector pi/2 0 0)))
    ;; (send *ri* :move-to (make-coords :rpy (float-vector pi/2 0 0)))
    (send *pr2* :move-to (make-coords :pos #f(1600 0 0)))
    ;; (send *ri* :move-to (make-coords :pos #f(1600 0 0)))
    (send *pr2* :move-to (make-coords :rpy (float-vector pi/2 0 0)))
    ;; (send *ri* :move-to (make-coords :rpy (float-vector pi/2 0 0)))
    (send *ri* :move-to (make-coords :pos #f(4500 1600 0) :rpy (float-vector pi 0 0)))
    
)

(defun grab-chair-with-l-hand ()
    ;; look at the chair
    (send *pr2* :head :look-at (send *chair* :worldpos))
    ;; dissoc the chair and the room
    (if (send *chair* :parent) (send (send *chair* :parent) :dissoc *chair*))
    ;; solve ik and grab the chair
    (send *pr2* :larm :inverse-kinematics
                (car (send *chair* :handle))
                :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
                :move-target (send *pr2* :larm :end-coords)
                :rotation-axis :z
                :debug-view t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    ;; assoc the chair and the l-arm
    (send *pr2* :larm :end-coords :assoc *chair*)
    ;; grasp
    (send *ri* :start-grasp)
    (send *ri* :wait-interpolation)
    ;; pull back
    (send *pr2* :move-to (make-coords :pos #f(-350 0 0)))
    (send *ri* :go-pos -0.35 0)
    (send *ri* :wait-interpolation)
)

(defun remove-chair ()
    ;; remove the chair. This code has a little problem.
    (send *pr2* :larm :move-end-pos #f(200 -400 0) :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-rot 90 :z :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    ;; THIS IS OPTIONAL or should rotate more angle...
    (send *pr2* :larm :move-end-rot 30 :z :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
)

(defun grab-box-with-r-hand ()
    ;; go straight
    (send *pr2* :move-to (make-coords :pos #f(950 200 0)))
    ;; look at the box
    (setq *box* (send *room* :object "room73b2-empty-box"))
    (send *pr2* :head :look-at (send *box* :worldpos))
    ;; dissoc the box
    (if (send *box* :parent) (send (send *box* :parent) :dissoc *box*))
    (send *pr2* :rarm :inverse-kinematics
                (car (send *box* :handle))
                :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
                :move-target (send *pr2* :rarm :end-coords)
                :rotation-axis :z
                :debug-view t)
    ;; assoc the box and the r-arm
    (send *pr2* :rarm :end-coords :assoc *box*)
    ;; lift up
    (send *pr2* :rarm :move-end-pos #f(100 0 100) :parent
                :debug-view t :look-at-target t)
    ;; get back to the original pos
    (send *pr2* :move-to (make-coords :pos #f(-950 -200 0)))
    ;; reset pose only rarm
    (send *pr2* :angle-vector #f(26.5955 68.3427 49.1055 32.6661 -116.323 176.624 -61.6646 160.917 -60.0 74.0 -70.0 -120.0 -20.0 -30.0 180.0 54.9067 38.0989))
)

(defun return-chair ()
    (send *pr2* :larm :move-end-rot -120 :z :parent
                :debug-view t :look-at-target t)
    (send *pr2* :larm :move-end-pos #f(-200 500 0) :parent
                :debug-view t :look-at-target t)
    (send *pr2* :move-to (make-coords :pos #f(350 0 0)))
    (if (send *chair* :parent) (send (send *chair* :parent) :dissoc *chair*))
    (send *pr2* :move-to (make-coords :pos #f(-400 0 0)))
    (send *pr2* :reset-pose)
)

(defun main ()
    (init)
    (go-to-chair-spot)
    ;; (grab-chair-with-l-hand)
    ;; (remove-chair)
    ;; (grab-box-with-r-hand)
    ;; (return-chair)
)