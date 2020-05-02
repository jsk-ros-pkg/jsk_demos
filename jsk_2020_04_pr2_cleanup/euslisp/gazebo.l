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

(defun grab-chair-with-l-hand ()
    (setq *pre-chair* (make-coords :pos #f(3900 1335.0 780) :rpy #f(2.88 0 0)))
    ;; look at the chair
    (send *pr2* :head :look-at (send *chair* :worldpos))
    ;; dissoc the chair and the room
    (if (send *chair* :parent) (send (send *chair* :parent) :dissoc *chair*))
    ;; pre chair
    (send *pr2* :larm :inverse-kinematics
                *pre-chair*
                :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
                :move-target (send *pr2* :larm :end-coords)
                :rotation-axis :z
                :debug-view t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp :larm)
    ;; solve ik and grab the chair
    (send *pr2* :larm :inverse-kinematics
                (car (send *chair* :handle))
                :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
                :move-target (send *pr2* :larm :end-coords)
                :rotation-axis :z
                :debug-view t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;; assoc the chair and the l-arm
    (send *pr2* :larm :end-coords :assoc *chair*)
    ;; grasp
    (send *ri* :start-grasp :larm)
    (send *ri* :wait-interpolation)
    ;; pull back
    (send *pr2* :move-to (make-coords :pos #f(-350 0 0)))
    (send *ri* :go-pos-unsafe -0.35 0 0)
    (send *ri* :wait-interpolation)
)

(defun remove-chair ()
    ;; remove the chair. This code has a little problem.
    (send *pr2* :larm :move-end-pos #f(200 -400 0) :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    ;; need debug
    ;; 15 15 (100 -50 0) 15 (100 -50 0) (100 -50 0) 15
    ;; I think it could be wrote much better...
    (send *pr2* :larm :move-end-rot 15 :z :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-rot 15 :z :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-pos #f(100 -50 0) :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-rot 15 :z :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-pos #f(100 -50 0) :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-pos #f(100 -50 0) :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-rot 15 :z :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
)

(defun grab-box-with-r-hand ()
    (setq *box* (send *room* :object "room73b2-empty-box"))
    (setq *pre-box* (make-coords :pos #f(3915 1600 770) :rpy #f(0 0 0)))
    (send *pr2* :head :look-at (send *box* :worldpos))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    ;; make grasp pose
    ;; ik
    (send *pr2* :rarm :inverse-kinematics
                *pre-box*
                :link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
                :move-target (send *pr2* :rarm :end-coords)
                :rotation-axis :z
                :debug-view t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp :rarm)
    ;; go straight
    (send *pr2* :move-to (make-coords :pos #f(850 0 0)))
    (send *ri* :go-pos-unsafe 0.85 0 0)
    ;; dissoc the box
    (if (send *box* :parent) (send (send *box* :parent) :dissoc *box*))
    ;; assoc the box and the r-arm
    (send *pr2* :rarm :end-coords :assoc *box*)
    ;; grab
    (send *ri* :start-grasp :rarm)
    (send *ri* :wait-interpolation)
    ;; lift up
    ;; (send *pr2* :rarm :move-end-pos #f(100 0 100) :parent
    ;;             :debug-view t :look-at-target t)
    ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    ;; (send *ri* :wait-interpolation)
    ;; get back to the original pos
    (send *pr2* :move-to (make-coords :pos #f(-850 0 0)))
    (send *ri* :go-pos-unsafe -0.85 0 0)
    ;; reset pose only rarm
    (send *pr2* :angle-vector #f(49.9755 118.818 30.5735 58.6601 -89.6712 44.3389 -71.5621 242.034 -60.0 74.0 -70.0 -120.0 -20.0 -30.0 180.0 76.848 42.9233))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
)

(defun return-chair ()
    (send *pr2* :larm :move-end-rot -15 :z :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-pos #f(-100 50 0) :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-pos #f(-100 50 0) :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-rot -15 :z :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-pos #f(-100 50 0) :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-rot -15 :z :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-rot -15 :z :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-pos #f(-200 400 0) :parent
                :debug-view t :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *pr2* :move-to (make-coords :pos #f(350 0 0)))
    (send *ri* :go-pos-unsafe 0.35 0 0)
    ;; release the chair
    (if (send *chair* :parent) (send (send *chair* :parent) :dissoc *chair*))
    (send *ri* :stop-grasp :larm)
    (send *ri* :wait-interpolation)
    (send *pr2* :move-to (make-coords :pos #f(-400 0 0)))
    (send *ri* :go-pos-unsafe -0.4 0 0)
    (send *pr2* :reset-pose)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
)

(defun main ()
    (init)
    (go-to-chair-spot)
    (grab-chair-with-l-hand)
    (remove-chair)
    (grab-box-with-r-hand)
    (return-chair)
)