(load "package://pr2eus/pr2.l")
(load "models/room73b2-scene.l")
(load "package://pr2eus/pr2-interface.l")
(load "trashmodel.l")
(load "toolmodel.l")
(load "movement.l")
;; ;; chair
;; (load "models/room73b2-chair0-object.l")
;; ;; can
;; (load "models/room73b2-georgia-emerald-mountain-object.l")
;; (load "models/room73b2-wanda-object.l")
;; ;; iemon
;; (load "models/room73b2-iemon-object.l")
;; ;; bottle
;; (load "models/room73b2-bottle-object.l")


(defun init-irt()
    (setq *pr2* (pr2))
    (setq *room* (room73b2))
    ;; ROBOT_INITIAL_POSE
    (send *pr2* :move-to (make-coords :pos #f(750 0 0) :rpy #f(0 0 0)))
    ;; chairs <- box1
    (setq *chair_1* (room73b2-chair0))
    (setq *chair_2* (room73b2-chair0))
    (setq *chair_3* (room73b2-chair0))
    (send *chair_1* :transform (make-coords :pos #f(4000.0 -1000.0 0.1) :rpy #f(0.2 0.0 0.0)))
    (send *chair_2* :transform (make-coords :pos #f(4500.0 -300.0 0.1) :rpy #f(0.8 0.0 0.0)))
    (send *chair_3* :transform (make-coords :pos #f(6000.0 300.0 0.1) :rpy #f(1.4 0.0 0.0)))
    ;; trash
    (setq *can_1* (room73b2-georgia-emerald-mountain))
    (setq *can_2* (room73b2-wanda))
    (setq *carton_1* (room73b2-iemon))
    (setq *bottle_1* (room73b2-bottle))
    ;; https://github.com/euslisp/EusLisp/pull/444
    (setq *trashbox* (trashbox))
    (send *can_1* :transform (make-coords :pos #f(7500.0 1000.0 30.0) :rpy #f(0.0 0.0 1.571)))
    (send *can_2* :transform (make-coords :pos #f(9500.0 2000.0 30.0) :rpy #f(0.0 0.0 1.571)))
    (send *carton_1* :transform (make-coords :pos #f(8000.0 3000.0 0.0) :rpy #f(0.0 0.0 1.571)))
    (send *bottle_1* :transform (make-coords :pos #f(11000.0 3000.0 45.0) :rpy #f(0.0 0.0 1.571)))
    (send *trashbox* :transform (make-coords :pos #f(1000.0 -1000.0 0.0) :rpy #f(0 0 0)))
    ;; mini box
    (setq *box_1* (room73b2-empty-box))
    (setq *box_2* (room73b2-empty-box))
    (send *box_1* :transform (make-coords :pos #f(11200.0 0.0 675.0) :rpy #f(0.0 0.0 0.0)) :world)
    (send *box_2* :transform (make-coords :pos #f(11200.0 1000.0 675.0) :rpy #f(0.0 0.0 0.0)) :world)
    ;; eliminate objects on the table
    (send *room* :remove-object "room73b2-bottle")
    (send *room* :remove-object "room73b2-sushi-cup2")
    (send *room* :remove-object "room73b2-empty-box")
    ;; eliminate objects on the kitchen
    (send *room* :remove-object "room73b2-knife")
    
    ;; 2900.0 1800.0 700.0

    ;; meal setting
    ;; define object
    (setq *mc-bag-1* (instance mc-paper-bag-s :init))
    (setq *mc-bag-2* (instance mc-paper-bag-m :init))
    (setq *mc-cup-1* (mc-cup))
    (setq *mc-cup-2* (mc-cup))

    (send *mc-bag-1* :transform (make-coords :pos #f(2900 2000 700) :rpy #f(0 0 0)) :world)
    (send *mc-bag-2* :transform (make-coords :pos #f(3200 2000 700) :rpy #f(0 0 0)) :world)
    (send *mc-cup-1* :transform (make-coords :pos #f(2900 1500 700) :rpy #f(0 0 0)) :world)
    (send *mc-cup-2* :transform (make-coords :pos #f(3300 1800 700) :rpy #f(0 0 0)) :world)
    
    ;; petbottle empty and filled

    ;; view
    (objects (list *pr2* *room* *chair_1* *chair_2* *chair_3* *can_1* *can_2* *carton_1* *bottle_1* *trashbox* *box_1* *box_2* *mc-bag-1* *mc-bag-2* *mc-cup-1* *mc-cup-2*))
    )

(defun init-simulator()
    (setq *ri* (instance pr2-interface :init :objects (send *room* :objects)))
    (send *ri* :angle-vector (send *pr2* :reset-pose) 5000))

;; start clean-up
;; go around the room and find out what to do
(defun walk-around()
    (send *pr2* :move-to (make-coords :pos #f(2500 0 0)))
    (send *pr2* :move-to (make-coords :pos #f(0 0 0) :rpy #f(0.7 0 0)))
    (send *pr2* :move-to (make-coords :pos #f(1800 0 0)))
    (send *pr2* :move-to (make-coords :rpy #f(0.871 0 0)))
    (send *pr2* :move-to (make-coords :pos #f(900 0 0)))
    (send *pr2* :move-to (make-coords :rpy #f(3.141 0 0)))
    )

;; calculate which is better, cleaning up the table or moving the chairs
(defun calculate-cost())

;; move chairs
(defun move-chairs()
    (send *pr2* :move-to (make-coords :pos #f(1300 0 0)))
    (grasp *pr2* *chair_2* :rarm)
    ;; [WIP]head look-at front
    (send *pr2* :move-to (make-coords :pos #f(300 0 0)))
    (send *pr2* :move-to (make-coords :rpy #f(0.7 0 0)))
    (send *pr2* :move-to (make-coords :pos #f(400 0 0)))
    (grasp *pr2* *chair_3* :larm)
    (send *pr2* :move-to (make-coords :pos #f(1500 0 0)))
    (send *pr2* :move-to (make-coords :rpy #f(-0.7 0 0)))
    (send *pr2* :move-to (make-coords :pos #f(500 0 0)))

    ;; rotate rarm
    (send *pr2* :rarm :move-end-rot -45 :z :parent
    :debug-view t 
    :look-at-target t)
    ;; release
    (if (send *chair_2* :parent) (send (send *chair_2* :parent) :dissoc *chair_2*))
    (send *pr2* :move-to (make-coords :pos #f(-200 0 0)))
    (send *pr2* :move-to (make-coords :pos #f(0 400 0)))

    ;; *chair_3* NOT COMPLETED
    ;; move larm
    ;; (send *pr2* :larm :move-end-pos #f(0 -400 0) :parent
    ;; :debug-view t 
    ;; :look-at-target t)
    (if (send *chair_3* :parent) (send (send *chair_3* :parent) :dissoc *chair_3*))

    (send *pr2* :move-to (make-coords :pos #f(-300 0 0)))
    (send *pr2* :reset-pose)

    (send *pr2* :move-to (make-coords :rpy #f(-1.571 0 0)))
    (send *pr2* :move-to (make-coords :pos #f(1400 0 0)))
    (send *pr2* :move-to (make-coords :rpy #f(-1.571 0 0)))
    (send *pr2* :move-to (make-coords :pos #f(1000 0 0)))
    (send *pr2* :move-to (make-coords :rpy #f(1.571 0 0)))
    (send *pr2* :move-to (make-coords :pos #f(1000 0 0)))
    (send *pr2* :move-to (make-coords :rpy #f(1.571 0 0)))

    ;; grasp chair1
    ;; it had better to hug the chair...
    (grasp *pr2* *chair_1* :larm)
    (send *pr2* :move-to (make-coords :pos #f(1400 300 0)))
    (if (send *chair_1* :parent) (send (send *chair_1* :parent) :dissoc *chair_1*))
    (send *pr2* :move-to (make-coords :pos #f(-1200 0 0)))

    (send *pr2* :reset-manip-pose)
    )

;; collect trash on the dining table
(defun collect-trash-and-dispose()
    ;; (send *pr2* :move-to (make-coords :rpy #f(-1.571 0 0)))
    ;; (send *pr2* :move-to (make-coords :pos #f(800 0 0)))
    ;; (send *pr2* :move-to (make-coords :rpy #f(-1.571 0 0)))
    ;; (send *pr2* :move-to (make-coords :pos #f(600 0 0)))
    
    ;; move to trashbox
    (send *pr2* :move-to (make-coords :pos #f(1700 -1000 0) :rpy #f(3.141 0 0)) :world)


    (send *pr2* :move-to (make-coords :rpy #f(3.141 0 0)))
    (send *pr2* :move-to (make-coords :pos #f(1500 -400 0)))
    (send *pr2* :move-to (make-coords :rpy #f (1.571 0 0)))


    (setq *room-chair0* (send *room* :object "room73b2-chair0"))
    (setq *room-chair1* (send *room* :object "room73b2-chair1"))
    (grasp *pr2* *room-chair0* :rarm)
    (send *pr2* :rarm :move-end-pos #f(400 600 0) :parent :debug-view t :look-at-target t)
    ;; or #f(400 400 0) + 2 * #f(0 100 0)
    (send *pr2* :reset-manip-pose)
    (send *pr2* :move-to (make-coords :pos #f(600 -200 0)))
    
    ;; [WIP] teleport the cup
    (send *mc-cup-1* :move-to (make-coords :pos #f(500 0 0)))


)

;; collect box
(defun collect-box)