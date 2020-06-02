(load "package://pr2eus/pr2.l")
(load "models/room73b2-scene.l")
(load "package://pr2eus/pr2-interface.l")
;; chair
(load "models/room73b2-chair0-object.l")
;; can
(load "models/room73b2-georgia-emarald-mountain-object.l")

(defun init-irt()
    (setq *pr2* (pr2))
    (setq *room* (room73b2))
    ;; ROBOT_INITIAL_POSE
    (send *pr2* :move-to (make-coords :pos #f(750 0 0) :rpy #f(0 0 0)))
    ;; chairs
    (setq *chair_1* (room73b2-chair0))
    (setq *chair_2* (room73b2-chair0))
    (setq *chair_3* (room73b2-chair0))
    (send *chair_1* :transform (make-coords :pos #f(4000.0 -1000.0 0.1) :rpy #f(0.2 0.0 0.0)))
    (send *chair_2* :transform (make-coords :pos #f(4500.0 -300.0 0.1) :rpy #f(0.8 0.0 0.0)))
    (send *chair_3* :transform (make-coords :pos #f(6000.0 300.0 0.1) :rpy #f(1.4 0.0 0.0)))
    ;; trash
    (setq *can_1* (room73b2-georgia-emerald-mountain))
    
    (objects (list *pr2* *room* *chair_1* *chair_2* *chair_3*)))

(defun init-simulator()
    (setq *ri* (instance pr2-interface :init :objects (send *room* :objects)))
    (send *ri* :angle-vector (send *pr2* :reset-pose) 5000))

;; move chairs
(defun move-chairs()
    )

;; collect trash
(defun collect-trash)

;; collect box
(defun collect-box)