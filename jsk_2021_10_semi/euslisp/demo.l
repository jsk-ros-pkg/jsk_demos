(when (not (boundp '*room73b2*))
  (require "models/room73b2-scene.l")
  (setq *room73b2* (room73b2)))

(when (not (boundp '*nao*))
  (require "package://naoeus/nao.l")
  (setq *nao* (NaoH25V50)))

(when (not (boundp '*fetch*))
  (load "package://fetcheus/fetch.l")
  (load "package://fetcheus/fetch-interface.l")
  ;;(setq *fetch* (fetch))
  (fetch-init)
  )

(when (not (boundp '*spotkinova*))
  (require "package://spotkinovaeus/spotkinova.l")
  (setq *spotkinova* (spotkinova)))

(when (not (boundp '*pr2*))
  (require "package://pr2eus/pr2.l")
  (setq *pr2* (pr2)))

(objects (list *room73b2* *nao* *fetch* *pr2* *spotkinova*))

(defun wait-for-human ()
  ;;(send *pr2* :move-to (make-coords :pos #f(5000 0 0) :rpy (float-vector pi 0 0)) :world)
  (send *fetch* :move-to (make-coords :pos #f(5000 0 0) :rpy (float-vector pi 0 0)) :world)
  
  (send *irtviewer* :draw-objects)
  )

(defun write()
  (send *fetch* :move-to (make-coords :pos #f(5000 0 0) :rpy (float-vector pi 0 0)) :world)
  (send *fetch* :reset-pose)
  (send *fetch* :rarm :move-end-pos #f(0 -400 250) :local :rotation-axis t :debug-view t)
  (send *fetch* :rarm :move-end-rot -90 :z :local :debug-view t)
  (send *fetch* :rarm :move-end-rot -90 :x :local :debug-view t)
  (send *irtviewer* :draw-objects)
  )

(defun write_I()
  
  )

(defun move-forward ()
  (send *pr2* :translate #f(100 0 0 )))

;; (send dest :draw-on :flush t :size 1000)

(defun move-arms () ;; まずは体の位置を基準に腕を動かす
  (setq dest (send *pr2* :copy-coords)) ;; pr2の座標
  (send dest :translate #f(700 -200 1000)) ;; (2400 200 700)
  (send *pr2* :rarm :inverse-kinematics
	dest
	:link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	:move-target (send *pr2* :rarm :end-coords)
	:rotation-axis nil
	:thre 30
	:debug-view t)
  (objects (list *room73b2* *nao* *fetch* *pr2* (send *pr2* :rarm :end-coords))))

