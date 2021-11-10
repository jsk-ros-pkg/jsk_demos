(when (not (boundp '*room73b2*))
  (require "models/room73b2-scene.l")
  (setq *room73b2* (room73b2)))

(when (not (boundp '*nao*))
  (require "package://naoeus/nao.l")
  (setq *nao* (NaoH25V50)))

(when (not (boundp '*fetch*))
  (require "package://fetcheus/fetch.l")
  (setq *fetch* (fetch)))

(when (not (boundp '*pr2*))
  (require "package://pr2eus/pr2.l")
  (setq *pr2* (pr2)))

(objects (list *room73b2* *nao* *fetch* *pr2*))

(defun wait-for-human ()
  (send *pr2* :move-to (make-coords :pos #f(3000 000 0) :rpy (float-vector pi 0 0)) :world)
  (send *irtviewer* :draw-objects)
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
