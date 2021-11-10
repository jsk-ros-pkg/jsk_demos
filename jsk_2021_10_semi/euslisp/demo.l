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

(when (not (boundp '*pr2*))
  (require "package://pr2eus/pr2.l")
  (setq *pr2* (pr2)))

(objects (list *room73b2* *nao* *fetch* *pr2*))

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

;; (send *fetch* :rarm :move-end-pos #f(50 0 50) :local :rotation-axis t :debug-view t)
;; (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)

(defun write_I()
  
  )
