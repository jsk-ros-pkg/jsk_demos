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

(when (not (boundp '*spotkinova*))
  (require "package://spotkinovaeus/spotkinova.l")
  (setq *spotkinova* (spotkinova)))

(objects (list *room73b2* *nao* *fetch* *pr2* *spotkinova*))

(defun wait-for-human ()
  (send *pr2* :move-to (make-coords :pos #f(5000 000 0) :rpy (float-vector pi 0 0)) :world)
  (send *irtviewer* :draw-objects)
  )
