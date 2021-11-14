(when (not (boundp '*room73b2*))
  (require "models/room73b2-scene.l")
  (setq *room73b2* (room73b2)))

(when (not (boundp '*nao*))
  (require "package://naoeus/nao.l")
  (setq *nao* (NaoH25V50)))

(when (not (boundp '*fetch*))
  (require "package://fetcheus/fetch.l")
  (setq *fetch* (fetch)))

(when (not (boundp '*spotkinova*))
  (require "package://spotkinovaeus/spotkinova.l")
  (setq *spotkinova* (spotkinova)))

(when (not (boundp '*pr2*))
  (require "package://pr2eus/pr2.l")
  (setq *pr2* (pr2)))

(objects (list *room73b2* *nao* *fetch* *pr2* *spotkinova*))

(defun wait-for-human ()
  (send *nao* :locate #f(50 0 350) :worldpos)
  (send *nao* :reset-pos)
  (send *irtviewer* :draw-objects)
  )

(defun move-to-spot(spot)
  (let ((coods-dif (send *nao* :transformation (send *room73b2* :spot spot))))

    (send *nao* :calc-walk-pattern-from-footstep-list (send *nao* :go-pos-params->footstep-list (aref (send coods-dif :pos) 0) (aref (send coods-dif :pos) 1) (rad2deg (elt (car (euler-angle (send coods-dif :rot))) 2)))
	  :debug-view
	  :no-message
	  :default-step-height 10)
  )
  nil
  )

(defun all-task ()
  (wait-for-human)
  (move-to-spot "fridge-front-spot")
 )
