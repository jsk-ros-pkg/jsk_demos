(load "models/arrow-object.l")
(setq *arrow1* (arrow))
(setq *arrow2* (arrow))
(setq *arrow3* (arrow))

;; (load "curry-dish.l")
;; (setq *curry-dish* (curry-dish))
;; (send *curry-dish* :translate #f(100 0 0) :world)
;; (send *arrow1* :newcoords (send (send *curry-dish* :attention-attention0) :copy-worldcoords))

;; (objects (list *curry-dish* *arrow1*))


;; (load "sato-rice.l")
;; (setq *sato-rice* (sato-rice))
;; (send *arrow1* :newcoords (send (send *sato-rice* :attention-attention0) :copy-worldcoords))

;; (objects (list *sato-rice* *arrow1*))

(load "cutting-board.l")
(setq *cutting-board* (cutting-board))
(send *arrow1* :newcoords (send (send *cutting-board* :attention-attention0) :copy-worldcoords))

(objects (list *cutting-board* *arrow1*))
