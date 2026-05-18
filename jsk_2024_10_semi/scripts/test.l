(require "package://pr2eus/pr2.l")  ;;import pr2 package
; (require "package://pr2eus/pr2-utils.l")
; (require "package://pr2eus/pr2-interface.l") 
; (if (not (boundp '*pr2*)) (pr2-init)) ;; (setq *pr2* (pr2)))  ;; (pr2) is alternated by  *pr2*
(if (not (boundp '*pr2*)) (setq *pr2* (pr2)))

(send *pr2* :reset-pose) ;; 初期姿勢
; (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
; (send *ri* :wait-interpolation)
(objects (list *pr2*))

(send *pr2* :larm :shoulder-p :joint-angle 50)
(send *pr2* :larm :shoulder-r :joint-angle -20)
(send *pr2* :larm :wrist-p :joint-angle 20)
(send *pr2* :rarm :shoulder-p :joint-angle 50)
(send *pr2* :rarm :shoulder-r :joint-angle 20)
(send *pr2* :rarm :wrist-p :joint-angle -20)
(send *pr2* :head :neck-p :joint-angle 50)
(send *irtviewer* :draw-objects)
; (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
; (send *ri* :wait-interpolation)

(send *pr2* :larm :shoulder-p :joint-angle 5)
(send *pr2* :rarm :shoulder-p :joint-angle 5)
(send *pr2* :head :neck-p :joint-angle -10)
(send *irtviewer* :draw-objects)
; (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
; (send *ri* :wait-interpolation)

