(require "package://pr2eus/pr2.l")  
(if (not (boundp '*pr2*)) (setq *pr2* (pr2)))

(objects (list *pr2*))
(send *pr2* :larm :shoulder-p :joint-angle 10)

(send *pr2* :rarm :shoulder-p :joint-angle -20)
(send *pr2* :larm :shoulder-p :joint-angle 50)
(send *pr2* :rarm :shoulder-r :joint-angle -130)
(send *pr2* :larm :shoulder-r :joint-angle 120)
(send *pr2* :larm :elbow-p :joint-angle -100)
(send *pr2* :rarm :elbow-p :joint-angle -70)
(send *pr2* :rarm :elbow-r :joint-angle 0)
(send *pr2* :rarm :wrist-p :joint-angle -40)
(send *pr2* :larm :wrist-r :joint-angle 30)

(send *irtviewer* :draw-objects)
