(load "package://fetcheus/fetch-interface.l")

(unless (boundp '*ri*) (fetch-init))
(objects (list *fetch*))

(send *fetch* :reset-pose)
(send *irtviewer* :draw-objects)

(send *fetch* :rarm :shoulder-y :joint-angle 30)
(send *fetch* :rarm :shoulder-r :joint-angle -120)
(send *fetch* :rarm :elbow-r :joint-angle -30)
(send *irtviewer* :draw-objects)
