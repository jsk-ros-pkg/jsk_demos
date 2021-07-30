;;; parameters for plug
(setq *plug-to-pre-grasp-pose*
    (make-cascoords
        :pos #f(1.07 -100.66 -39.27)
        :rpy #f(1.563 0.025 3.122)
        ))
(setq *plug-to-grasp-pose*
    (make-cascoords
        :pos #f(-6.21 15.0 -32.68)
        :rpy #f(1.575 0.016 -3.121)
        ))
(setq *plug-frame* "plug_frame")

;;; parameters for socket
(setq *socket-to-pre-insert-pose*
    (make-cascoords
      :pos #f(-6.05 -125.96 -37.01)
      :rpy #f(1.573 0.024 3.105)
      ))
(setq *socket-to-insert-pose*
    (make-cascoords
      :pos #f(-6.21992330507 -104.447650348 -37.4935173654)
      :rpy #f(1.546 0.037 3.107)
      ))
(setq *socket-to-too-plugged-pose*
    (make-cascoords
        :pos #f(-2.66183125247 -70 -42.614939075)
        :rpy #f(1.575 -0.001 -3.137)
        ))
(setq *socket-to-plugged-pose*
    (make-cascoords
        :pos #f(-2.66183125247 -85.6109378176 -42.614939075)
        :rpy #f(1.575 -0.001 -3.137)
        ))
(setq *socket-frame* "socket_frame")

;;; parameters for holder
(setq *holder-to-pre-insert-pose*
    (make-cascoords
      :pos #f(-25.6 -130 92.4)
      :rpy #f(1.610 1.230 -3.089)
      ))
(setq *holder-to-insert-pose*
    (make-cascoords
      :pos #f(-19.1765897755 -98.7200844605 48.5858921497)
      :rpy #f(1.644 1.133 -3.058)
      ))
(setq *holder-frame* "holder_frame")

;;; pre-reset-pose TODO
(setq *pre-reset-pose*
      #f())
