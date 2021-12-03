#!/usr/bin/env roseus

;;; parameters for cable
(setq *front-of-cable-pose*
    (make-cascoords
        :pos #f(1.07 -100.66 -39.27)
        :rpy #f(1.563 0.025 3.122)
        ))
(setq *cable-front-to-pre-grasp-pose*
    (make-cascoords
        :pos #f(-37.691 11.443 -7.628)
        :rpy #f(-1.374 0.172 1.421)
        ))

(setq *cable-front-to-grasp-pose*
    (make-cascoords
        :pos #f(-21.262 11.534 -8.893)
        :rpy #f(-1.401 0.177 1.409)
        ))

(setq *cable-frame* "cable-frame")



;;; parameters for servo
(setq *servo-to-servo-waiting-pose*
      (make-cascoords
       :pos #f(12.503 -38.046 48.706)
       :rpy #f(2.517 1.299 -2.2)
       ))
