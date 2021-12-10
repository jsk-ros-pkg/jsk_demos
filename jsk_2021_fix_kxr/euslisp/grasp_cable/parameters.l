#!/usr/bin/env roseus

(defvar *base-to-rarm-elbow-p-servo*)



;;; parameters
(setq *rarm-elbow-p-to-search-cable-pose*
    (make-cascoords
        :pos #f(-3.797 -98.159 34.853)
        :rpy #f(1.549 0.916 3.105)
        ))
(setq *rarm-elbow-p-to-search-tag-pose-1*
    (make-cascoords
        :pos #f(1.133 -85.27 61.446)
        :rpy #f(1.581 1.228 3.132)
        ))
(setq *rarm-elbow-p-to-search-tag-pose-2*
    (make-cascoords
        :pos #f(1.898 -42.878 78.671)
        :rpy #f(-1.931 1.548 -0.355)
        ))
(setq *rarm-elbow-p-to-search-tag-pose-3*
    (make-cascoords
        :pos #f(2.51 -45.813 49.055)
        :rpy #f(-1.675 1.106 -0.081)
        ))

(setq *rarm-elbow-p-to-inter1-pose*
    (make-cascoords
        :pos #f(-168.038 -3.379 89.959)
        :rpy #f(0.042 -0.008 1.539)
        ))
(setq *rarm-elbow-p-to-inter2-pose*
    (make-cascoords
        :pos #f(-141.254 -5.393 -96.748)
        :rpy #f(0.024 -0.937 -0.027)
        ))
(setq *rarm-elbow-p-to-pre-grasp-pose*
    (make-cascoords
        :pos #f(-28.257 -2.82 -95.35)
        :rpy #f(-2.973 -1.515 2.986)
        ))
(setq *rarm-elbow-p-to-grasp-pose*
    (make-cascoords
        :pos #f(-24.612 4.967 -27.886)
        :rpy #f(-2.962 -1.494 3.004)
        ))

(setq *cable-frame* "cable-frame")



;;; parameters for servo
(setq *servo-to-servo-waiting-pose*
      (make-cascoords
       :pos #f(12.503 -38.046 48.706)
       :rpy #f(2.517 1.299 -2.2)
       ))
