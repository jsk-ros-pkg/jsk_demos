#!/usr/bin/env roseus

(defvar *base_to_body_front*)
(defvar *base_to_body_back*)
(defvar *base_to_rarm_elbow_p_servo*)
(defvar *target-id*)
(defvar *target-id-list*)

(defun marker-name-to-servo-id (marker-name)
  (setq marker-name (subseq marker-name 5 (length marker-name)))
  (if (string= marker-name "rarm_elbow_p_servo")
      (return-from marker-name-to-servo-id 2)))

(defun servo-name-to-cable-id-list (servo-name)
  (setq servo-name (subseq servo-name 5 (length servo-name)))
  (if (string= servo-name "rarm_elbow_p_servo")
      (return-from servo-name-to-cable-id-list (list 8 9 10 11))))

(setq *null-wrench* #f(0 0 0 0 0 0))



;;; parameters
(setq *body-back-to-larm-grasp-pose*
      (make-cascoords
       :pos #f(-124.991 17 -55)
       :rpy #f(2.171 1.536 2.122)
       ))
(setq *body-back-to-rarm-grasp-pose*
      (make-cascoords
       :pos #f(124.456 17 -55)
       :rpy #f(2.576 1.517 -0.646)
       ))
(setq *servo-to-search-cable-pose*
      (make-cascoords
       :pos #f(-3.797 -98.159 34.853)
       :rpy #f(1.549 0.916 3.105)
       ))
(setq *servo-to-search-cable-pose-1*
      (make-cascoords
        :pos #f(-2.566 -82.76 38.397)
        :rpy #f(0.188 1.494 1.785)
        ))
(setq *servo-to-search-cable-pose-2*
      (make-cascoords
        :pos #f(-17.376 -144.684 -6.541)
        :rpy #f(-1.561 0.409 3.087)
        )) 
(setq *servo-to-search-cable-pose-3*
      (make-cascoords
        :pos #f(13.83 -55.579 91.701)
        :rpy #f(-1.799 1.134 -0.262)
        ))
(setq *servo-to-cable-front-grasp-ready*
      (make-cascoords
        :pos #f(34.974 -76.339 49.308)
        :rpy #f(1.453 0.441 1.446)
        ))
(setq *cable-front-to-pre-grasp-pose-1*
      (make-cascoords
        :pos #f(-45.691 22.526 27.675)
        :rpy #f(-1.655 0.5 1.413)
        )) 
(setq *cable-front-to-pre-grasp-pose-2*
      (make-cascoords
        :pos #f(-39.479 16.61 12.343)
        :rpy #f(-1.365 0.5 1.57)
        ))
(setq *cable-front-to-grasp-pose*
      (make-cascoords
        :pos #f(-16.344 21.751 10.114)
        :rpy #f(-1.45 0.5 1.552)
        ))
(setq *cable-front-change-pose-before-insert*
      (make-cascoords
        :pos #f(0 0 0)
        :rpy #f(0 1.57 0)
        ))
(setq *cable-front-change-pose-before-insert-1*
      (make-cascoords
        :pos #f(17.295 -52.435 39.996)
        :rpy #f(1.279 0.372 1.172)
        ))
(setq *cable-front-change-pose-before-insert-2*
      (make-cascoords
        :pos #f(17.353 -43.806 45.33)
        :rpy #f(1.294 0.654 1.195)
        ))
(setq *cable-front-change-pose-before-insert-3*
      (make-cascoords
        :pos #f(22.596 -32.874 50.651)
        :rpy #f(1.347 0.974 1.204)
        ))
(setq *servo-to-slide-insert-right-cable-inter-pose*
      (make-cascoords
        :pos #f(34.757 -12.996 40.748)
        :rpy #f(-2.515 1.479 -2.446)
        ))
(setq *servo-to-slide-insert-right-cable-pose*
      (make-cascoords
        :pos #f(28 -13.281 35.626)
        :rpy #f(-1.57 1.57 -1.57)
        ))

(setq *servo-to-lower-insert-right-cable-pre-pose*
      (make-cascoords
        :pos #f(0.313 -29.084 14.451)
        :rpy #f(0.076 0.952 2.952)
        ))
(setq *servo-to-lower-insert-right-cable-pose*
      (make-cascoords
        :pos #f(3.789 -31.211 10.625)
        :rpy #f(0.111 0.946 2.953)
        ))
(setq *servo-to-search-tag-pose-1*
      (make-cascoords
       :pos #f(1.133 -85.27 61.446)
       :rpy #f(1.581 1.228 3.132)
       ))
(setq *servo-to-search-tag-pose-2*
      (make-cascoords
       :pos #f(1.898 -42.878 78.671)
       :rpy #f(-1.931 1.548 -0.355)
       ))
(setq *servo-to-search-tag-pose-3*
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
