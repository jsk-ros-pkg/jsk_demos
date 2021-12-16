#!/usr/bin/env roseus

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
(setq *servo-to-search-cable-pose*
      (make-cascoords
       :pos #f(-3.797 -98.159 34.853)
       :rpy #f(1.549 0.916 3.105)
       ))
(setq *servo-to-search-cable-pose-1*
      (make-cascoords
        :pos #f(-1.162 -91.94 18.84)
        :rpy #f(0.205 1.494 1.799)
        ))
(setq *servo-to-search-cable-pose-2*
      (make-cascoords
        :pos #f(-17.376 -144.684 -6.541)
        :rpy #f(-1.561 0.409 3.087)
        )) 
(setq *servo-to-search-cable-pose-3*
      (make-cascoords
        :pos #f(44.827 -95.802 12.263)
        :rpy #f(2.19 0.856 3.056)
        ))
(setq *cable-front-to-pre-grasp-pose-1*
      (make-cascoords
        :pos #f(-57.691 22.526 17.675)
        :rpy #f(-1.655 0.272 1.413)
        )) 
(setq *cable-front-to-pre-grasp-pose-2*
      (make-cascoords
        :pos #f(-39.479 16.61 12.343)
        :rpy #f(-1.365 0.367 1.57)
        ))
(setq *cable-front-to-grasp-pose*
      (make-cascoords
        :pos #f(-16.344 21.751 16.114)
        :rpy #f(-1.45 0.449 1.552)
        ))
(setq *servo-to-slide-insert-right-cable-inter-pose*
      (make-cascoords
        :pos #f(34.757 -12.996 42.748)
        :rpy #f(-2.515 1.479 -2.446)
        ))
(setq *servo-to-slide-insert-right-cable-pose*
      (make-cascoords
        :pos #f(33.3 -13.281 37.626)
        :rpy #f(-1.774 1.482 -1.652)
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
