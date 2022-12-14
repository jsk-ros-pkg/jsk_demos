 (load "package://spoteus/spot-interface.l")
 (spot-init)
 (objects (list *spot*))

(send *spot* :reset-pose)
(send *ri* :angle-vector (send *spot* :angle-vector) 5000)
(send *ri* :wait-interpolation)

;start position ver.1
;(send *spot* :angle-vector #f(0.579579 51.0722 -90.3575 -0.196024 51.0818 -90.3335 0.430053 51.2779 -90.7333 -0.059803 51.1679 -90.4999 -7.3573 -141.649 7.83607 8.55783 -38.5174 -8.67582 -1.36665))
;(send *ri* :angle-vector (send *spot* :angle-vector) 5000)
;(send *ri* :wait-interpolation)

;start position ver.2
;; (send *spot* :angle-vector #f(0.900186 50.8197 -90.3914 -0.238533 50.9587 -90.6933 0.584529 51.1155 -90.9841 0.020179 50.707 -90.6455 0.534681 -143.785 18.4731 -0.344761 -36.7685 96.8463 -0.311272))
;; (send *ri* :angle-vector (send *spot* :angle-vector) 5000)
;; (send *ri* :wait-interpolation)

;start position ver.3
(send *spot* :angle-vector #f(-0.19416 47.9212 -91.3344 0.221026 53.7194 -91.2161 -3.88364 54.1232 -93.1145 3.72919 48.8215 -92.7384 -0.483659 -131.043 7.24327 -3.75731 -48.426 97.3056 -0.355218))
(send *ri* :angle-vector (send *spot* :angle-vector) 5000)
(send *ri* :wait-interpolation)


;grab fishing rod
(send *ri* :gripper-open)
(send *ri* :wait-interpolation)
(unix:sleep 5)
(send *ri* :gripper-close)
(send *ri* :wait-interpolation)


;end position ver.1
;; (send *spot* :angle-vector #f(-0.001642 42.3328 -90.2934 -0.746104 41.8198 -89.2853 -0.17725 42.2826 -90.1418 -0.69892 41.8884 -89.4534 -8.08705 -59.1052 13.8305 8.54947 -38.486 -8.63814 -0.376507))
;; (send *ri* :angle-vector (send *spot* :angle-vector) 500)
;; (send *ri* :wait-interpolation)

;end position ver.2
;; (send *spot* :angle-vector #f(0.8435 42.1846 -90.1341 -0.297538 42.1008 -90.3042 0.444329 42.1165 -90.3753 -0.015944 41.9824 -90.4953 4.08791 -61.5594 21.981 66.0059 -1.56093 24.9492 -89.512))
;; (send *ri* :angle-vector (send *spot* :angle-vector) 1000)
;; (send *ri* :wait-interpolation)

;end position ver.3
(send *spot* :angle-vector #f(-0.219186 43.0658 -90.6477 0.221758 49.2979 -91.2719 -3.85436 49.5271 -92.9653 3.72034 43.9016 -91.9354 -7.38435 -97.7657 30.7545 10.1184 -52.493 89.5932 -0.356591))
(send *ri* :angle-vector (send *spot* :angle-vector) 1000)
(send *ri* :wait-interpolation)

(dotimes (i 5)
  (setq hand-coord-copy (send (send *spot* :arm :end-coords) :copy-worldcoords))
  (send hand-coord-copy :translate (float-vector 0 (* 50 (expt -1 i)) 0) :world)
  (send *spot* :arm :inverse-kinematics hand-coord-copy :rotation-axis :z)
  
  (setq hand-coord-copy (send (send *spot* :arm :end-coords) :copy-worldcoords))
  (send hand-coord-copy :translate (float-vector 0 (* 50 (expt -1 i)) 0) :world)
  (send *spot* :arm :inverse-kinematics hand-coord-copy :rotation-axis :z)

  (setq hand-coord-copy (send (send *spot* :arm :end-coords) :copy-worldcoords))
  (send hand-coord-copy :translate (float-vector 0 (* 50 (expt -1 i)) 0) :world)
  (send *spot* :arm :inverse-kinematics hand-coord-copy :rotation-axis :z)

  (setq hand-coord-copy (send (send *spot* :arm :end-coords) :copy-worldcoords))
  (send hand-coord-copy :translate (float-vector 0 (* 50 (expt -1 i)) 0) :world)
  (send *spot* :arm :inverse-kinematics hand-coord-copy :rotation-axis :z)

  (setq hand-coord-copy (send (send *spot* :arm :end-coords) :copy-worldcoords))
  (send hand-coord-copy :translate (float-vector 0 (* 50 (expt -1 i)) 0) :world)
  (send *spot* :arm :inverse-kinematics hand-coord-copy :rotation-axis :z)
  
  (setq hand-coord-copy (send (send *spot* :arm :end-coords) :copy-worldcoords))
  (send hand-coord-copy :translate (float-vector 0 (* 50 (expt -1 i)) 0) :world)
  (send *spot* :arm :inverse-kinematics hand-coord-copy :rotation-axis :z)
  
  ;; (send *irtviewer* :draw-objects)
  ;; (x::window-main-one)

  (send *ri* :angle-vector (send *spot* :angle-vector) 500) 
  (send *ri* :wait-interpolation)
  )


