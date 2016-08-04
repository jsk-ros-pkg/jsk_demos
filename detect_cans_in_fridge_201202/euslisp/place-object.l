(require :find-object "find-object.l")

(setq *pre-place-pose* (float-vector 20.3294 11.2244 -18.1145 72.5353 -114.441 -192.919 -106.159 -82.312 -4.53687 73.6792 -109.653 -86.239 79.6664 -7.00982 19.9594 0.006701 40.608))

(defun place-object-to-table (&key (max-retry 3)
                                   (arm :larm)
                                   (place-forward 250)
                                   (wait-hand-impact-timeout 10))
  (send *pr2* :head :neck-p :joint-angle 40)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)

  (setq *table-box* (find-fcn-object "table"))

  (send *pr2* :angle-vector *pre-place-pose*)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)

  (let* ((table-top (subseq (sort (send *table-box* :vertices)
                                  #'(lambda (v1 v2)
                                      (>= (aref v1 2) (aref v2 2))))
                            0 4))
         (table-front (subseq (sort table-top
                                    #'(lambda (v1 v2)
                                        (<= (aref v1 0) (aref v2 0))))
                              0 2))
         (cnt 0)
         place-pos (move-back 0.0))
    (print table-top)
    (setq *table-top* table-top)
    (setq place-pos (make-coords
                     :pos (float-vector (+ (aref (car table-front) 0) place-forward)
                                        0.0
                                        (+ (aref (car table-top) 2) 50))))
    (send place-pos :rotate pi :x)
    (setq *place-pos* place-pos)
    (while (and (ros::ok) (< cnt max-retry))
      (if (send *pr2* arm :inverse-kinematics
                place-pos
                :look-at-target t
                :rotation-axis :z)
          (let ()
            (send *ri* :angle-vector (send *pr2* :angle-vector))
            (send *ri* :wait-interpolation)
            (send *ri* :stop-grasp arm :wait t)
            
            (send *pr2* :angle-vector *pre-place-pose*)
            (send *ri* :angle-vector (send *pr2* :angle-vector))
            (send *ri* :wait-interpolation)
            (when (> move-back 0.0)
              (send *ri* :go-pos-unsafe (- move-back) 0 0))
            (pr2-tuckarm-pose)
            (return-from place-object-to-table t))
        (progn
          (inc cnt)
          (send *ri* :go-pos-unsafe 0.1 0 0)
          (setq move-back (+ move-back 0.1))
          (send *pr2* :translate (float-vector 100 0 0)))))
    (speak-jp "テーブルに近づけないのでとってください")
    (wait-for-hand-impact arm :timeout wait-hand-impact-timeout)
    (send *ri* :move-gripper arm 0.08 :wait t)
    (return-from place-object-to-table t)))
