;; place-broom.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(defun place-broom-laundry ()
  (warn "place-broom-laundry (furushchev)~%")
  (let ((place-broom-spot (make-cascoords :pos #f(3269.927 1251.466 0) :rpy (list pi/2 0 0))))
    ;; (move-to-broom)
    (send *ri* :go-pos-unsafe -0.2 0 0)
    (send *pr2* :translate #f(-200 0 0))

    (send *pr2* :move-to place-broom-spot :world)
    (if (not (send *ri* :move-to place-broom-spot :frame-id *room610-origin*))
        (return-from place-broom-laundry nil)
      )
    (send *ri* :go-pos-unsafe -0.2 0 0)
    (send *pr2* :translate #f(-200 0 0))

    (send *ri* :stop-grasp :rarm :wait t)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :inverse-kinematics
          ;600 -410 1500
          (send (make-coords :pos (float-vector 650 -560 1500)
                             :rpy (list pi (deg2rad 4) 0))
                :transform *laundry* :world)
          :look-at-target t
          :rotation-axis t
          :revert-if-fail nil
          :debug-view nil)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (move-to-broom)
    (send *ri* :stop-grasp :larm :wait t)
    ;; for visualization
    (let ((sweeper (send *ri* :find-object "room610-sweeper")))
      (when sweeper
        (setq sweeper (car sweeper))
        (send sweeper :translate (float-vector 0 0
                                           (- (elt (send sweeper :worldpos) 2))))
    (send *ri* :wait-interpolation)
    (send *ri* :go-pos-unsafe -0.1 0 0)
    (pr2-reset-pose)
    )
  )

(provide :place-broom) ;; end of place-broom.l
