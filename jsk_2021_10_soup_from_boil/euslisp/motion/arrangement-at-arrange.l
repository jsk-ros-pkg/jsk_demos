(load "package://jsk_2021_10_soup_from_boil/euslisp/model/curry-dish.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/ladle.l")

(defun arrangement-init ()
  ;; set objects
  (setq *curry-dish* (curry-dish))
  (send *curry-dish* :newcoords (send *curry-dish-coords* :copy-worldcoords))
  (setq *ladle* (ladle))
  (send *ladle* :newcoords (send *ladle-coords-a* :copy-worldcoords))
  ;; def global coords
  (move-to-arrange-irt)
  (objects (list *pr2* *scene* *ladle* *curry-dish*))
  (update-view)
  )

(arrangement-init)

;; util funcs
(defun reset-rarm-arrangement-at-arrangement () ;; TODO
  (setq *rav0* #f(-20.8048 54.2396 -5.42985 -119.734 197.32 -75.8373 -189.059))
  (send *pr2* :rarm :angle-vector *rav0*)

  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (setq *rav2* #f(-30.0303 1.63624 -169.166 -116.783 100.843 -98.3015 -58.1037))
  (send *pr2* :rarm :angle-vector *rav2*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (setq *rav3* #f(-75.0 50.0 -110.0 -110.0 20.0 -10.0 -10.0))
  (send *pr2* :rarm :angle-vector *rav3*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  )

(defun reset-larm-arrangement-at-arrangement () ;; TODO
  (send *arrow* :newcoords (make-coords :pos (float-vector 557 8260 31125) :rpy (float-vector 2.356 0.785 1.571)))
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *arrow* :newcoords (make-coords :pos (float-vector 600.0 8061.0 31167.0) :rpy (float-vector 1.571 1.047 1.571)))
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *arrow* :newcoords (make-coords :pos (float-vector 800.191 8010.896 30866.677) :rpy (float-vector 2.341 -0.222 0.133)))
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )


(defun look-at-dish ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :head :look-at (send (send *curry-dish* :copy-worldcoords) :worldpos))
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun look-at-pot ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :head :look-at (send (send (send *pot* :attention-attention0) :copy-worldcoords) :worldpos))
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun put-obj-larm ()
  ;; (send *arrow* :translate (float-vector 0 0 200) :world) ;; がしてある状態で使う
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :translate (float-vector 0 0 -100) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 -100) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :larm :wait t)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view nil)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )

(defun put-obj-rarm ()
  ;; (send *arrow* :translate (float-vector 0 0 200) :world) ;; がしてある状態で使う
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :translate (float-vector 0 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :rarm :wait t)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view nil)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )

(defun grasp-obj-rarm ()
  (send *ri* :stop-grasp :rarm :wait t)
  
  ;; targetの位置
  (send *arrow* :newcoords (send *target* :copy-worldcoords))
  (send *arrow* :translate (float-vector 0 0 200) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :translate (float-vector 0 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :rarm :wait t)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)  
  )

(defun grasp-obj-larm ()
  (send *ri* :stop-grasp :larm :wait t)
  
  ;; targetの位置
  (send *arrow* :newcoords (send *target* :copy-worldcoords))
  (send *arrow* :translate (float-vector 0 0 200) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :translate (float-vector 0 0 -100) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 -100) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm :wait t)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :larm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)  
  )



;; curry codes
(defun scoop-up-curry-1 ()
  ;; (send *arrow* :newcoords (make-coords :pos (float-vector 391.83 8092.564 30820) :rpy (float-vector -1.135 0.805 -1.056)))
  (send *arrow* :newcoords (make-coords :pos (float-vector 426 8039 30834) :rpy (float-vector -0.74 0.613 -1.132)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (elt (send *ladle* :attention) 1)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  )


(defun scoop-up-curry-2 ()
  ;; (send *arrow* :translate #f(40 60 0) :world)
  (send *arrow* :translate #f(0 0 -10) :world)
  (send *pr2* :rarm :inverse-kinematics  
        (send *arrow* :copy-worldcoords)
        :move-target (elt (send *ladle* :attention) 1)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun scoop-up-curry-3 ()
  ;; (send *arrow* :newcoords (make-coords :pos (float-vector 446.773 8053.785 30870) :rpy (float-vector -1.525 0.066 -0.424)))
  (send *arrow* :newcoords (make-coords :pos (float-vector 479 8064 30870) :rpy (float-vector -1.296 0.242 -0.152)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (elt (send *ladle* :attention) 1)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun scoop-up-curry-0 ()
  (send *arrow* :newcoords (make-coords :pos (float-vector 471 8170 31070) :rpy (float-vector -1.603 0.0 3.14))) ;; wait-pose
  ;; Pre-pose for IK
  (send *pr2* :angle-vector
        #f(324.895 27.5033 64.5305 54.0589 -83.4934 -44.0265 -47.123 217.695 -75.2863 3.32114 -91.388 -82.7469 226.169 -11.2871 -143.423 11.7494 47.4173))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (car (send *ladle* :attention))
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  ;; Add interpolation poses not to contact ladle with kitchen
  (send *ri* :angle-vector-sequence
        (list
         #f(324.895 27.5033 64.5305 54.0589 -83.4934 -44.0265 -47.123 217.695 -86.3553 -16.7247 -80.025 -90.9527 266.392 -20.906 -164.124 11.7494 47.4173)
         #f(324.895 27.5033 64.5305 54.0589 -83.4934 -44.0265 -47.123 217.695 -44.8146 -12.8411 -56.6328 -47.8682 343.944 -86.0331 -146.852 11.7494 47.4173)
         (send *pr2* :angle-vector))
        (list 1500 1500 1500))
  (send *ri* :wait-interpolation)
  )


(defun scoop-up-curry ()
  (scoop-up-curry-0)
  (scoop-up-curry-1)
  (scoop-up-curry-2)
  (scoop-up-curry-3)

  (send *pr2* :rarm :move-end-pos #f(0 0 50) :world)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (warn "please run (carry-curry-set) ~%")
  )

(defun carry-curry-set ()
  (send *arrow* :newcoords (make-coords :pos (float-vector 489.77 8042.025 31004.991) :rpy (float-vector -1.621 0.06 0.033)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (elt (send *ladle* :attention) 1)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (warn "please run (carry-curry) ~%")
  )


(defun carry-curry ()
  (dotimes (i 2)
    (send *pr2* :rarm :move-end-pos #f(0 100 0) :world)
    (send *viewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    )
  (warn "please run (transfer-curry-set) ~%")
  )

(defun transfer-curry-set ()
  (send *arrow* :newcoords (make-coords :pos (float-vector 504.422 8343.425 30917.724) :rpy (float-vector -1.367 0.118 -0.067)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :move-target (elt (send *ladle* :attention) 1)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (warn "please run (transfer-curry) ~%")
  )

(defun transfer-curry ()
  (send *arrow* :newcoords (send (elt (send *ladle* :attention) 1) :copy-worldcoords))

  (dotimes (i 9)
    (send *arrow* :rotate (deg2rad 10) :y :world)
    (send *pr2* :rarm :inverse-kinematics
          (send *arrow* :copy-worldcoords)
          :move-target (elt (send *ladle* :attention) 1)
          :rotation-axis t
          :debug-view *debug-flag*)
    (send *viewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    )
  (warn "please run (after-transfer-curry) ~%")
  )

(defun after-transfer-curry ()
  (send *pr2* :rarm :move-end-pos #f(0 0 100) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (setq avs (list))
  (setq tms (list))

  (dotimes (i 3)
    (send *pr2* :rarm :move-end-pos #f(0 0 -40) :world)
    (setq av (send *pr2* :angle-vector))
    (setq avs (append avs (list av)))
    (setq tms (append tms (list 400)))

    (send *pr2* :rarm :move-end-pos #f(0 0 40) :world)
    (setq av (send *pr2* :angle-vector))
    (setq avs (append avs (list av)))
    (setq tms (append tms (list 400)))
    )

  (send *ri* :angle-vector-sequence avs tms)
  (send *ri* :wait-interpolation)

  ;; avoid-pose的な姿勢になる．
  (send *arrow* :newcoords (make-coords :pos (float-vector 628.029 8528.539 31050.217) :rpy (float-vector -3.14 0.18 3.056)))
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view *debug-flag*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  )

(defun now-arrange-curry ()
  (set-ladle)
  (look-at-pot)
  (scoop-up-curry)
  (carry-curry-set)
  (carry-curry)

  (look-at-dish)

  (transfer-curry-set)
  (transfer-curry)
  (after-transfer-curry)
  )
