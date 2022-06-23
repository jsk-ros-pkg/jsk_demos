(load "package://jsk_2021_10_soup_from_boil/euslisp/model/my-eng2-coords.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/curry-dish.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/ladle.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/util/interaction.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/util/move-to-kitchen-with-map.l")

(defun arrangement-init ()
  ;; set objects
  (setq *curry-dish* (curry-dish))
  (send *curry-dish* :newcoords (send *curry-dish-coords* :copy-worldcoords))
  (setq *ladle* (ladle))
  (send *ladle* :newcoords (send *ladle-coords-at-arrange* :copy-worldcoords))
  ;; def global coords
  (move-on-irt :arrange)
  (objects (list *pr2* *scene* *ladle* *curry-dish*))
  (update-view)
  )

(arrangement-init)

(defun look-at-dish ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :head :look-at (send (send *curry-dish* :copy-worldcoords) :worldpos))
  (update-view)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun look-at-pot ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :head :look-at (send (send (send *pot* :attention-attention0) :copy-worldcoords) :worldpos))
  (update-view)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

;; curry codes
(defun scoop-up-curry ()
  (let ((ik-target (make-coords :pos (float-vector 471 8170 31070) :rpy (float-vector -1.603 0.0 3.14)))) ;; wait-pose
  ;; Pre-pose for IK

    ;; Add interpolation poses not to contact ladle with kitchen
    (send *ri* :angle-vector-sequence
          (list
           (progn
             (send *pr2* :rarm :angle-vector #f(-86.3553 -16.7247 -80.025 -90.9527 266.392 -20.906 -164.124))
             (update-view)
             (send *pr2* :angle-vector))
           (progn
             (send *pr2* :rarm :angle-vector
                   #f(-44.8146 -12.8411 -56.6328 -47.8682 343.944 -86.0331 -146.852))
             (update-view)
             (send *pr2* :angle-vector))
           (progn
             (send *pr2* :rarm :angle-vector
                   #f(-75.2863 3.32114 -91.388 -82.7469 226.169 -11.2871 -143.423))
             (send *pr2* :rarm :inverse-kinematics
                   (send ik-target :copy-worldcoords)
                   :move-target (car (send *ladle* :attention))
                   :rotation-axis :z
                   :debug-view *debug-flag*)
             (update-view)
             (send *pr2* :angle-vector)))
          (list 1500 1500 1500))
    (send *ri* :wait-interpolation)

    (setq ik-target (make-coords :pos (float-vector 426 8039 30834) :rpy (float-vector -0.74 0.613 -1.132)))
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :move-target (elt (send *ladle* :attention) 1)
          :rotation-axis :z
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation)

    (send ik-target :translate #f(0 0 -10) :world)
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :move-target (elt (send *ladle* :attention) 1)
          :rotation-axis :z
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)

    (send ik-target :newcoords (make-coords :pos (float-vector 479 8064 30870) :rpy (float-vector -1.296 0.242 -0.152)))
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :move-target (elt (send *ladle* :attention) 1)
          :rotation-axis :z
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)

    (send ik-target :translate #f(30 0 80) :world)
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :move-target (elt (send *ladle* :attention) 1)
          :rotation-axis :z
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation))

  (warn "please run (carry-curry-set) ~%")
  )

(defun carry-curry-set ()
  (let ((ik-target (make-coords :pos (float-vector 489.77 8042.025 31004.991) :rpy (float-vector -1.621 0.06 0.033))))
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :move-target (elt (send *ladle* :attention) 1)
          :rotation-axis :z
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation))

  (warn "please run (carry-curry) ~%")
  )

(defun carry-curry ()
  (dotimes (i 2)
    (let ((ik-target (send (elt (send *ladle* :attention) 1) :copy-worldcoords)))
      (send ik-target :translate #f(0 100 0) :world)
      (send *pr2* :rarm :inverse-kinematics
            (send ik-target :copy-worldcoords)
            :move-target (elt (send *ladle* :attention) 1)
            :rotation-axis :z
            :debug-view *debug-flag*)
      (update-view)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
      (send *ri* :wait-interpolation)
      ))
  (warn "please run (transfer-curry-set) ~%")
  )

(defun transfer-curry-set ()
  (send *pr2* :rarm :angle-vector #f(-16.7496 -20.1768 -97.9848 -24.3933 312.645 -99.862 17.5958))
  ;; このik-targetのcoordsに、コップの位置をセットする。
  (let ((ik-target (make-coords :pos (float-vector 554.422 8493.425 30917.724) :rpy (float-vector -1.367 0.118 -0.067))))
    (send *pr2* :rarm :inverse-kinematics
          (send ik-target :copy-worldcoords)
          :move-target (elt (send *ladle* :attention) 1)
          :rotation-axis :z
          :debug-view *debug-flag*)
    (update-view)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation))
  (warn "please run (transfer-curry) ~%")
  )

(defun transfer-curry ()
  (let ((ik-target (send (elt (send *ladle* :attention) 1) :copy-worldcoords)))
    (dotimes (i 9)
      (send ik-target :rotate (deg2rad 10) :y :world)
      (send *pr2* :rarm :inverse-kinematics
            (send ik-target :copy-worldcoords)
            :move-target (elt (send *ladle* :attention) 1)
            :rotation-axis t
            :debug-view *debug-flag*)
      (update-view)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
    ))
  (warn "please run (after-transfer-curry) ~%")
  )

(defun after-transfer-curry ()
  (send *pr2* :rarm :move-end-pos #f(0 0 100) :world :rotation-axis :z)
  (update-view)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  ;; おたまの先端についている水滴を落とす動作
  (let (av avs tms)
    (dotimes (i 3)
      ;; IKが失敗することがある。ladleの先端に関して:rotation-axis :zでIkを解くと良いかも。
      ;; #f(0 0 -40)移動させるIKに失敗したら、何もしない
      (when (send *pr2* :rarm :move-end-pos #f(0 0 -40) :world)
        (update-view)
        (setq av (send *pr2* :angle-vector))
        (setq avs (append avs (list av)))
        (setq tms (append tms (list 400)))
        (send *pr2* :rarm :move-end-pos #f(0 0 40) :world)
        (update-view)
        (setq av (send *pr2* :angle-vector))
        (setq avs (append avs (list av)))
        (setq tms (append tms (list 400)))))
    (send *ri* :angle-vector-sequence avs tms)
    (send *ri* :wait-interpolation))

  ;; avoid-pose的な姿勢になる．
  (send *pr2* :rarm :angle-vector #f(-73.4895 9.68013 -86.5224 -111.491 188.871 -13.522 68.2161))
  (update-view)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (update-view)
  )

(defun arrange-curry ()
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

(defun set-plate-with-dialogue ()
  (transfer-curry-set)
  (transfer-curry)
  (have-the-plate-set-with-dialgue)
  (after-transfer-curry)
  )