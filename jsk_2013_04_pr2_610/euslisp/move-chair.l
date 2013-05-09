(defun move-chair ()
  (warn "move-chair (inagaki)~%")
  (let (dest-c)

    ;;reset from furuta's pose

    (send *ri* :stop-grasp :rarm :wait t)
    (send *ri* :wait-interpolation)

    (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate #f(0 -300 0) :world)
          :revert-if-fail nil
          :link-list (list
                      (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
          :move-target (list (send *pr2* :rarm :end-coords))
          :look-at-target t
          ;;                :debug-view t
          )
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    ;; use handle to manipulate
    ;;(send *pr2* :reset-pose) ;; to solve ik, do not  send to *ri*
#|    (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate #f(0 0 500) :world)
          :revert-if-fail nil
          :link-list (list
                      (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
          :move-target (list (send *pr2* :rarm :end-coords))
          :look-at-target t
          ;;                :debug-view t
          )
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation) |#
    (send *ri* :stop-grasp :rarm)
;    (send *ri* :wait-interpolation)

    (send *ri* :go-pos-unsafe -0.1 0 0)
;    (send *pr2* :translate (send *pr2* :transform-vector #(-100 0 0)) :world)
    (send *pr2* :translate #f(-100 0 0))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

    ;;  (send *ri* :go-pos-unsafe -0.3 0.05 0)
    ;;  (send *pr2* :translate #f(-300 50 0))
    (send *ri* :wait-interpolation)


    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    ;;(send *pr2* :reset-pose) ;; to solve ik, do not  send to *ri*
    (send *pr2* :inverse-kinematics (list (send (send (send *chair* :handle-above-rarm) :translate #f(-20 0 15) :wolrd) :rotate (deg2rad 20) :y :world))
          :link-list (list
                      (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
          :move-target (list (send *pr2* :rarm :end-coords))
          :revert-if-fail nil
          :look-at-target t
          ;;		:debug-view t
          )
    
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    (send *pr2* :rarm :move-end-pos #f(0 0 -30) :world)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    (send (send *pr2* :r_wrist_roll_link) :assoc *chair*)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    (send *ri* :start-grasp :rarm)
    (send *ri* :wait-interpolation)

    (send *ri* :go-pos-unsafe -0.1 0 0)
    (send *pr2* :translate #f(-100 0 0))
    
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *ri* :wait-interpolation)

    ;;furuta
    ;; (setq dest-c (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
    ;; ;;(send dest-c :translate #f(-280 -120 -220) :world)
    ;; (send dest-c :translate #f(-260 -100 -200) :world)
    ;; (send dest-c :rotate (deg2rad -50) :y :world)

    ;; (send *pr2* :inverse-kinematics dest-c
    ;;       :link-list (list
    ;;                   (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
    ;;       :move-target (list (send *pr2* :rarm :end-coords))
    ;;       :look-at-target t
    ;;       ;;                :debug-view t
    ;;       )
    ;;(send *ri* :angle-vector (send *pr2* :angle-vector))


    (setq back-legs-center (make-cascoords :coords (send *chair* :worldcoords)))
    (send back-legs-center :locate #f(-250 0 0))
    (send back-legs-center :assoc *chair*)
    (objects (list *room610* *pr2* (send *chair* :worldcoords) back-legs-center))

    (setq dest-c (make-cascoords :coords (send (send *pr2* :rarm :end-coords) :worldcoords)))
    (send back-legs-center :assoc dest-c)

    (setq angle-vector-list nil)
    (dotimes (i 9)
      (send back-legs-center :rotate (deg2rad -2) :y)
      (send *pr2* :inverse-kinematics dest-c
	    :link-list (list
			(send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
	    :move-target (list (send *pr2* :rarm :end-coords))
	    :look-at-target t
               :debug-view t
	    )
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (push (send *pr2* :angle-vector) angle-vector-list)
      )

    (setq angle-vector-list (reverse angle-vector-list))

    (send *ri* :angle-vector-sequence angle-vector-list (make-sequence 'list (length angle-vector-list) :initial-element 300))
    (send *ri* :wait-interpolation)

    (send (send *pr2* :r_wrist_roll_link) :assoc *chair*)


    (send *ri* :go-pos-unsafe -0.2 -0 10)
    ;;(send *pr2* :rotate (deg2rad -30) :z)
    (send *pr2* :translate #f(-200 -0 0))
    (send *pr2* :rotate (deg2rad 10) :z)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

    (send *ri* :wait-interpolation)

    (unix::sleep 5)

    ;; backword
    (send *ri* :go-pos-unsafe -1.2 -0.3 -40)
    ;;(send *pr2* :rotate (deg2rad -30) :z)
    (send *pr2* :translate #f(-1200 -300 0))
    (send *pr2* :rotate (deg2rad -40) :z)

    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :wait-interpolation)

    ;;furuta
;;     (setq dest-c (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
;; ;;    (send dest-c :translate #f(100 -100 300) :world)
;; ;;    (send dest-c :rotate (deg2rad -45) :y :world)
;;     (send dest-c :translate #f(100 -100 300) )
;;     (send dest-c :rotate (deg2rad -45) :y)



;;     (send *pr2* :inverse-kinematics dest-c
;;           :revert-if-fail nil
;;           :link-list (list
;;                       (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
;;           :move-target (list (send *pr2* :rarm :end-coords))
;;           :look-at-target t
;;           ;;                :debug-view t
;;           )

;;     (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
;;     (send *ri* :angle-vector (send *pr2* :angle-vector))
;;     (send *ri* :wait-interpolation)

    (setq back-legs-center (make-cascoords :coords (send *chair* :worldcoords)))
    (send back-legs-center :locate #f(-250 0 0))
    (send back-legs-center :assoc *chair*)
    (objects (list *room610* *pr2* (send *chair* :worldcoords) back-legs-center))

    (setq dest-c (make-cascoords :coords (send (send *pr2* :rarm :end-coords) :worldcoords)))
    (send back-legs-center :assoc dest-c)

    (setq angle-vector-list nil)
    (dotimes (i 9)
      (send back-legs-center :rotate (deg2rad 2) :y)
      (send *pr2* :inverse-kinematics dest-c
	    :link-list (list
			(send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
	    :move-target (list (send *pr2* :rarm :end-coords))
	    :look-at-target t
	    ;;                :debug-view t
	    )
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (push (send *pr2* :angle-vector) angle-vector-list)
      )

    (setq angle-vector-list (reverse angle-vector-list))

    (send *ri* :angle-vector-sequence angle-vector-list (make-sequence 'list (length angle-vector-list) :initial-element 300))
    (send *ri* :wait-interpolation)


    ;; ;;push and back
    ;; (send *ri* :go-pos-unsafe 0.2 0 0)
    ;; (send *pr2* :translate #f(200 0 0))

    ;; (send *ri* :go-pos-unsafe -0.3 0 0)
    ;; (send *pr2* :translate #f(-300 0 0))

    
    (send (send *pr2* :r_wrist_roll_link) :dissoc *chair*)
    (send *ri* :stop-grasp :rarm)
    (send *ri* :wait-interpolation)

    (send *pr2* :rarm :move-end-pos #f(-100 0 0))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)


    (send *pr2* :rarm :move-end-pos #f(0 200 0))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    ;; (setq rotate-origin (make-cascoords :coords (send (send *pr2* :r_wrist_roll_link) :copy-worldcoords)))
    ;; (send rotate-origin :assoc *chair*)
    ;; (send rotate-origin :rotate pi/2 :z)
    ;; (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

    ))

