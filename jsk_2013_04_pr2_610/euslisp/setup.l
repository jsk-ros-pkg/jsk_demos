#!/usr/bin/env roseus

(load "models/room610-scene.l")
(load "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")
(load "models/room73b2-broom-object.l")
(load "models/room610-scene.l")
(load "iwaishi-pick-croth.l")
(load "y-tanaka-pick-tray.l")
(load "wesleyc-place-tray.l")
(load "sweep-under-table.l")

(setq debug nil)

(room610)
(pr2)
(if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init :objects (send *room610* :objects))))
(if (not (boundp '*tfl*)) (setq *tfl* (instance transform-listener :init)))

(objects (list *room610* *pr2*))

(setq *room610-origin* "/eng8/6f/610")
(setq *table-coords* (make-coords :pos #f(4700 1000 0) :rpy (float-vector -pi/2 0 0)))
(setq *chair-coords* (make-coords :pos #f(3800 0 0)))

(setq *tray* (send *room610* :object "room610-tray"))
(send *tray* :locate #f(5050.0 0.0 700.0) :world)
(setq *broom* (send *room610* :object "room610-sweeper"))
(send *broom* :locate #f(3000.0 2090.0 0.0) :world)
;;(send *broom* :translate #f(0 100 0) :world)
(setq *chair* (send *room610* :object "room610-chair"))
(send *chair* :locate #f(4700.0 -360 0.0) :world)
(setq *laundry* (send *room610* :object "room610-laundry-machine"))
;; yukizaki?
(send *laundry* :translate #f(-120 10 -85) (send *room610* :worldcoords))

;; spots
(setq *tray-spot* (make-cascoords :pos #f(5050 700 0) :rpy (list -pi/2 0 0)))
(send *room610* :assoc *tray-spot*)
(setq *chair-spot* (make-cascoords :pos #f(3750 -250 0) :rpy (list 0 0 0)))
(send *room610* :assoc *chair-spot*)
(setq *laundry-spot* (make-cascoords :pos #f(3176.358 1409.163 -0.854) :rpy #f(1.544 0.004 0.002)))
(send *room610* :assoc *laundry-spot*)
(setq *pick-broom-spot* (make-cascoords :pos #f(3269.927 1351.466 0) :rpy (list pi/2 0 0)))
(send *room610* :assoc *pick-broom-spot*)
(setq *sink-spot* (make-cascoords :pos #f(7300 200 0) :rpy (list 0 0 0)))
(send *room610* :assoc *sink-spot*)
(setq *sweep-spot* (make-cascoords :pos #f(3750 200 0) :rpy (list (deg2rad -30) 0 0)))
(send *room610* :assoc *sweep-spot*)

(defmethod room610-scene
  (:spot-tray-spot () *tray-spot*)
  (:spot-chair-spot () *chair-spot*)
  (:spot-laundry-spot () *laundry-spot*)
  (:spot-pick-broom-spot () *pick-broom-spot*)
  (:spot-sink-spot () *sink-spot*)
  (:spot-sweep-spot () *sweep-spot*)
  (:spots ()
          (list *tray-spot* *chair-spot* *laundry-spot* *pick-broom-spot* *sink-spot* *sweep-spot*)))

(load "package://jsk_2013_04_pr2_610/euslisp/pick-broom.l")
(load "package://jsk_2013_04_pr2_610/euslisp/put-cloth-into-laundry.l")

(setq *world-to-610* (send *tfl* :lookup-transform "/world" *room610-origin* (ros::time 0)))

(defun move-to-table ()
  (warn "move-to-table~%")
  (pr2-reset-pose)
  (pr2-tuckarm-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *pr2* :move-to (send *room610* :spot-tray-spot) :world)
  (send *ri* :move-to (send *room610* :spot-tray-spot) :frame-id *room610-origin*)
  )

(defun pick-tray ()
  (warn "pick-tray (tanaka)~%")
  (y-tanaka-pick-tray)
)

(defun move-to-sink ()
  (warn "move-to-sink~%")
  ;;(pr2-tuckarm-pose)
  (send *pr2* :move-to (send *room610* :spot-sink-spot) :world)
  (send *ri* :move-to (send *room610* :spot-sink-spot) :frame-id *room610-origin*)
  )

(defun place-tray ()
  (warn "place-tray (chan)~%")
;;  (wesleyc-place-tray)
  )

(defun move-to-chair ()
  (warn "move-to-chair~%")
  (send *pr2* :move-to (send *room610* :spot-chair-spot) :world)
  (send *ri* :move-to (send *room610* :spot-chair-spot) :frame-id *room610-origin*)
  )

(defun pick-cloth ()
  (warn "pick-cloth ()~%")
  (iwaishi-pick-cloth)
  )

(defun move-to-laund ()
  (setq ima (send *ri* :state :worldcoords "/eng8/6f/610"))
  (setq diff (v- (send *laundry-spot* :pos) (send ima :pos)))
  (send *laundry* :translate diff (send *room610* :worldcoords))
  )

(defun relocate-pr2 ()
  (setq ima (send *ri* :state :worldcoords "/eng8/6f/610"))
  (send *pr2* :move-to ima :world))

(defun mtl ()
  (setq ima (send *ri* :state :worldcoords "/eng8/6f/610"))
  (send *pr2* :move-to ima :world)
)

(defun test ()
  (init)
  (mtl)
  (put-cloth-into-laundry)
)

(defun move-to-laundry ()
  (warn "move-to-laundry~%")
  (send *pr2* :move-to (send *room610* :spot-laundry-spot) :world)
  ;; (send *pr2* :translate #f(0 300 0) (send *room610* :worldcoords))
  (send *ri* :move-to (send *room610* :spot-laundry-spot) :frame-id *room610-origin*)
  (setq current-ri-coords (send (send *world-to-610* :worldcoords) :transformation (send *ri* :state :worldcoords)))
  (setq mou-chotto-pos (v- (send (send *room610* :spot-laundry-spot) :pos) (send current-ri-coords :pos)))
  (setq mou-chotto-local-pos (transform (inverse-matrix (send (send *ri* :state :worldcoords) :worldrot)) mou-chotto-pos))
  (setq mou-chotto-roll
		(- 
		 (elt (elt (send (send *room610* :spot-laundry-spot) :rpy-angle) 0) 0)
		 (elt (elt (send current-ri-coords :rpy-angle) 0) 0)))
  (send *ri* :go-pos-unsafe (/ (elt mou-chotto-local-pos 0) 1000) (/ (elt mou-chotto-local-pos 1) 1000) (rad2deg mou-chotto-roll))
  )

;; (defun open-laundry-door ()
;;   (warn "open-laundry-door (kuroiwa)~%"))

;; (defun put-cloth ()
;;   (warn "put-cloth (kuroiwa)~%")
;;   )

;; (defun close-laundry-door ()
;;   (warn "close-laundry-door (kuroiwa)~%")
;;   )

(defun move-chair ()
  (warn "move-chair (inagaki)~%")
  ;;reset from furuta's pose

  (send *ri* :stop-grasp :rarm)
  (send *ri* :wait-interpolation)

(send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate #f(0 -300 0) :world)
        :revert-if-fail nil
        :link-list (list
                      (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
        :move-target (list (send *pr2* :rarm :end-coords))
        :look-at-target t
;;                :debug-view t
        )
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)





  ;; use handle to manipulate
  ;;(send *pr2* :reset-pose) ;; to solve ik, do not  send to *ri*
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate #f(0 0 500) :world)
	:revert-if-fail nil
        :link-list (list
                      (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
        :move-target (list (send *pr2* :rarm :end-coords))
        :look-at-target t
;;                :debug-view t
        )
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :rarm)
  (send *ri* :wait-interpolation)


  (send *ri* :go-pos-unsafe -0.1 0 0)
  (send *pr2* :translate #f(-100 0 0))
  (send *ri* :wait-interpolation)


  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
;;(send *pr2* :reset-pose) ;; to solve ik, do not  send to *ri*
  (send *pr2* :inverse-kinematics (list (send (send (send *chair* :handle-above-rarm) :translate #f(0 0 -15) :wolrd) :rotate (deg2rad 20) :y :world))
        :link-list (list
                      (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
        :move-target (list (send *pr2* :rarm :end-coords))
	:revert-if-fail nil
        :look-at-target t
;;		:debug-view t
        )
 
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)

  (send (send *pr2* :r_wrist_roll_link) :assoc *chair*)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)


  (send *ri* :start-grasp :rarm)
  (send *ri* :wait-interpolation)


(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(setq destC (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
(send destC :translate #f(-280 -120 -220) :world)
(send destC :rotate (deg2rad -50) :y :world)



(send *pr2* :inverse-kinematics destC
        :link-list (list
                      (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
        :move-target (list (send *pr2* :rarm :end-coords))
        :look-at-target t
;;                :debug-view t
        )
  (send *ri* :wait-interpolation)

  (send *ri* :angle-vector (send *pr2* :angle-vector))

  (send *ri* :wait-interpolation)
  (send *ri* :go-pos-unsafe -0.2 -0 10)
  ;;(send *pr2* :rotate (deg2rad -30) :z)
  (send *pr2* :translate #f(-200 -0 10))


  (send *ri* :wait-interpolation)

 (unix::sleep 5)

  ;; backword
  (send *ri* :go-pos-unsafe -1.2 -0.3 -40)
  ;;(send *pr2* :rotate (deg2rad -30) :z)
  (send *pr2* :translate #f(-1200 -300 -40))
  (send *ri* :wait-interpolation)

(setq destC (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
(send destC :translate #f(100 -100 300) :world)
(send destC :rotate (deg2rad -45) :y :world)



(send *pr2* :inverse-kinematics destC
	:revert-if-fail nil
        :link-list (list
                      (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
        :move-target (list (send *pr2* :rarm :end-coords))
        :look-at-target t
;;                :debug-view t
        )

  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)



  
  (send (send *pr2* :r_wrist_roll_link) :dissoc *chair*)
  (send *ri* :stop-grasp :rarm)
  (send *ri* :wait-interpolation)
  )

(defun move-to-sweep ()
  (warn "move-to-sweep~%")
  (send *pr2* :move-to (send *room610* :spot-sweep-spot) :world)
  (send *ri* :move-to (send *room610* :spot-sweep-spot) :frame-id *room610-origin*)
)

(defun sweep-under-table ()
  (warn "sweep-under-table-yukizaki~%")
  (sweep-under-table-yukizaki)
  )

(defun pass-broom-to-right-hand ()
  (warn "pass-broom-to-right-hand~%"))

(defun sweep-floor ()
  (warn "sweep-floor (furuta)~%")
  (furuta-sweep)
  )


(defun switch-global-planner-observation (enable)
  (unix::system
   (format nil "rosrun topic_tools mux_select /tilt_laser_mux ~s"
           (if enable "ground_object_cloud" "empty_cloud"))))



(defun demo ()
  (if (not debug)
      (progn
        (move-to-table)
        (pick-tray)
        (switch-global-planner-observation nil)
        (move-to-sink)
        (switch-global-planner-observation t)
        (place-tray)
        (move-to-chair)
        (pick-cloth)
        (switch-global-planner-observation nil)
        (move-to-laundry)
        (pick-broom)
        (move-to-chair)
        (move-chair)
        (move-to-sweep)
        (sweep-under-table)
        (sweep-floor)
        (switch-global-planner-observation t)
        ))
  (put-cloth-into-laundry)
  )

(warn "type (demo) to start~%")
