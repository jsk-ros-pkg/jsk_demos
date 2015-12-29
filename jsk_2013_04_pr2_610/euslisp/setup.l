#!/usr/bin/env roseus
(ros::roseus "jsk_irt_demo")
(load "models/room610-scene.l")
(load "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")
(load "models/room73b2-broom-object.l")
(load "models/room610-scene.l")
(load "models/cup-object.l")
(load "package://jsk_2013_04_pr2_610/euslisp/utils.l")
(load "package://jsk_2013_04_pr2_610/euslisp/pick-cloth.l")
(load "package://jsk_2013_04_pr2_610/euslisp/pick-tray.l")
(load "package://jsk_2013_04_pr2_610/euslisp/move-chair.l")
;(load "package://jsk_2013_04_pr2_610/euslisp/move-table.l")
(load "package://jsk_2013_04_pr2_610/euslisp/place-tray.l")
(load "package://jsk_2013_04_pr2_610/euslisp/sweep-under-table.l")
(load "package://jsk_2013_04_pr2_610/euslisp/pick-broom.l")
(load "package://jsk_2013_04_pr2_610/euslisp/put-cloth-into-laundry.l")
;(load "package://jsk_2013_04_pr2_610/euslisp/detect-chair.l")
;(load "package://jsk_2013_04_pr2_610/euslisp/sweep-floor.l")
(load "package://jsk_2013_04_pr2_610/euslisp/pick-up-mug.l")
(load "package://jsk_2013_04_pr2_610/euslisp/handover-mug.l")
(load "package://jsk_2013_04_pr2_610/euslisp/pick-up-remote.l")
(load "package://jsk_2013_04_pr2_610/euslisp/handover-remote.l")
(load "package://jsk_2013_04_pr2_610/euslisp/pick-up-kitchen-knife.l")
(load "package://jsk_2013_04_pr2_610/euslisp/handover-kitchen-knife.l")

(defun setup ()
  (room610)
  (pr2)

  (if (boundp '*irtviewer*) (objects (list *room610* *pr2*)))
  (setq *room610-origin* "/eng8/6f/610")
  (setq *table-coords* (make-coords :pos #f(4700 1000 0) :rpy (float-vector -pi/2 0 0)))
  (setq *chair-coords* (make-coords :pos #f(3800 0 0)))

  (setq *table* (send *room610* :object "room610-table"))
;  (send *table* :locate #f(4700 1000 0) :world)
  (send *table* :locate #f(5000 -400 0) :world)
  (setq *tray* (send *room610* :object "room610-tray"))
  (send *tray* :locate #f(5050.0 0.0 700.0) :world)
  (setq *broom* (send *room610* :object "room610-sweeper"))
  ;;(send *broom* :locate #f(3000.0 2090.0 0.0) :world)
  (send *broom* :locate #f(2830.0 1900.0 0.0) :world)
  ;;(send *broom* :locate #f(2760.0 1900.0 0.0) :world)
  ;;(send *broom* :translate #f(0 100 0) :world)
  (setq *chair* (send *room610* :object "room610-chair"))
  (send *chair* :locate #f(4700.0 -360 0.0) :world)
  (setq *laundry* (send *room610* :object "room610-laundry-machine"))
  ;; yukizaki?
;  (send *laundry* :translate #f(-120 10 -85) (send *room610* :worldcoords))
  (send *laundry* :translate #f(-260 -30 -125) (send *room610* :worldcoords))
  (setq *cabinet* (send *room610* :object "room610-cabinet-mamoru"))
  (send *cabinet* :locate #f(7000 -2200 0) :world)
  ;; spots
  (setq *mug-spot* (make-cascoords :pos #f(5500 900 0) :rpy (list -pi/2 0 0)))
  (send *room610* :assoc *mug-spot*)
  (setq *mug-handover-spot* (make-cascoords :pos #f(6100 600 0) :rpy (list -pi/2 0 0))) ;; where person is sitting
  (send *room610* :assoc *mug-handover-spot*)
  (setq *remote-spot* (make-cascoords :pos #f(7000 900 0) :rpy (list pi/2 0 0))) ;; where person is sitting
  (send *room610* :assoc *remote-spot*)
  (setq *remote-handover-spot* (make-cascoords :pos #f(6100 600 0) :rpy (list -pi/2 0 0))) ;; where person is sitting
  (send *room610* :assoc *remote-handover-spot*)
  (setq *kitchen-knife-spot* (make-cascoords :pos #f(6900 400 0) :rpy (list 0 0 0))) ;; where person is sitting
  (send *room610* :assoc *kitchen-knife-spot*)
  
  (setq *tray-spot* (make-cascoords :pos #f(4900 600 0) :rpy (list -pi/2 0 0)))
  (send *room610* :assoc *tray-spot*)
  (setq *chair-spot* (make-cascoords :pos #f(3650 50 0) :rpy (list 0 0 0)))
  (send *room610* :assoc *chair-spot*)
  (setq *chair-spot2* (make-cascoords :pos #f(3650 50 0) :rpy (list 0 0 0)))
  (send *room610* :assoc *chair-spot2*)
;;  (setq *laundry-spot* (make-cascoords :pos #f(3176.358 1409.163 -0.854) :rpy #f(1.544 0.004 0.002)))
;;  (setq *laundry-spot* (make-cascoords :pos #f(3376.358 1309.163 -0.854) :rpy #f(1.544 0.004 0.002))) ;; move-to bug?
  (setq *laundry-spot* (make-cascoords :pos #f(3326.358 1349.163 -0.854) :rpy #f(1.544 0.004 0.002)))
  (send *room610* :assoc *laundry-spot*)
  (setq *pick-broom-spot* (make-cascoords :pos #f(3269.927 1451.466 0) :rpy (list pi/2 0 0)))
  (send *room610* :assoc *pick-broom-spot*)
  (setq *sink-spot* (make-cascoords :pos #f(7300 200 0) :rpy (list 0 0 0)))
  (send *room610* :assoc *sink-spot*)
  ;;(setq *sweep-spot* (make-cascoords :pos #f(3450 -400 0) :rpy (list (deg2rad -40) 0 0)))
  (setq *sweep-spot* (make-cascoords :pos #f(3750 -400 0) :rpy (list (deg2rad -40) 0 0)))
  (send *room610* :assoc *sweep-spot*)
  (setq *moved-back-chair-spot* (make-cascoords :pos #f(750 0 0) :rpy (list 0 0 0)))
  (send *room610* :assoc *moved-back-chair-spot*)

  ;; need to create objects for handover 
  (setq *mug* (cup))
  (send *mug* :locate #f(5050.0 100.0 800.0) :world)
  (send *mug* :rotate (/ -pi 2) #f(0 0 1) :world)
  (let ((objs (send *room610* :objects)))
    (setf (room610-scene-objs *room610*) (append objs (list *mug*))))
  
  (setq *person-seated* (instance room610-tray-object :init))
  (send *person-seated* :locate #f(6100.0 -600.0 1100.0) :world)
  (send *person-seated* :rotate (/ pi 2) #f(0 0 1) :world)

  (setq *remote* (instance room610-tray-object :init))
  (send *remote* :locate #f(6800.0 1600.0 720.0) :world)
  (send *remote* :rotate (/ pi 2) #f(0 1 0) :world)
  (send *remote* :rotate (/ pi 2) #f(0 0 1) :world)

  (setq *kitchen-knife* (instance room610-tray-object :init))
  (send *kitchen-knife* :locate #f(7500.0 300.0 700.0) :world)
  (send *kitchen-knife* :rotate (/ pi 2) #f(0 1 0) :world)

  (setq *person-at-kitchen* (instance room610-tray-object :init))
  (send *person-at-kitchen* :locate #f(7200.0 -100.0 1500.0) :world)

  
  ;;after move-to, save the pos
  (setq *moved-chair-spot* nil)

  (if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init :objects (send *room610* :objects))))
  (if (not (boundp '*tfl*)) (setq *tfl* (instance transform-listener :init)))

  (setq *world-to-610* (send *tfl* :lookup-transform "/world" *room610-origin* (ros::time 0)))

  (change-inflation-range 0.4) ;;0.1
  (if (not (boundp '*irtviewer*)) (clear-costmap))
  )


(defmethod room610-scene
  (:spot-mug-spot () *mug-spot*)
  (:spot-mug-handover-spot () *mug-handover-spot*)
  (:spot-remote-spot () *remote-spot*)
  (:spot-remote-handover-spot () *remote-handover-spot*)
  (:spot-kitchen-knife-spot () *kitchen-knife-spot*)
  (:spot-tray-spot () *tray-spot*)
  (:spot-chair-spot () *chair-spot*)
  (:spot-chair-spot2 () *chair-spot2*)
  (:spot-laundry-spot () *laundry-spot*)
  (:spot-pick-broom-spot () *pick-broom-spot*)
  (:spot-sink-spot () *sink-spot*)
  (:spot-sweep-spot () *sweep-spot*)
  (:spot-moved-back-chair () *moved-back-chair-spot*)
  (:spots ()
          (list *mug-spot* *mug-handover-spot* *remote-spot* *remote-handover-spot* *kitchen-knife-spot* *tray-spot* *chair-spot* *laundry-spot* *pick-broom-spot* *sink-spot* *sweep-spot* *moved-back-chair-spot*)))

(defun pr2-pick-tray-pose ()
  ;; Open collar-y a little bit
  (send *pr2* :larm :collar-y :joint-angle 75)
  (send *pr2* :rarm :collar-y :joint-angle -75)

  ;; Rise shoulder-p upto horizontal position
  (send *pr2* :arms :shoulder-p :joint-angle -20)

  ;; Rotate shoulder-p
  (send *pr2* :larm :shoulder-r :joint-angle 70)
  (send *pr2* :rarm :shoulder-r :joint-angle -70)

  (send *pr2* :head :neck-p :joint-angle 50)
  )

(defun move-to-table-side ()
  (warn "move-to-table-side~%")
  (use-tilt-laser-obstacle-cloud nil)

  (send *pr2* :reset-pose)
  (pr2-pick-tray-pose)

  ;; send angle-vector
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)

  (clear-costmap)
  (send *pr2* :move-to (send *room610* :spot-tray-spot) :world)
  (if (not (send *ri* :move-to (send *room610* :spot-tray-spot) :frame-id *room610-origin*))
	  (return-from move-to-table-side nil)


    )

  (send *pr2* :head :neck-p :joint-angle 49.386)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  t)

(defun move-to-mug-pick-up-position ()
  (warn "move-to-mug-pick-up-position~%")
  (use-tilt-laser-obstacle-cloud nil)

  (send *pr2* :reset-pose)
  (pr2-pick-tray-pose)

  ;; send angle-vector
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)

  (clear-costmap)
  (send *pr2* :move-to (send *room610* :spot-mug-spot) :world)
  (if (not (send *ri* :move-to (send *room610* :spot-mug-spot) :frame-id *room610-origin*))
	  (return-from move-to-mug-pick-up-position nil)
	  )
  (send *pr2* :head :look-at (send *mug* :copy-pos))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  t)

(defun move-to-mug-handover-position ()
  (warn "move-to-mug-handover-position~%")
  (use-tilt-laser-obstacle-cloud nil)

  ;; send angle-vector
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)

  (clear-costmap)
  (send *pr2* :move-to (send *room610* :spot-mug-handover-spot) :world)
  (if (not (send *ri* :move-to (send *room610* :spot-mug-handover-spot) :frame-id *room610-origin*))
	  (return-from move-to-ug-handover-position nil)
	  )
  (send *pr2* :head :look-at (send *person-seated* :copy-pos))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  t)

(defun move-to-remote-handover-position ()
  (warn "move-to-mug-handover-position~%")
  (use-tilt-laser-obstacle-cloud nil)

  ;; send angle-vector
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)

  (clear-costmap)
  (send *pr2* :move-to (send *room610* :spot-remote-handover-spot) :world)
  (if (not (send *ri* :move-to (send *room610* :spot-remote-handover-spot) :frame-id *room610-origin*))
	  (return-from move-to-ug-handover-position nil)
	  )
  (send *pr2* :head :look-at (send *person-seated* :copy-pos))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  t)


(defun move-to-sink ()
  (warn "move-to-sink~%")
  (use-tilt-laser-obstacle-cloud nil)
  (clear-costmap)
  (send *pr2* :torso  :waist-z :joint-angle 250)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *pr2* :move-to (send *room610* :spot-sink-spot) :world)
  (if (not (send *ri* :move-to (send *room610* :spot-sink-spot) :frame-id *room610-origin*))
	  (return-from move-to-sink nil)
	  )
  (use-tilt-laser-obstacle-cloud t)
  (send *ri* :wait-interpolation)
  t)

(defun move-to-kitchen-knife ()
  (warn "move-to-kitchen-knife~%")

  (send *pr2* :angle-vector #f(50.1262 5.0 70.0 105.0 -90.0 -70.0 -6.0 -20.0 0.0 -10.0 -40.0 -70.0 -70.0 -90.0 -90.0 0.007004 -1.914))
  (send *pr2* :torso :waist-z :joint-angle 50)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
  (send *ri* :stop-grasp :rarm)

  (use-tilt-laser-obstacle-cloud nil)
  (clear-costmap)
  (send *pr2* :torso  :waist-z :joint-angle 250)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *pr2* :move-to (send *room610* :spot-kitchen-knife-spot) :world)
  (if (not (send *ri* :move-to (send *room610* :spot-kitchen-knife-spot) :frame-id *room610-origin*))
	  (return-from move-to-kitchen-knife nil)
	  )
  (use-tilt-laser-obstacle-cloud t)
  (send *ri* :wait-interpolation)
  t)

(defun move-to-chair ()
  (warn "move-to-chair~%")
  (move-to-chair2)
  )

(defun move-to-chair-rarm ()
  (warn "move-to-chair~%")

  ;;kamae
  ;;(larm-tuck)
  (pr2-tuckarm-pose :rarm)
  (send *pr2* :rarm :angle-vector #f(-40.0 -20.0 0.0 -70.0 -70.0 -90.0 0.0))
  (send *pr2* :torso :waist-z :joint-angle 50)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
  (send *ri* :stop-grasp :rarm)

  (use-tilt-laser-obstacle-cloud nil)
  (clear-costmap)
  (send *pr2* :move-to (send *room610* :spot-chair-spot) :world)
  (if (not (send *ri* :move-to (send *room610* :spot-chair-spot) :frame-id *room610-origin*))
	  (return-from move-to-chair nil)
	  )
  (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
  (use-tilt-laser-obstacle-cloud t)
  t)

(defun move-to-chair-larm ()
  (warn "move-to-chair-larm~%")

  ;;kamae
  ;;(larm-tuck)
  ;;(pr2-tuckarm-pose :larm)
  ;;(send *pr2* :larm :angle-vector #f(40.0 -20.0 0.0 -70.0 70.0 -90.0 0.0))
  (send *pr2* :angle-vector #f(50.1262 0.0 -10.0 40.0 -70.0 70.0 -90.0 90.0 -5.0 70.0 -105.0 -90.0 70.0 -6.0 20.0 0.007004 -1.914))
  (send *pr2* :torso :waist-z :joint-angle 50)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
  (send *ri* :stop-grasp :larm)

  (use-tilt-laser-obstacle-cloud nil)
  (clear-costmap)
  (send *pr2* :move-to (send *room610* :spot-chair-spot) :world)
  (if (not (send *ri* :move-to (send *room610* :spot-chair-spot) :frame-id *room610-origin*))
	  (return-from move-to-chair-larm nil)
	  )
  (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
  (use-tilt-laser-obstacle-cloud t)
  t)

(defun move-to-remote-larm ()
  (warn "move-to-remote-larm~%")

  (send *pr2* :angle-vector #f(50.1262 0.0 -10.0 40.0 -70.0 70.0 -90.0 90.0 -5.0 70.0 -105.0 -90.0 70.0 -6.0 20.0 0.007004 -1.914))
  (send *pr2* :torso :waist-z :joint-angle 50)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
  (send *ri* :stop-grasp :larm)

  (use-tilt-laser-obstacle-cloud nil)
  (clear-costmap)
  (send *pr2* :move-to (send *room610* :spot-remote-spot) :world)
  (if (not (send *ri* :move-to (send *room610* :spot-remote-spot) :frame-id *room610-origin*))
	  (return-from move-to-chair-larm nil)
	  )
  (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
  (use-tilt-laser-obstacle-cloud t)
  t)


(defun move-to-chair2 ()
  (warn "move-to-chair2~%")
  (use-tilt-laser-obstacle-cloud nil)
  (clear-costmap)
  (send *pr2* :move-to (send *room610* :spot-chair-spot2) :world)
  (if (not (send *ri* :move-to (send *room610* :spot-chair-spot2) :frame-id *room610-origin*))
	  (return-from move-to-chair2 nil)
	  )
  (use-tilt-laser-obstacle-cloud t)
  t)


(defun move-to-back-chair ()
  (warn "move-to-back-chair~%")
  (use-tilt-laser-obstacle-cloud nil)
  (clear-costmap)
  (if *moved-chair-spot*
    (progn
      (send *pr2* :move-to *moved-chair-spot* :world)
      (send *ri* :move-to *moved-chair-spot* :frame-id *room610-origin*)
    )
    (progn
      (ros::ros-error "I Don't now where *moved-chair-spot*")
      (send *pr2* :move-to (send *room610* :spot-moved-back-chair) :world)
      (send *ri* :move-to (send *room610* :spot-moved-back-chair) :frame-id *room610-origin* )
    )
   )
  (use-tilt-laser-obstacle-cloud t)
)


(defun move-to-laundry ()
  (warn "move-to-laundry~%")
  (use-tilt-laser-obstacle-cloud nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (clear-costmap)
  (send *pr2* :move-to (send *room610* :spot-laundry-spot) :world)
  (if (not (send *ri* :move-to (send *room610* :spot-laundry-spot) :frame-id *room610-origin*))
	  (return-from move-to-laundry nil)
	  )
  (use-tilt-laser-obstacle-cloud t)
  t)

(defun move-to-broom ()
  (warn "move-to-broom~%")
    (use-tilt-laser-obstacle-cloud nil)
    (send *pr2* :move-to (send *room610* :spot-pick-broom-spot) :world)
    (if (not (send *ri* :move-to (send *room610* :spot-pick-broom-spot) :frame-id *room610-origin*))
		(return-from move-to-broom nil)
		)
    (use-tilt-laser-obstacle-cloud t)
	t)


(defun rarm-tuck ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :rarm :wrist-r  :joint-angle 19.9)
  (send *pr2* :rarm :wrist-p  :joint-angle -6)
  (send *pr2* :rarm :elbow-r  :joint-angle 69.8225)
  (send *pr2* :rarm :elbow-p  :joint-angle -90.0744)
  (send *pr2* :rarm :shoulder-r  :joint-angle -104.746)
  (send *pr2* :rarm :shoulder-p  :joint-angle 69.8)
  (send *pr2* :rarm :collar-y  :joint-angle -5.05)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  )

(defun larm-tuck ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :larm :wrist-r  :joint-angle 20)
  (send *pr2* :larm :wrist-p  :joint-angle -5)
  (send *pr2* :larm :elbow-r  :joint-angle 69)
  (send *pr2* :larm :elbow-p  :joint-angle -90)
  (send *pr2* :larm :shoulder-r  :joint-angle 79)
  (send *pr2* :larm :shoulder-p  :joint-angle 60)
  (send *pr2* :larm :collar-y  :joint-angle 0)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  )


(defun move-to-table-front ()
  (warn "move-to-table-front~%")
  (sweep-under-table-init-slim)
;;  (rehold-mop)
  (raise-mop)
  (use-tilt-laser-obstacle-cloud nil)
  (send *pr2* :move-to (send *room610* :spot-sweep-spot) :world)
  (let ((cnt 0))
  (while (not (send *ri* :move-to (send *room610* :spot-sweep-spot) :frame-id *room610-origin*))
      (if (> (incf cnt) 3)
          (raise-mop)
	  (return-from move-to-table-front nil)
	  )
      ))
  (use-tilt-laser-obstacle-cloud t)
  t)

(defun pass-broom-to-right-hand ()
  (warn "pass-broom-to-right-hand~%"))

(defun move-to-sweep-floor ()
  (warn "move-to-sweep-floor~%")
  (send *ri* :go-pos-unsafe -0.15 0)
  (send *pr2* :translate (send *pr2* :transform-vector #f(-150 0 0)) :world))

(defun sweep-floor ()
  (warn "sweep-floor (furuta)~%")
  ;(furuta-sweep)
  )

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
    (send *ri* :wait-interpolation)
    (send *ri* :go-pos-unsafe -0.1 0 0)
    (pr2-reset-pose)
    )
  )

(defun demo ()
  (speak-jp "部屋の掃除を始めます")
  (setup)
  (pr2-reset-pose)

  (speak-jp "水を渡します")
  (while (not (move-to-mug-pick-up-position)))
  (pick-up-mug)
  (while (not (move-to-mug-handover-position)))
  (handover-mug)
  
  (speak-jp "トレイをキッチンに運びます")
  (while (not (move-to-table-side)))
  (while (not (pick-tray-table-side)))
  ;;
  (while (not (move-to-sink)))
  (place-tray-sink)
  (send *ri* :go-pos-unsafe 0 0 -90)

  (speak-jp "リモコンをとります")
  (while (not (move-to-remote-larm)))
  (pick-up-remote)
  (while (not (move-to-remote-handover-position)))
  (handover-remote)
  
  (speak-jp "洗濯物を洗濯機に入れます")
  (while (not (move-to-chair-larm)))
  (while (not (pick-cloth-chair)))
  (send *ri* :go-pos-unsafe 0 0 90)

  (while (not (move-to-laundry)))
  (put-cloth-larm)

  (speak-jp "モップをとります")
  (while (not (move-to-broom)))
  (pick-broom-laundry)
  (send *ri* :go-pos-unsafe 0 0 -90)
  ;;
  (speak-jp "いすをひきます")
  (while (not (move-to-chair2)))
  (pull-chair-table)
  ;;
  (speak-jp "モップをかけます")
  (while (not (move-to-table-front)))
  (sweep-under-table-front)

  (sweep-under-table-dual-arm)
  (speak-jp "部屋の掃除が終わりました")
  (use-tilt-laser-obstacle-cloud t)

  (speak-jp "料理手伝います")
  (while (not (move-to-kitchen-knife)))
  (pick-up-kitchen-knife)
  (handover-kitchen-knife)
)


(warn "type (demo) to start~%")
(warn "type (setup) to revert all object location~%")

(setup)
