#!/usr/bin/env roseus

(load "package://pr2eus/pr2-utils.l")
(load "package://pr2eus/pr2-interface.l")

(load "models/room73b2-scene.l")
(load "models/blue-clear-cup-object.l")
(load "models/flower-tea-cup-dish-object.l")
(load "models/white-bread-salad-dish-object.l")

(defun setup ()
  (let (work-spot table-name table-top)
    ;; setup environemnt
    (room73b2)
    (setq *dish1* (blue-clear-cup)
          *dish2* (flower-tea-cup-dish)
          *dish3* (white-bread-salad-dish))
    ;;
    ;;(setq table-name "room73b2-coe-800-shelf")
    (setq table-name "room73b2-coe-450-shelf")
    (send-all (send (send *room73b2* :object table-name) :bodies) :worldcoords) ;; magic code to update vertices position???
    (setq table-top
          (car (send (send (make-bounding-box (flatten (send-all (send (send *room73b2* :object table-name) :faces) :vertices))) :body) :faces :prism :top))) ;; we need better way to get table top face

    (send *dish1* :locate (cadr (send table-top :centroid)) :world)
    (send *dish2* :locate (cadr (send table-top :centroid)) :world)
    (send *dish2* :locate #f(0  300 0))
    (send *dish3* :locate (cadr (send table-top :centroid)) :world)
    (send *dish3* :locate #f(0 -300 0))

    ;; setup robot
    (pr2)

    ;; setup origin coords
    (setq *room73b2-origin* "/eng2/7f/73b2")

    ;; draw on IRT-viewer
    (objects (list *room73b2* *pr2* *dish1* *dish2* *dish3*))

    ;; create robot-interface
    (setq *ri* (instance pr2-interface :init :objects (append (list *dish1* *dish2* *dish3*) (send *room73b2* :objects))))

    ;; setup navigation pramaters
    (change-inflation-range 0.1)
    (clear-costmap)

    ;; move to initial position and posture
    ;;(setq work-spot (send *room73b2* :spot "/eng2/7f/room73B2-front-kitchen-table"))
    (setq work-spot (send *room73b2* :spot "coe-spot")) ;; need /eng2/7f/.. style spot name
    (send *pr2* :move-to work-spot :world)
    (send *ri* :move-to work-spot :frame-id *room73b2-origin*)

    (send *pr2* :reset-manip-pose)
    (send *pr2* :arms :shoulder-p :joint-angle 0)
    (send *pr2* :torso :waist-z :joint-angle)

    (send *ri* :angle-vector (send *pr2* :angle-vector) 1500) ;; 1500[msec]
    (send *ri* :stop-grasp) ;; open hand
    (send *irtviewer* :draw-objects)
    ))

(defun pick-dish (dish)
  (let (move-arm handle avs)
  ;; need to choose appropriate handle, but now we takes first one
    (case (send *pr2* :select-target-arm dish)
      (:rarm (setq handle (elt (send dish :handle) 2)))
      (:larm (setq handle (elt (send dish :handle) 4))))

    ;; solve ik
    (setq move-arm (send *pr2* :select-target-arm handle))
    (send *pr2* :inverse-kinematics handle)
    (send *irtviewer* :draw-objects)
    (push (send *pr2* :angle-vector) avs) ;; push to angle-vector sequence

    ;; set way point
    (send *pr2* move-arm :move-end-pos #f(-100 0 0))
    (push (send *pr2* :angle-vector) avs) ;; push to angle-vector sequence

    ;; send to real robot
    (send *ri* :angle-vector-sequence avs (list 500))
    (send *ri* :wait-interpolation)

    ;; grasp
    (send *ri* :start-grasp)

    ;; pick up
    (send *pr2* :angle-vector (car (last avs))) ;; set current posture
    (send *pr2* move-arm :end-coords :assoc dish) ;; assoc dish and hand
    (send *pr2* move-arm :move-end-pos #f(0 0 100) :world)
    (send *irtviewer* :draw-objects)

    ;; set to real robot
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ))

(defun place-dish (target-dish)
  (let (move-arm grasp-dish target-coords avs)
    (cond ((send *pr2* :larm :end-coords :descendants)
           (setq move-arm :larm
                 grasp-dish (car (send *pr2* :larm :end-coords :descendants))))
          ((send *pr2* :rarm :end-coords :descendants)
           (setq move-arm :rarm
                 grasp-dish (car (send *pr2* :rarm :end-coords :descendants))))
          (t
           (warning-message 1 "nothing to place....~%")
           (return-from place-dish nil)))

    (setq target-coords
          (send (send grasp-dish :transformation (send *pr2* move-arm :end-coords)) :transform target-dish :world)) ;; set target-coords where center of grasp-dish and target-dish matches
    (send target-coords :translate #f(0 0 20) :world) ;; not to break dish
    (send *pr2* move-arm :inverse-kinematics target-coords)

    (send *irtviewer* :draw-objects)

    (send *pr2* move-arm :end-coords :dissoc grasp-dish) ;; assoc dish and hand
    (push (send *pr2* :angle-vector) avs) ;; push to angle-vector sequence

    ;; set way point
    (send *pr2* move-arm :move-end-pos #f(0 0 100))
    (push (send *pr2* :angle-vector) avs) ;; push to angle-vector sequence

    ;; send to real robot
    (send *ri* :angle-vector-sequence avs (list 500))
    (send *ri* :wait-interpolation)

    ;;
    (send *ri* :stop-grasp)

    ;; remove hand from dish
    (send *pr2* move-arm :move-end-pos #f(-100 0 0) :world)
    (send *irtviewer* :draw-objects)
    ;; send to real robot
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)

    ;; back to reset pose
    (send *pr2* :reset-manip-pose)
    (send *pr2* :arms :shoulder-p :joint-angle 0)
    (send *pr2* :torso :waist-z :joint-angle)

    (send *ri* :angle-vector (send *pr2* :angle-vector) 1500) ;; 1500[msec]
    (send *ri* :stop-grasp) ;; open hand
    (send *irtviewer* :draw-objects)
    ))

(defun demo ()
  (pick-dish  *dish2*)
  (place-dish *dish3*)
  (pick-dish  *dish1*)
  (place-dish *dish2*)
  )

(warn "
;; to play...
 (setup)
 (demo)
")

;;(setup)
;;(demo)



