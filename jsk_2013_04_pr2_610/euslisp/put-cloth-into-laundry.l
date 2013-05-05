
(defun put-cloth-into-laundry ()
  (warn "put-cloth-into-laundry is too long. So ...~%")
  (warn "use (put-cloth)~%")
)

(defun put-cloth ()
  (warn "open-laundry-door (kuroiwa)~%")
  (let ((release (send *laundry* :handle-contents))
        (handle (send *laundry* :handle-handle-cap))
        (button (send *laundry* :handle-open-button))
        (debug nil)
        (r 500) (depth 500) (near 80) (cap-point (make-coords)) dst front)

    ;;STEP0 open collar-y
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :rarm :collar-y :joint-angle -90)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    ;; STEP1 Push the Button
    (setq dst (make-coords) front 120)
    (send dst :init
          :pos (send (send (send button :copy-worldcoords) :translate (float-vector (- (* near 1.2)) -30 25) (send *pr2* :worldcoords)) :worldpos)
          :rot (send (send (send *pr2* :copy-worldcoords) :rotate (deg2rad -45) :x) :worldrot))

    (send dst :rotate (deg2rad -135) :y :world)


    ;; ready to push
    (send *pr2* :rarm :inverse-kinematics
          dst
          :look-at-target t
          :debug-view debug
          :revert-if-fail nil)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *pr2* :rarm :wrist-r :joint-angle 90)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (print "ready to put")

    ;; push
    (send *pr2* :rarm :move-end-pos (float-vector 150 0 0))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 100)
    (send *ri* :wait-interpolation)
    (print "push")
    
    ;; pull the right arm for opening the cap
    (send *pr2* :rarm :move-end-pos (float-vector -150 0 -30))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) )
    (send *ri* :wait-interpolation)
    (print "pull")


    (send *pr2* :rarm :move-end-pos (float-vector 0 -80 0))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (print "pull")


    ;; store current position of right arm
    (setq former (send (send *pr2* :rarm :end-coords) :copy-worldcoords))

;    (speak-en "STEP1 complete")
    (print "STEP1 done")

    ;; STEP2 Open the Cap
    (setq dst (make-coords))
    ;; modify the model's parameters
    (send cap-point :init 
          :pos (send 
                (send (send handle :copy-worldcoords) :translate #f(-60 80 0)
                      (send *pr2* :worldcoords))
                :worldpos)
          :rot (send handle :worldrot))

    ;; insert the right arm into the gap
    (send *pr2* :rarm :inverse-kinematics
          cap-point 
          :look-at-target t
          :debug-view debug
          :revert-if-fail nil)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (print "insert")
    
    ;; open the cap
    ;; modify the model's parameters
    (send dst :init 
          :pos (send 
                (send (send release :copy-worldcoords) :translate
                      #f(130 -50 90) (send *pr2* :worldcoords))
                :worldpos)
          :rot (send *pr2* :worldrot))

    ;; ready to put
    (send *pr2* :rarm :inverse-kinematics
          (send (send dst :copy-worldcoords) 
                :translate (float-vector (- (* depth 0.4)) 0 near) (send *pr2* :worldcoords))
          :look-at-target t
          :debug-view debug
          :revert-if-fail nil)

    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (print "ready to put")

    ;; put the cloth
    (send *pr2* :rarm :inverse-kinematics
          dst
          :rotation-axis nil
          :look-at-target t
          :debug-view debug
          :revert-if-fail nil)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp :rarm)
    (send *pr2* :rarm :wrist-r :joint-angle -180)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *pr2* :rarm :wrist-r :joint-angle 0)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    ;; (send *ri* :start-grasp :rarm)
    (print "put the cloth")

    ;; pull the right arm
    (send *pr2* :rarm :inverse-kinematics
          (send (send dst :copy-worldcoords)
                :translate (float-vector (- (* depth 0.7)) 0 (* near 1.2)) (send *pr2* :worldcoords))
          :look-at-target t
          :debug-view debug
          :revert-if-fail nil)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (print "pull the right arm")


;    (speak-en "STEP3 complete")

    ;; STEP4 Close the Cap

    ;; move the right arm in order to avoid the cap
    (send *pr2* :rarm :inverse-kinematics
          (send (send former :copy-worldcoords)
                :translate #f(-100 -200 -100) (send *pr2* :worldcoords))
          :rotation-axis nil
          :debug-view debug
          :revert-if-fail nil)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (print "move the right arm")

    ;; ready to close
    (send *pr2* :larm :move-end-pos (float-vector 0 (- (* r 0.6)) 0) (send *pr2* :worldcoords))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (print "ready to close")

    ;; close the cap
    (send *pr2* :larm :inverse-kinematics
          (send (send cap-point :copy-worldcoords) 
                :translate (float-vector (+ near 50) 0 0) (send *pr2* :worldcoords))
          :rotation-axis nil
          :debug-view debug
          :revert-if-fail nil)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (print "close the cap")


;    (speak-en "STEP4 complete")

    ;; initialize the pose
    (send *pr2* :reset-pose)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
;    (speak-jp "ミッションコンプリート")
    ))
