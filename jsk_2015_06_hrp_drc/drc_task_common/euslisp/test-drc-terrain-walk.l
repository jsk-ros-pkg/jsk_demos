(load "package://drc_task_common/euslisp/drc-testbed-models.l")
(load "package://jsk_footstep_planner/euslisp/footstep_planner_util.l")

;;;;;;;;;;;;;;;;
;; utilities
;;;;;;;;;;;;;;;;
(defun walking-pose ;; this should be method
  (robot
   &key (root-link-height-offset 0)
        (root-link-pitch-offset 0)
        (root-link-roll-offset 0)
        (fix-coords (make-coords))
        (default-pose-method :reset-manip-pose))
  "Generate and set walking pose.
   default-pose-method is initial pose, reset-manip-pose by default.
   Generated pose is near from default-pose-method pose.
   root-link-height-offset is root height offset [mm] from default-pose-method.
   root-link-pitch-offset and root-link-roll-offset are root pitch and roll offset [deg] from default-pose-method."
  (send robot default-pose-method)
  (send robot :fix-leg-to-coords fix-coords)
  (let ((lc (mapcar #'(lambda (l)
                        (send robot l :end-coords :copy-worldcoords))
                    '(:rleg :lleg))))
    (send robot :move-coords
          (send
           (send
            (send (send (car (send robot :links)) :copy-worldcoords)
                  :translate (float-vector 0 0 root-link-height-offset))
            :rotate (deg2rad root-link-pitch-offset) :y)
           :rotate (deg2rad root-link-roll-offset) :x)
          (car (send robot :links)))
    (mapcar #'(lambda (l c)
                (send robot l :inverse-kinematics c))
            '(:rleg :lleg) lc)
    (send robot :move-centroid-on-foot :both '(:rleg :lleg))
    (send robot :angle-vector)
    ))

(defun walking-pose-with-toe-joint ;; this should be method
  (robot
   &key (root-link-height-offset 0)
        (root-link-pitch-offset 0)
        (root-link-roll-offset 0)
        (fix-coords (make-coords))
        (default-pose-method :reset-manip-pose)
        ;;
        (toe-joint-angles (list 0 0))) ;; rleg lleg
  "Generate and set walking pose with setting toe joint angles.
   "
  (walking-pose robot
                :root-link-height-offset root-link-height-offset
                :root-link-pitch-offset root-link-pitch-offset
                :root-link-roll-offset root-link-roll-offset
                :fix-coords fix-coords :default-pose-method default-pose-method)
  (let ((lc (mapcar #'(lambda (l)
                        (send robot l :end-coords :copy-worldcoords))
                    '(:rleg :lleg))))
    (mapcar #'(lambda (l ja)
                (send robot l :toe-p :joint-angle ja))
            '(:rleg :lleg) toe-joint-angles)
    (mapcar #'(lambda (l c)
                (send robot l :inverse-kinematics c))
            '(:rleg :lleg) lc)
    (send robot :move-centroid-on-foot :both '(:rleg :lleg))
    (send robot :angle-vector)
    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Stair walking functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun test-stair-walking-common
  (robot stair-height
         &key (fwd-stride 250) (stair-once nil) (y-or-n-p nil)
              (initial-support-leg :lleg))
  (let ((counter-leg (case initial-support-leg (:rleg :lleg) (:lleg :rleg))))
    (if stair-once
        (send *ri* :set-foot-steps
              (list
               (make-coords :coords (send robot initial-support-leg :end-coords :copy-worldcoords)
                            :name initial-support-leg)
               (make-coords :coords (send (send robot counter-leg :end-coords :copy-worldcoords) :translate (float-vector fwd-stride 0 stair-height))
                            :name counter-leg)
               (make-coords :coords (send (send robot initial-support-leg :end-coords :copy-worldcoords) :translate (float-vector fwd-stride 0 stair-height))
                            :name initial-support-leg)))
      (progn
        (send *ri* :set-foot-steps
              (list
               (make-coords :coords (send robot initial-support-leg :end-coords :copy-worldcoords)
                            :name initial-support-leg)
               (make-coords :coords (send (send robot counter-leg :end-coords :copy-worldcoords) :translate (float-vector fwd-stride 0 stair-height))
                            :name counter-leg)
               ))
        (if y-or-n-p
            (y-or-n-p ";; next step?"))
        (send *ri* :set-foot-steps
              (list
               (make-coords :coords (send (send robot counter-leg :end-coords :copy-worldcoords) :translate (float-vector fwd-stride 0 stair-height))
                            :name counter-leg)
               (make-coords :coords (send (send robot initial-support-leg :end-coords :copy-worldcoords) :translate (float-vector fwd-stride 0 stair-height))
                            :name initial-support-leg)
               ))))
    ))

;; thin 40[mm] wooden bar
(defun test-stair-walk-0-up
  (robot &key (fwd-stride 250) (stair-once nil) (initial-support-leg :lleg))
  (test-stair-walking-common robot 40.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

(defun test-stair-walk-0-down
  (robot &key (fwd-stride 250) (stair-once nil) (initial-support-leg :lleg))
  (test-stair-walking-common robot -40.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

;; thick 88[mm] wooden bar
(defun test-stair-walk-1-up
  (robot &key (fwd-stride 250) (stair-once nil) (initial-support-leg :lleg))
  (test-stair-walking-common robot 88.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

(defun test-stair-walk-1-down
  (robot &key (fwd-stride 250) (stair-once nil) (initial-support-leg :lleg))
  (test-stair-walking-common robot -88.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

;; concrete block 150[mm], thin side
(defun test-stair-walk-2-up
  (robot &key (fwd-stride 270) (stair-once nil) (initial-support-leg :lleg))
  (test-stair-walking-common robot 150.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

(defun test-stair-walk-2-down
  (robot &key (fwd-stride 270) (stair-once nil) (initial-support-leg :lleg))
  (test-stair-walking-common robot -150.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

;; concrete block, thick side, 190[mm] Qualification block
(defun test-stair-walk-3-up
  (robot &key (fwd-stride 270) (stair-once nil) (initial-support-leg :lleg))
  (test-stair-walking-common robot 190.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

(defun test-stair-walk-3-down
  (robot &key (fwd-stride 270) (stair-once nil) (initial-support-leg :lleg))
  (test-stair-walking-common robot -190.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

;; concrete block, thick side, 20[mm] Qualification block
(defun test-stair-walk-4-up
  (robot &key (fwd-stride 270) (stair-once t) (initial-support-leg :lleg))
  (test-stair-walking-common robot 200.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

(defun test-stair-walk-4-down
  (robot &key (fwd-stride 270) (stair-once t) (initial-support-leg :lleg))
  (test-stair-walking-common robot -200.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

;; concrete block stair in TerrainFloor.wrl
(defun test-stair-walk-5-up
  (robot &key (fwd-stride 250) (stair-once nil))
  (test-stair-walking-common robot (* 1e3 0.1524) :fwd-stride fwd-stride :stair-once stair-once))

(defun test-stair-walk-5-down
  (robot &key (fwd-stride 250) (stair-once nil))
  (test-stair-walking-common robot (* -1e3 0.1524) :fwd-stride fwd-stride :stair-once stair-once))

;;;;;;;;
;; demo functions
;;;;;;;;
(defun setup-terrain-walk-hrp2jsknts
  ()
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
  (hrp2jsknts-init)
  (setq *robot* *hrp2jsknts*)
  (objects (list *robot*))
  (walking-pose *robot* :root-link-height-offset -70)
  )
(warn ";; (setup-terrain-walk-hrp2jsknts)~%")

(defun setup-terrain-walk-hrp2jsknt
  ()
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
  (hrp2jsknt-init)
  (setq *robot* *hrp2jsknt*)
  (objects (list *robot*))
  (walking-pose *robot* :root-link-height-offset -70)
  )
(warn ";; (setup-terrain-walk-hrp2jsknt)~%")

(defun setup-terrain-walk-hrp2jsk
  ()
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
  (hrp2jsk-init)
  (setq *robot* *hrp2jsk*)
  (objects (list *robot*))
  (walking-pose *robot* :root-link-height-offset -70)
  )
(warn ";; (setup-terrain-walk-hrp2jsk)~%")

(defun setup-terrain-walk-jaxon
  ()
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
  (jaxon-init)
  (setq *robot* *jaxon*)
  (objects (list *robot*))
  (walking-pose *robot* :root-link-height-offset -70)
  )
(warn ";; (setup-terrain-walk-jaxon)~%")

(defun setup-terrain-walk-staro
  ()
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/staro-interface.l")
  (staro-init)
  (setq *robot* *staro*)
  (objects (list *robot*))
  (walking-pose *robot* :root-link-height-offset -70)
  )
(warn ";; (setup-terrain-walk-staro)~%")

(defun setup-terrain-walk-real-hrp2jsknts
  ()
  (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-auto-balancer)
  (send *ri* :start-st)
  (send *ri* :set-gait-generator-param
        :stair-trajectory-way-point-offset #f(0.03 -0.0 0)
        :swing-trajectory-delay-time-offset 0.25
        :default-orbit-type :stair
        :default-step-time 1.3
        :default-double-support-ratio 0.16
        :default-step-height 0.03)
  (send *ri* :set-auto-balancer-param :default-zmp-offsets (list (float-vector 5 30 0) (float-vector 5 -30 0)))
  )

(defun setup-terrain-walk-real-hrp2jsknt
  ()
  (setup-terrain-walk-real-hrp2jsknts)
  )

(defun setup-terrain-walk-real-hrp2jsk
  ()
  (setup-terrain-walk-real-hrp2jsknts)
  )

(defun setup-terrain-walk-real-jaxon
  ()
  (setup-terrain-walk-real-hrp2jsknts)
  )

(defun setup-terrain-walk-real-staro
  ()
  (setup-terrain-walk-real-hrp2jsknts)
  )

(warn ";; (setup-terrain-walk-real-hrp2jsknts)~%")
(warn ";; (setup-terrain-walk-real-hrp2jsknt)~%")
(warn ";; (setup-terrain-walk-real-hrp2jsk)~%")

(defun test-15cm-block-hrp2jsknts (&key (stair-once t))
  (print ";; up")
  (test-stair-walk-2-up *robot* :stair-once stair-once)
  (print ";; go pos fwd")
  (read-line)
  (send *ri* :go-pos 0.2 0 0)
  (print ";; down")
  (read-line)
  (test-stair-walk-2-down *robot* :stair-once stair-once :fwd-stride 240)
  )

(defclass drc-sagami-slope
  :super cascaded-link
  :slots (handles)
  )

(defmethod drc-sagami-slope
  (:init
   ()
   (send-super :init :name "drc-sagami-slope")
   (let ((bb (make-prism (list (float-vector 0 0 0) (float-vector 825 210 0) (float-vector (* 2 825) 0 0))
                         1000)))
     (send bb :rotate pi/2 :x)
     (send bb :translate (float-vector 0 0 -500))
     (setq links (list (instance bodyset-link :init (make-cascoords) :bodies (list bb) :name :root-link)))
      (send self :assoc (car (send self :links)))
      (setq handles (list
                     (make-cascoords :rpy (list 0 (deg2rad -14.5) 0) :name :first-slope-start)
                     (send (make-cascoords :rpy (list 0 (deg2rad -14.5) 0) :name :first-slope-end)
                           :translate (float-vector 850 0 0))
                     (make-cascoords :pos (float-vector 825 0 210)
                                     :rpy (list 0 (deg2rad 14.5) 0)
                                     :name :second-slope-start)
                     (send (make-cascoords :pos (float-vector 825 0 210)
                                           :rpy (list 0 (deg2rad 14.5) 0)
                                           :name :second-slope-end)
                           :translate (float-vector 850 0 0))
                     (make-cascoords :pos (float-vector (* 825 2) 0 0) :name :second-slope-end2)
                     ))
      (dolist (h handles) (send self :assoc h))
      (send self :init-ending)
      self))
  (:handle (name) (find name handles :test #'equal :key #'(lambda (x) (send x :name))))
  )

(defun make-drc-sagami-slope
  ()
  (instance drc-sagami-slope :init))

(defun test-drc-sagami-slope-walk-hrp2jsknt
  ()
  (load "package://jsk_footstep_planner/euslisp/robot-model-util.l")
  (send *robot* :gen-footstep-parameter)
  (walking-pose *robot* :root-link-height-offset -40)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (setq *slope* (make-drc-sagami-slope))
  (send *slope* :translate (float-vector 150 0 0))
  (print ";; Put the robot in the front of slope. Press Enter.")
  (read-line)
  (print ";; Done.")
  (objects (list *robot* *slope*))
  (send *ri* :set-gait-generator-param
        :default-double-support-ratio 0.24
        :default-step-height 0.03)
  (let ((of (elt (cadr (memq :default-half-offset (send *robot* :get :footstep-parameter))) 1)))
    (setq fs
          (list (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
                (make-coords :coords (send (send (send *slope* :handle :first-slope-start) :copy-worldcoords) :translate (float-vector 100 of 0)) :name :lleg)
                (make-coords :coords (send (send (send *slope* :handle :first-slope-start) :copy-worldcoords) :translate (float-vector 200 (- of) 0)) :name :rleg)
                (make-coords :coords (send (send (send *slope* :handle :first-slope-start) :copy-worldcoords) :translate (float-vector 300 of 0)) :name :lleg)
                (make-coords :coords (send (send (send *slope* :handle :first-slope-start) :copy-worldcoords) :translate (float-vector 400 (- of) 0)) :name :rleg)
                (make-coords :coords (send (send (send *slope* :handle :first-slope-start) :copy-worldcoords) :translate (float-vector 500 of 0)) :name :lleg)
                (make-coords :coords (send (send (send *slope* :handle :first-slope-start) :copy-worldcoords) :translate (float-vector 600 (- of) 0)) :name :rleg)
                (make-coords :coords (send (send (send *slope* :handle :first-slope-start) :copy-worldcoords) :translate (float-vector 700 of 0)) :name :lleg)
                (make-coords :coords (send (send (send *slope* :handle :first-slope-start) :copy-worldcoords) :translate (float-vector 750 (- of) 0)) :name :rleg)
                (make-coords :coords (send (send (send *slope* :handle :first-slope-start) :copy-worldcoords) :translate (float-vector 750 of 0)) :name :lleg)
                ))
    (send-all fs :draw-on :flush t)
    (send *robot* :publish-footstep fs)
    (print ";; Go up slope. Press Enter.")
    (read-line)
    (send *ri* :set-foot-steps fs)
    (print ";; Done.")
    (send *ri* :set-gait-generator-param :default-step-height 0.1)
    (setq fs (list
              (make-coords :coords (send (send (send *slope* :handle :first-slope-start) :copy-worldcoords) :translate (float-vector 750 of 0)) :name :lleg)
              (make-coords :coords (send (send (send *slope* :handle :second-slope-start) :copy-worldcoords) :translate (float-vector 100 (- of) 0)) :name :rleg)
              ))
    (send-all fs :draw-on :flush t)
    (send *robot* :publish-footstep fs)
    (print ";; Step over 1. Press Enter.")
    (read-line)
    (send *ri* :set-foot-steps fs)
    (print ";; Done.")
    (setq fs (list
              (make-coords :coords (send (send (send *slope* :handle :second-slope-start) :copy-worldcoords) :translate (float-vector 100 (- of) 0)) :name :rleg)
              (make-coords :coords (send (send (send *slope* :handle :second-slope-start) :copy-worldcoords) :translate (float-vector 100 of 0)) :name :lleg)
              ))
    (send-all fs :draw-on :flush t)
    (send *robot* :publish-footstep fs)
    (print ";; Step over 2. Press Enter.")
    (read-line)
    (send *ri* :set-foot-steps fs)
    (print ";; Done.")
    (send *ri* :set-gait-generator-param :default-step-height 0.03)
    (setq fs (list
              (make-coords :coords (send (send (send *slope* :handle :second-slope-start) :copy-worldcoords) :translate (float-vector 100 of 0)) :name :lleg)
              (make-coords :coords (send (send (send *slope* :handle :second-slope-start) :copy-worldcoords) :translate (float-vector 150 (- of) 0)) :name :rleg)
              (make-coords :coords (send (send (send *slope* :handle :second-slope-start) :copy-worldcoords) :translate (float-vector 250 of 0)) :name :lleg)
              (make-coords :coords (send (send (send *slope* :handle :second-slope-start) :copy-worldcoords) :translate (float-vector 350 (- of) 0)) :name :rleg)
              (make-coords :coords (send (send (send *slope* :handle :second-slope-start) :copy-worldcoords) :translate (float-vector 450 of 0)) :name :lleg)
              (make-coords :coords (send (send (send *slope* :handle :second-slope-start) :copy-worldcoords) :translate (float-vector 550 (- of) 0)) :name :rleg)
              (make-coords :coords (send (send (send *slope* :handle :second-slope-start) :copy-worldcoords) :translate (float-vector 650 of 0)) :name :lleg)
              (make-coords :coords (send (send (send *slope* :handle :second-slope-start) :copy-worldcoords) :translate (float-vector 650 (- of) 0)) :name :rleg)
              ))
    (send-all fs :draw-on :flush t)
    (send *robot* :publish-footstep fs)
    (print ";; Go down slope. Press Enter.")
    (read-line)
    (send *ri* :set-foot-steps fs)
    (print ";; Done.")
    (setq fs (list
              (make-coords :coords (send (send (send *slope* :handle :second-slope-start) :copy-worldcoords) :translate (float-vector 650 of 0)) :name :lleg)
              (make-coords :coords (send (send (send *slope* :handle :second-slope-end2) :copy-worldcoords) :translate (float-vector 50 (- of) 0)) :name :rleg)
              (make-coords :coords (send (send (send *slope* :handle :second-slope-end2) :copy-worldcoords) :translate (float-vector 50 of 0)) :name :lleg)))
    (send-all fs :draw-on :flush t)
    (send *robot* :publish-footstep fs)
    (print ";; Go to even plane. Press Enter.")
    (read-line)
    (send *ri* :set-foot-steps fs)
    (print ";; Done.")
    (send *ri* :set-gait-generator-param
        :default-double-support-ratio 0.16
        :default-step-height 0.03)
    ))

(defun make-sole-body (robot leg)
  (instance bodyset :init (make-cascoords :coords (send robot leg :end-coords :copy-worldcoords))
            :bodies (list (let ((b (make-prism (butlast (send (send robot :support-polygon leg) :vertices)) 5)))
                            (send b :set-color (case leg (:rleg :red) (:lleg :green)))
                            b))
            :name (read-from-string (format nil "~A-sole" leg))
            ))

;; Before terrain simulation, please execute

;; For hrp2jsknt
;;   $ rtmtest -t hrpsys_tools _gen_project.launch INPUT:=${CVSDIR}/OpenHRP/etc/HRP2JSKNT_for_OpenHRP3/HRP2JSKNTmain.wrl OUTPUT:=/tmp/HRP2JSKNT_for_DRCTestbedBlock.xml OBJECT_MODELS:="`rospack find hrpsys`/share/hrpsys/samples/environments/DRCTestbedTerrainJPBlock.wrl,0.4,-2.0,0,1,0,0,0, `rospack find openhrp3`/share/OpenHRP-3.1/sample/model/longfloor.wrl,0,0,0,1,0,0,0" CORBA_PORT:=15005 INTEGRATE:=false
;;   $ rtmlaunch hrpsys_ros_bridge_tutorials hrp2jsknt.launch PROJECT_FILE:=/tmp/HRP2JSKNT_for_DRCTestbedBlock.xml

;; For hrp2jsk
;;   $ rtmtest -t hrpsys_tools _gen_project.launch INPUT:=${CVSDIR}/OpenHRP/etc/HRP2JSK_for_OpenHRP3/HRP2JSKmain.wrl OUTPUT:=/tmp/HRP2JSK_for_DRCTestbedBlock.xml OBJECT_MODELS:="`rospack find hrpsys`/share/hrpsys/samples/environments/DRCTestbedTerrainJPBlock.wrl,0.4,-2.0,0,1,0,0,0, `rospack find openhrp3`/share/OpenHRP-3.1/sample/model/longfloor.wrl,0,0,0,1,0,0,0" CORBA_PORT:=15005 INTEGRATE:=false
;;   $ rtmlaunch hrpsys_ros_bridge_tutorials hrp2jsk.launch PROJECT_FILE:=/tmp/HRP2JSK_for_DRCTestbedBlock.xml

;; For jaxon
;;   $ rtmtest -t hrpsys_tools _gen_project.launch INPUT:=${CVSDIR}/euslib/rbrain/jaxon/JAXONmain.wrl OUTPUT:=/tmp/JAXON_for_DRCTestbedBlock.xml OBJECT_MODELS:="`rospack find hrpsys`/share/hrpsys/samples/environments/DRCTestbedTerrainJPBlock.wrl,0.4,-2.0,0,1,0,0,0, `rospack find openhrp3`/share/OpenHRP-3.1/sample/model/longfloor.wrl,0,0,0,1,0,0,0" CORBA_PORT:=15005 CONF_DT_OPTION:="--dt 0.002" SIMULATION_TIMESTEP_OPTION:="--timeStep 0.002" INTEGRATE:=false
;;   $ rtmlaunch hrpsys_ros_bridge_tutorials jaxon.launch PROJECT_FILE:=/tmp/JAXON_for_DRCTestbedBlock.xml

;; For staro
;;   $ rtmtest -t hrpsys_tools _gen_project.launch INPUT:=${CVSDIR}/euslib/rbrain/staro/STAROmain.wrl OUTPUT:=/tmp/STARO_for_DRCTestbedBlock.xml OBJECT_MODELS:="`rospack find hrpsys`/share/hrpsys/samples/environments/DRCTestbedTerrainJPBlock.wrl,0.4,-2.0,0,1,0,0,0, `rospack find openhrp3`/share/OpenHRP-3.1/sample/model/longfloor.wrl,0,0,0,1,0,0,0" CORBA_PORT:=15005 CONF_DT_OPTION:="--dt 0.002" SIMULATION_TIMESTEP_OPTION:="--timeStep 0.002" INTEGRATE:=false
;;   $ rtmlaunch hrpsys_ros_bridge_tutorials staro.launch PROJECT_FILE:=/tmp/STARO_for_DRCTestbedBlock.xml

(warn "~%;; Following simulations for hrp2jsknt, hrp2jsk, jaxon, staro~%~%")

(warn ";; Load each robot interface.l and Send walking-pose and (objects (list *robot* *rleg-sole* *lleg-sole* *terrain*))~%")

(defun setup-drc-testbed-terrain-simulation-hrp2jsknt ()
  (unless (boundp '*terrain*) (make-drc-terrain-japanese-block-ver :add-groud-p t))
  (setup-terrain-walk-hrp2jsknt)
  (setup-terrain-walk-real-hrp2jsknt)
  (walking-pose *robot* :root-link-height-offset -70) ;; <= -70mm
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *terrain* :newcoords (make-coords :pos (float-vector 400 -2000 0)))
  (setq *rleg-sole* (make-sole-body *robot* :rleg))
  (setq *lleg-sole* (make-sole-body *robot* :lleg))
  (setq *terrain-faces* (send *terrain* :get-all-terrain-top-faces))
  (objects (list *robot* *rleg-sole* *lleg-sole* *terrain*))
  )
(warn ";; (setup-drc-testbed-terrain-simulation-hrp2jsknt)~%")

(defun setup-drc-testbed-terrain-simulation-hrp2jsk ()
  (unless (boundp '*terrain*) (make-drc-terrain-japanese-block-ver :add-groud-p t))
  (setup-terrain-walk-hrp2jsk)
  (setup-terrain-walk-real-hrp2jsk)
  (walking-pose *robot* :root-link-height-offset -70) ;; <= -70mm
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *terrain* :newcoords (make-coords :pos (float-vector 400 -2000 0)))
  (setq *rleg-sole* (make-sole-body *robot* :rleg))
  (setq *lleg-sole* (make-sole-body *robot* :lleg))
  (setq *terrain-faces* (send *terrain* :get-all-terrain-top-faces))
  (objects (list *robot* *rleg-sole* *lleg-sole* *terrain*))
  )
(warn ";; (setup-drc-testbed-terrain-simulation-hrp2jsk)~%")

(defun setup-drc-testbed-terrain-simulation-jaxon ()
  (unless (boundp '*terrain*) (make-drc-terrain-japanese-block-ver :add-groud-p t))
  (setup-terrain-walk-jaxon)
  (setup-terrain-walk-real-jaxon)
  (walking-pose *robot* :root-link-height-offset -70) ;;  <= -70mm
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *terrain* :newcoords (make-coords :pos (float-vector 400 -2000 0)))
  (setq *rleg-sole* (make-sole-body *robot* :rleg))
  (setq *lleg-sole* (make-sole-body *robot* :lleg))
  (setq *terrain-faces* (send *terrain* :get-all-terrain-top-faces))
  (objects (list *robot* *rleg-sole* *lleg-sole* *terrain*))
  )
(warn ";; (setup-drc-testbed-terrain-simulation-jaxon)~%")

(defun setup-drc-testbed-terrain-simulation-staro ()
  (unless (boundp '*terrain*) (make-drc-terrain-japanese-block-ver :add-groud-p t))
  (setup-terrain-walk-staro)
  (setup-terrain-walk-real-staro)
  (walking-pose *robot* :root-link-height-offset -70) ;;  <= -70mm
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *terrain* :newcoords (make-coords :pos (float-vector 400 -2000 0)))
  (setq *rleg-sole* (make-sole-body *robot* :rleg))
  (setq *lleg-sole* (make-sole-body *robot* :lleg))
  (setq *terrain-faces* (send *terrain* :get-all-terrain-top-faces))
  (objects (list *robot* *rleg-sole* *lleg-sole* *terrain*))
  )
(warn ";; (setup-drc-testbed-terrain-simulation-staro)~%")

(warn "~%;; One step terrain walk simulation for each robot by footstep~%")
(warn ";; Footstep arguments are (rleg-fwd-offset[mm] lleg-fwd-offset[mm])~%")

(defun test-drc-testbed-terrain-simulation-hrp2jsknt-one
  (rleg-fwd-offset lleg-fwd-offset)
  (setq *prev-rleg-coords* (send *rleg-sole* :copy-worldcoords))
  (let ((crds (car (project-coords-on-to-plane
                    (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector rleg-fwd-offset 0 300) :world)
              *terrain-faces*
              (float-vector 0 0 1)))))
    (if crds
        (send *rleg-sole* :newcoords crds)
      (error ";; Cannot snap ~A~%" rleg-fwd-offset)))
  (let ((crds (car (project-coords-on-to-plane
              (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector lleg-fwd-offset 0 300) :world)
              *terrain-faces*
              (float-vector 0 0 1)))))
    (if crds
        (send *lleg-sole* :newcoords crds)
      (error ";; Cannot snap ~A~%" lleg-fwd-offset)))
  (send *ri* :set-foot-steps
        (list (make-coords :coords *prev-rleg-coords* :name :rleg)
              (make-coords :coords (send *lleg-sole* :copy-worldcoords) :name :lleg)
              (make-coords :coords (send *rleg-sole* :copy-worldcoords) :name :rleg)))
  )
(warn ";; (test-drc-testbed-terrain-simulation-hrp2jsknt-one 280 280)~%")

(defun test-drc-testbed-stair-simulation-hrp2jsknt-one
  (rleg-fwd-offset lleg-fwd-offset &key (set-foot-steps t))
  (setq *prev-rleg-coords* (send *rleg-sole* :copy-worldcoords))
  (let ((crds (car (project-coords-on-to-plane
                    (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector rleg-fwd-offset 0 1200) :world)
                    *stair-faces*
              (float-vector 0 0 1)))))
    (if crds
        (send *rleg-sole* :newcoords crds)
      (error ";; Cannot snap ~A~%" rleg-fwd-offset)))
  (let ((crds (car (project-coords-on-to-plane
              (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector lleg-fwd-offset 0 1200) :world)
              *stair-faces*
              (float-vector 0 0 1)))))
    (if crds
        (send *lleg-sole* :newcoords crds)
      (error ";; Cannot snap ~A~%" lleg-fwd-offset)))
  (if set-foot-steps
      (send *ri* :set-foot-steps
            (list (make-coords :coords *prev-rleg-coords* :name :rleg)
                  (make-coords :coords (send *lleg-sole* :copy-worldcoords) :name :lleg)
                  (make-coords :coords (send *rleg-sole* :copy-worldcoords) :name :rleg))))
  )

(defun test-drc-testbed-terrain-simulation-hrp2jsk-one
  (rleg-fwd-offset lleg-fwd-offset)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one rleg-fwd-offset lleg-fwd-offset)
  )
(warn ";; (test-drc-testbed-terrain-simulation-hrp2jsk-one 280 280)~%")

(defun test-drc-testbed-terrain-simulation-jaxon-one
  (rleg-fwd-offset lleg-fwd-offset)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one rleg-fwd-offset lleg-fwd-offset)
  )
(warn ";; (test-drc-testbed-terrain-simulation-jaxon-one 280 280)~%")

(defun test-drc-testbed-terrain-simulation-staro-one
  (rleg-fwd-offset lleg-fwd-offset)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one rleg-fwd-offset lleg-fwd-offset)
  )
(warn ";; (test-drc-testbed-terrain-simulation-staro-one 280 280)~%")

(warn "~%;; One through terrain walk simulation by some steps for each robot by footstep~%")

;; In the case of hrp2jsknt simulation, at 2nd block, 4th block and 6th block,
;; left foot sinks into blocks due to the saturation of lleg-ankle-r max-angle (20deg).
(defun test-drc-testbed-terrain-simulation-hrp2jsknt ()
  (warn ";; Step 1~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsknt-one 280 280)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one 280 280)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 2~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsknt-one 440 440)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one 440 440)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 3~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsknt-one 720 720)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one 720 720)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 4~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsknt-one 840 840)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one 840 840)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 5~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsknt-one 1150 1150)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one 1150 1150)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 6~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsknt-one 1260 1260)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one 1260 1260)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 7~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsknt-one 1520 1520)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one 1520 1520)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 8~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsknt-one 1600 1600)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one 1600 1600)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 9~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsknt-one 1900 1900)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one 1900 1900)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 10~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsknt-one 2070 2070)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one 2070 2070)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 11~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsknt-one 2320 2320)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one 2320 2320)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 12~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsknt-one 2460 2460)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one 2460 2460)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 13~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsknt-one 2750 2750)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one 2750 2750)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 14~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsknt-one 2900 2900)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one 2900 2900)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Final Step (Landing)~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsknt-landing)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-landing)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  )
(warn ";; (test-drc-testbed-terrain-simulation-hrp2jsknt)~%")

;; In the case of hrp2jsk simulation, at 2nd block, 4th block and 6th block,
;; either foot sinks into blocks due to the saturation of ankle-r max-angle (20deg).
(defun test-drc-testbed-terrain-simulation-hrp2jsk ()
  (warn ";; Step 1~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsk-one 280 280)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsk-one 280 280)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 2~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsk-one 440 440)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsk-one 440 440)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 3~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsk-one 720 720)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsk-one 720 720)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 4~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsk-one 840 840)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsk-one 840 840)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 5~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsk-one 1150 1150)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsk-one 1150 1150)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 6~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsk-one 1260 1260)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsk-one 1260 1260)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 7~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsk-one 1520 1520)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsk-one 1520 1520)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 8~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsk-one 1600 1600)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsk-one 1600 1600)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 9~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsk-one 1900 1900)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsk-one 1900 1900)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 10~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsk-one 2070 2070)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsk-one 2070 2070)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 11~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsk-one 2320 2320)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsk-one 2320 2320)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 12~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsk-one 2460 2460)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsk-one 2460 2460)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 13~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsk-one 2750 2750)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsk-one 2750 2750)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 14~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsk-one 2900 2900)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsk-one 2900 2900)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Final Step (Landing)~%")
  (warn ";; (test-drc-testbed-terrain-simulation-hrp2jsk-landing)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-hrp2jsk-landing)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  )
(warn ";; (test-drc-testbed-terrain-simulation-hrp2jsk)~%")

;; In the case of jaxon simulation,
;; for example on 1st block->2nd block, 2nd block->3rd block, 5th block->6th block,
;; foot collides with the edge of block due to probably the trajectory planner of end-effector.
(defun test-drc-testbed-terrain-simulation-jaxon ()
  (warn ";; Step 1~%")
  (warn ";; (test-drc-testbed-terrain-simulation-jaxon-one 280 280)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-jaxon-one 280 280)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 2~%")
  (warn ";; (test-drc-testbed-terrain-simulation-jaxon-one 440 440)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-jaxon-one 440 440)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 3~%")
  (warn ";; (test-drc-testbed-terrain-simulation-jaxon-one 720 720)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-jaxon-one 720 720)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 4~%")
  (warn ";; (test-drc-testbed-terrain-simulation-jaxon-one 840 840)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-jaxon-one 840 840)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 5~%")
  (warn ";; (test-drc-testbed-terrain-simulation-jaxon-one 1150 1150)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-jaxon-one 1150 1150)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 6~%")
  (warn ";; (test-drc-testbed-terrain-simulation-jaxon-one 1260 1260)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-jaxon-one 1260 1260)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 7~%")
  (warn ";; (test-drc-testbed-terrain-simulation-jaxon-one 1520 1520)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-jaxon-one 1520 1520)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 8~%")
  (warn ";; (test-drc-testbed-terrain-simulation-jaxon-one 1600 1600)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-jaxon-one 1600 1600)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 9~%")
  (warn ";; (test-drc-testbed-terrain-simulation-jaxon-one 1900 1900)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-jaxon-one 1900 1900)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 10~%")
  (warn ";; (test-drc-testbed-terrain-simulation-jaxon-one 2070 2070)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-jaxon-one 2070 2070)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 11~%")
  (warn ";; (test-drc-testbed-terrain-simulation-jaxon-one 2320 2320)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-jaxon-one 2320 2320)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 12~%")
  (warn ";; (test-drc-testbed-terrain-simulation-jaxon-one 2460 2460)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-jaxon-one 2460 2460)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 13~%")
  (warn ";; (test-drc-testbed-terrain-simulation-jaxon-one 2750 2750)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-jaxon-one 2750 2750)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 14~%")
  (warn ";; (test-drc-testbed-terrain-simulation-jaxon-one 2900 2900)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-jaxon-one 2900 2900)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Final Step (Landing)~%")
  (warn ";; (test-drc-testbed-terrain-simulation-jaxon-landing)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-jaxon-landing)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  )
(warn ";; (test-drc-testbed-terrain-simulation-jaxon)~%")

;; In the case of staro simulation,
;; on 1st block->2nd block, 2nd block->3rd block, 5th block->6th block,
;; the edge of staro foot collides with the edge of block due to probably the trajectory planner of end-effector.
(defun test-drc-testbed-terrain-simulation-staro ()
  (warn ";; Step 1~%")
  (warn ";; (test-drc-testbed-terrain-simulation-staro-one 280 280)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-staro-one 280 280)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 2~%")
  (warn ";; (test-drc-testbed-terrain-simulation-staro-one 440 440)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-staro-one 440 440)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 3~%")
  (warn ";; (test-drc-testbed-terrain-simulation-staro-one 720 720)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-staro-one 720 720)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 4~%")
  (warn ";; (test-drc-testbed-terrain-simulation-staro-one 840 840)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-staro-one 840 840)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 5~%")
  (warn ";; (test-drc-testbed-terrain-simulation-staro-one 1150 1150)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-staro-one 1150 1150)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 6~%")
  (warn ";; (test-drc-testbed-terrain-simulation-staro-one 1260 1260)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-staro-one 1260 1260)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 7~%")
  (warn ";; (test-drc-testbed-terrain-simulation-staro-one 1520 1520)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-staro-one 1520 1520)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 8~%")
  (warn ";; (test-drc-testbed-terrain-simulation-staro-one 1600 1600)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-staro-one 1600 1600)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 9~%")
  (warn ";; (test-drc-testbed-terrain-simulation-staro-one 1900 1900)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-staro-one 1900 1900)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 10~%")
  (warn ";; (test-drc-testbed-terrain-simulation-staro-one 2070 2070)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-staro-one 2070 2070)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 11~%")
  (warn ";; (test-drc-testbed-terrain-simulation-staro-one 2320 2320)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-staro-one 2320 2320)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 12~%")
  (warn ";; (test-drc-testbed-terrain-simulation-staro-one 2460 2460)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-staro-one 2460 2460)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 13~%")
  (warn ";; (test-drc-testbed-terrain-simulation-staro-one 2750 2750)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-staro-one 2750 2750)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Step 14~%")
  (warn ";; (test-drc-testbed-terrain-simulation-staro-one 2900 2900)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-staro-one 2900 2900)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  (warn ";; Final Step (Landing)~%")
  (warn ";; (test-drc-testbed-terrain-simulation-staro-landing)~%")
  (read-line)
  (test-drc-testbed-terrain-simulation-staro-landing)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :wait-interpolation)
  )
(warn ";; (test-drc-testbed-terrain-simulation-staro)~%")

(defun test-drc-testbed-terrain-simulation-hrp2jsknt-landing
  (&key (landing-foot-midpos (float-vector 3200 0 0)))
  (setq *prev-rleg-coords* (send *rleg-sole* :copy-worldcoords))
  (send *robot* :fix-leg-to-coords (make-coords :pos landing-foot-midpos))
  (let ((crds (send *robot* :rleg :end-coords :copy-worldcoords)))
    (if crds
        (send *rleg-sole* :newcoords crds)
      (error ";; Cannot snap ~A~%" rleg-fwd-offset)))
  (let ((crds (send *robot* :lleg :end-coords :copy-worldcoords)))
    (if crds
        (send *lleg-sole* :newcoords crds)
      (error ";; Cannot snap ~A~%" lleg-fwd-offset)))
  (send *irtviewer* :draw-objects)
  (send *ri* :set-foot-steps
        (list (make-coords :coords *prev-rleg-coords* :name :rleg)
              (make-coords :coords (send *lleg-sole* :copy-worldcoords) :name :lleg)
              (make-coords :coords (send *rleg-sole* :copy-worldcoords) :name :rleg)))
  )

(defun test-drc-testbed-terrain-simulation-hrp2jsk-landing
  (&key (landing-foot-midpos (float-vector 3200 0 0)))
  (test-drc-testbed-terrain-simulation-hrp2jsknt-landing :landing-foot-midpos landing-foot-midpos)
  )

(defun test-drc-testbed-terrain-simulation-jaxon-landing
  (&key (landing-foot-midpos (float-vector 3200 0 0)))
  (test-drc-testbed-terrain-simulation-hrp2jsknt-landing :landing-foot-midpos landing-foot-midpos)
  )

(defun test-drc-testbed-terrain-simulation-staro-landing
  (&key (landing-foot-midpos (float-vector 3200 0 0)))
  (test-drc-testbed-terrain-simulation-hrp2jsknt-landing :landing-foot-midpos landing-foot-midpos)
  )

(defun setup-drc-testbed-stair-simulation-hrp2jsknt ()
  (unless (boundp '*stair*) (make-drc-stair))
  (setup-terrain-walk-hrp2jsknt)
  (setq *pitch-off* 15) ;; [deg]
  (walking-pose *robot* :root-link-pitch-offset *pitch-off*)
  (setup-terrain-walk-real-hrp2jsknt)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *stair* :newcoords (make-coords :pos (float-vector 180 0 0)))
  (setq *rleg-sole* (make-sole-body *robot* :rleg))
  (setq *lleg-sole* (make-sole-body *robot* :lleg))
  (setq *stair-faces* (send *stair* :get-all-stair-top-faces))
  (objects (list *robot* *rleg-sole* *lleg-sole* *stair*))
  ;;
  (test-drc-testbed-stair-simulation-hrp2jsknt-one 250 250 :set-foot-steps nil)
  (walking-pose *robot* :root-link-height-offset 0 :root-link-pitch-offset *pitch-off*)
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (walking-pose *robot* :root-link-height-offset -100 :root-link-pitch-offset *pitch-off*)
  (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
  (send *ri* :set-foot-steps
        (list (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
              (make-coords :coords (send *lleg-sole* :copy-worldcoords) :name :lleg)))
  (send *ri* :set-auto-balancer-param :default-zmp-offsets (list (float-vector 95 30 0) (float-vector 5 -30 0)))
  (unix:sleep 2)
  (walking-pose-with-toe-joint *robot* :root-link-height-offset -50 :root-link-pitch-offset *pitch-off* :toe-joint-angles (list -30 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :set-foot-steps
        (list (make-coords :coords (send *lleg-sole* :copy-worldcoords) :name :lleg)
              (make-coords :coords (send *rleg-sole* :copy-worldcoords) :name :rleg)))
  (walking-pose-with-toe-joint *robot* :root-link-height-offset -50 :root-link-pitch-offset *pitch-off* :toe-joint-angles (list 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun move-base-up-until-ik-fail
  (&key (target-coords (mapcar #'(lambda (l) (send *robot* l :end-coords :copy-worldcoords)) '(:rleg :lleg)))
        (target-centroid-pos (apply #'midpoint ratio (send-all target-coords :worldpos)))
        (limbs '(:rleg :lleg)))
  (let* ((rs-list)
         (ret nil)
         (count 0))
    (send *robot* :translate (float-vector 0 0 -100) :world)
    (while (or (setq ret (send *robot* :fullbody-inverse-kinematics
                               target-coords
                               :joint-args (list :absolute-p t)
                               :target-centroid-pos target-centroid-pos
                               ;;:debug-view :no-message
                               :dif-pos-ratio 0.5 :dif-rot-ratio 0.5
                               :stop 20 :warnp nil
                               :root-link-virtual-joint-weight #f(0.1 0.1 0 0 0 0)
                               :move-target (mapcar #'(lambda (l) (send *robot* l :end-coords)) limbs)
                               :link-list (mapcar #'(lambda (l) (send *robot* :link-list (send *robot* l :end-coords :parent))) limbs)))
               (< count 9))
      (incf count)
      (if ret
          (push (list :angle-vector (send *robot* :angle-vector) :root-coords (send (car (send *robot* :links)) :copy-worldcoords)) rs-list))
      (send *robot* :translate (float-vector 0 0 25) :world)
      )
    (let ((idx 2))
      (send *robot* :angle-vector (cadr (memq :angle-vector (elt rs-list idx))))
      (send *robot* :move-coords (cadr (memq :root-coords (elt rs-list idx))) (car (send *robot* :links))))
    (send *irtviewer* :draw-objects)
    (cadr rs-list)
    ))

(defun check-leg-reachability-for-one-step
  (prev-foot-coords next-foot-coords leg default-step-height default-step-time)
  (let ((rs-list))
    (labels ((tmp-fullbody-ik
              (tc target-centroid-pos)
              (send *robot* :fullbody-inverse-kinematics
                    tc
                    :joint-args (list :absolute-p t)
                    :dif-pos-ratio 0.5 :dif-rot-ratio 0.5
                    :target-centroid-pos target-centroid-pos
                    :root-link-virtual-joint-weight #f(0.1 0.1 0.1 0 0 0)
                    :move-target (list (send *robot* leg :end-coords) (send *robot* (case leg (:rleg :lleg) (:lleg :rleg)) :end-coords))
                    :link-list (mapcar #'(lambda (l) (send *robot* :link-list (send *robot* l :end-coords :parent))) (list leg (case leg (:rleg :lleg) (:lleg :rleg)))))))
      (let ((tc (list (send prev-foot-coords :copy-worldcoords)
                      (case leg
                        (:rleg (send *robot* :lleg :end-coords :copy-worldcoords))
                        (:lleg (send *robot* :rleg :end-coords :copy-worldcoords))))))
        (walking-pose *robot* :fix-coords (orient-coords-to-axis (apply #'midcoords 0.5 tc) (float-vector 0 0 1)))
        (tmp-fullbody-ik tc (apply #'midpoint 0.5 (send-all tc :worldpos)))
        (move-base-up-until-ik-fail :target-coords tc
                                    :target-centroid-pos (apply #'midpoint 0.5 (send-all tc :worldpos))
                                    :limbs (list leg (case leg (:rleg :lleg) (:lleg :rleg))))
        (push (list :angle-vector (send *robot* :angle-vector) :root-coords (send (car (send *robot* :links)) :copy-worldcoords) :time 0.0) rs-list))
      ;;(load "~/prog/euslib/rbrain/virtual_interpolator.l")
      (let ((tc (list (make-coords :pos
                                   (float-vector (* 0.5 (+ (elt (send prev-foot-coords :worldpos) 0) (elt (send next-foot-coords :worldpos) 0)))
                                                 (* 0.5 (+ (elt (send prev-foot-coords :worldpos) 1) (elt (send next-foot-coords :worldpos) 1)))
                                                 (+ (max (elt (send prev-foot-coords :worldpos) 2) (elt (send next-foot-coords :worldpos) 2))
                                                    default-step-height))
                                   :rot (midrot 0.5 (send prev-foot-coords :worldrot) (send next-foot-coords :worldrot)))
                      (case leg
                        (:rleg (send *robot* :lleg :end-coords :copy-worldcoords))
                        (:lleg (send *robot* :rleg :end-coords :copy-worldcoords))))))
        (walking-pose *robot* :fix-coords (orient-coords-to-axis (apply #'midcoords 0.5 tc) (float-vector 0 0 1)))
        (tmp-fullbody-ik tc (apply #'midpoint 0.9 (send-all tc :worldpos)))
        (move-base-up-until-ik-fail :target-coords tc
                                    :target-centroid-pos (apply #'midpoint 0.9 (send-all tc :worldpos))
                                    :limbs (list leg (case leg (:rleg :lleg) (:lleg :rleg))))
        (push (list :angle-vector (send *robot* :angle-vector) :root-coords (send (car (send *robot* :links)) :copy-worldcoords) :time default-step-time) rs-list))
      (let ((tc (list (send next-foot-coords :copy-worldcoords)
                      (case leg
                        (:rleg (send *robot* :lleg :end-coords :copy-worldcoords))
                        (:lleg (send *robot* :rleg :end-coords :copy-worldcoords))))))
        (walking-pose *robot* :fix-coords (orient-coords-to-axis (apply #'midcoords 0.5 tc) (float-vector 0 0 1)))
        (tmp-fullbody-ik tc (apply #'midpoint 0.5 (send-all tc :worldpos)))
        (move-base-up-until-ik-fail :target-coords tc
                                    :target-centroid-pos (apply #'midpoint 0.5 (send-all tc :worldpos))
                                    :limbs (list leg (case leg (:rleg :lleg) (:lleg :rleg))))
        (push (list :angle-vector (send *robot* :angle-vector) :root-coords (send (car (send *robot* :links)) :copy-worldcoords) :time (* 2 default-step-time)) rs-list))
      ;;(dolist (rs *rs-list*) (send *robot* :angle-vector (cadr (memq :angle-vector rs))) (send *robot* :move-coords (cadr (memq :root-coords rs)) (car (send *robot* :links))) (send *irtviewer* :draw-objects) (read-line))
      (reverse rs-list)
      )))

(defun project-sole-on-faces
  (leg p-faces org-coords)
  (set (read-from-string (format nil "*prev-~A-coords*" (string-downcase leg)))
       (send (eval (read-from-string (format nil "*~A-sole*" (string-downcase leg)))) :copy-worldcoords))
  (let ((crds (car (project-coords-on-to-plane
                    org-coords p-faces (float-vector 0 0 1)))))
    (if crds
        (send (eval (read-from-string (format nil "*~A-sole*" (string-downcase leg)))) :newcoords crds)
      (error ";; Cannot snap ~A~%" leg))
    ))

(defun test-walk-with-auto-base-height-tune
  ()
  ;;(progn (load "test-drc-terrain-walk.l") (walking-pose *robot*) (setq *rs-list* (check-leg-ik (send *robot* :rleg :end-coords :copy-worldcoords) *rleg-sole* :rleg 30 1.3)))
  (project-sole-on-faces
   :rleg *terrain-faces* (send (make-coords :pos (float-vector 0 -105 0)) :translate (float-vector 280 0 300) :world))
  (let ((sh 30) (st 1.3))
    (send *ri* :set-gait-generator-param :default-step-height (* 1e-3 sh) :default-step-time st)
    (setq *rs-list* (check-leg-reachability-for-one-step *prev-rleg-coords* *rleg-sole* :rleg sh st)))
  (dump-seq-pattern-file *rs-list* "/tmp/hoge" :initial-sync-time 0.0)
  (send *rI* :load-pattern "/tmp/hoge")
  (send *ri* :set-foot-steps
        (list (make-coords :coords (send *lleg-sole* :copy-worldcoords) :name :lleg)
              (make-coords :coords (send *rleg-sole* :copy-worldcoords) :name :rleg)))
  (project-sole-on-faces
   :lleg *terrain-faces* (send (make-coords :pos (float-vector 0 105 0)) :translate (float-vector 280 0 300) :world))
  (let ((sh 30) (st 1.3))
    (send *ri* :set-gait-generator-param :default-step-height (* 1e-3 sh) :default-step-time st)
    (setq *rs-list* (check-leg-reachability-for-one-step *prev-lleg-coords* *lleg-sole* :lleg sh st)))
  (dump-seq-pattern-file *rs-list* "/tmp/hoge" :initial-sync-time 0.0)
  (send *rI* :load-pattern "/tmp/hoge")
  (send *ri* :set-foot-steps
        (list (make-coords :coords (send *rleg-sole* :copy-worldcoords) :name :rleg)
              (make-coords :coords (send *lleg-sole* :copy-worldcoords) :name :lleg)
              ))
  )

;; (pprint (mapcar #'(lambda (s) (if (derivedp s hrpsys_ros_bridge::openhrp_autobalancerservice_footstep) (send *ri* :abc-footstep->eus-footstep s))) (mapcar #'cdr (send (send (send *ri* :autobalancerservice_getfootstepparam) :i_param) :slots))))
#|
(send *ri* :set-foot-steps
      (list (make-coords :pos (send (send (send (send (car (send *block* :bodies)) :copy-worldcoords) :rotate -pi/2 :z) :translate (scale -2 (cadr (memq :default-half-offset (send *robot* :get :footstep-parameter))))) :worldpos) :name :rleg)
      (make-coords :coords (send (send (car (send *block* :bodies)) :copy-worldcoords) :rotate -pi/2 :z) :name :lleg)
      (make-coords :coords (send (send (send (car (send *block* :bodies)) :copy-worldcoords) :rotate -pi/2 :z) :translate (scale -2 (cadr (memq :default-half-offset (send *robot* :get :footstep-parameter))))) :name :rleg)))


(send *ri* :set-foot-steps
      (list (make-coords :coords (send (send (car (send *block* :bodies)) :copy-worldcoords) :rotate -pi/2 :z) :name :lleg)
            (make-coords :coords (send (send (send (car (send *block* :bodies)) :copy-worldcoords) :rotate -pi/2 :z) :translate (scale -2 (cadr (memq :default-half-offset (send *robot* :get :footstep-parameter))))) :name :rleg)
            (make-coords :coords (send (send (car (send *block* :bodies)) :copy-worldcoords) :rotate -pi/2 :z) :name :lleg)))


(send *rI* :set-foot-steps
      (list
      (make-coords :coords (send (send (send (car (send *block* :bodies)) :copy-worldcoords) :rotate -pi/2 :z) :translate (scale -2 (cadr (memq :default-half-offset (send *robot* :get :footstep-parameter))))) :name :rleg)
      (make-coords :pos (send (send (send (send (car (send *block* :bodies)) :copy-worldcoords) :rotate -pi/2 :z) :translate (scale 0 (cadr (memq :default-half-offset (send *robot* :get :footstep-parameter))))) :worldpos) :name :lleg)
      (make-coords :pos (v+ (send (send (send (send (car (send *block* :bodies)) :copy-worldcoords) :rotate -pi/2 :z) :translate (scale 0 (cadr (memq :default-half-offset (send *robot* :get :footstep-parameter))))) :worldpos) (scale -2 (cadr (memq :default-half-offset (send *robot* :get :footstep-parameter))))) :name :rleg)))
|#

#|
(send *ri* :set-foot-steps
       (list (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
             (make-coords :coords (send *robot* :lleg :end-coords :copy-worldcoords) :name :lleg)
             (make-coords :coords (send (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                              :rotate (deg2rad -14.5) :y)
                                        :translate (float-vector 100 0 0))
                          :name :rleg)
             (make-coords :coords (send (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                              :rotate (deg2rad -14.5) :y)
                                        :translate (float-vector 100 0 0))
                          :name :lleg)
             ))

(send *ri* :set-foot-steps
       (list (make-coords :coords (send (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                              :rotate (deg2rad -14.5) :y)
                                        :translate (float-vector 100 0 0))
                          :name :rleg)
             (make-coords :coords (send (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                              :rotate (deg2rad -14.5) :y)
                                        :translate (float-vector 250 0 0))
                          :name :lleg)
             (make-coords :coords (send (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                              :rotate (deg2rad -14.5) :y)
                                        :translate (float-vector 250 0 0))
                          :name :rleg)
             ))

(send *ri* :set-foot-steps
      (list
       (make-coords :coords (send (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                        :rotate (deg2rad -14.5) :y)
                                  :translate (float-vector 0 0 0))
                    :name :lleg)
       (make-coords :coords (send (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                        :rotate (deg2rad 0) :y)
                                  :translate (float-vector 0 0 0))
                    :name :rleg)
       (make-coords :coords (send (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                        :rotate (deg2rad 0) :y)
                                  :translate (float-vector 0 0 0))
                    :name :lleg)
       ))


(send *ri* :set-foot-steps
      (list (make-coords :coords (send *robot* :lleg :end-coords :copy-worldcoords) :name :lleg)
            (make-coords :coords (send (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                              :rotate (deg2rad -14.5) :y)
                                        :translate (float-vector 100 0 0))
                          :name :rleg)
             (make-coords :coords (send (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                              :rotate (deg2rad -14.5) :y)
                                        :translate (float-vector 250 0 0))
                          :name :lleg)
             (make-coords :coords (send (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                              :rotate (deg2rad -14.5) :y)
                                        :translate (float-vector 250 0 0))
                          :name :rleg)
             ))


(send *ri* :set-foot-steps
      (list (make-coords :coords (send (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 0 0 0))
                                              :rotate (deg2rad 14.5) :y)
                                        :translate (float-vector 0 0 0))
                          :name :rleg)
            (make-coords :coords (send (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 0 0 0))
                                              :rotate (deg2rad 14.5) :y)
                                        :translate (float-vector 250 0 0))
                          :name :lleg)
             (make-coords :coords (send (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 0 0 0))
                                              :rotate (deg2rad 14.5) :y)
                                        :translate (float-vector 250 0 0))
                          :name :rleg)
             ))

;;
(send *ri* :set-foot-steps
       (list (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
             (make-coords :coords (send *robot* :lleg :end-coords :copy-worldcoords) :name :lleg)
             (make-coords :coords (send (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                              :rotate (deg2rad -14.5) :y)
                                        :translate (float-vector 100 0 0))
                          :name :rleg)
             (make-coords :coords (send (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                              :rotate (deg2rad -14.5) :y)
                                        :translate (float-vector 200 0 0))
                          :name :lleg)
             (make-coords :coords (send (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                              :rotate (deg2rad -14.5) :y)
                                        :translate (float-vector 300 0 0))
                          :name :rleg)
             (make-coords :coords (send (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                              :rotate (deg2rad -14.5) :y)
                                        :translate (float-vector 400 0 0))
                          :name :lleg)
             (make-coords :coords (send (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                              :rotate (deg2rad -14.5) :y)
                                        :translate (float-vector 500 0 0))
                          :name :rleg)
             (make-coords :coords (send (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                              :rotate (deg2rad -14.5) :y)
                                        :translate (float-vector 600 0 0))
                          :name :lleg)
             (make-coords :coords (send (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                              :rotate (deg2rad -14.5) :y)
                                        :translate (float-vector 700 0 0))
                          :name :rleg)
             (make-coords :coords (send (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                              :rotate (deg2rad -14.5) :y)
                                        :translate (float-vector 750 0 0))
                          :name :lleg)
             (make-coords :coords (send (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))
                                              :rotate (deg2rad -14.5) :y)
                                        :translate (float-vector 750 0 0))
                          :name :rleg)))
|#