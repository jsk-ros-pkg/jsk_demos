(load "package://drc_task_common/euslisp/drc-testbed-models.l")
(load "package://jsk_footstep_planner/euslisp/footstep_planner_util.l")

;;;;;;;;;;;;;;;;
;; utilities
;;;;;;;;;;;;;;;;
(defun walking-pose ;; this should be method
  (robot
   &key (root-link-height-offset 0)
        (root-link-pitch-offset 0)
        (fix-coords (make-coords))
        (default-pose-method :reset-manip-pose))
  "Generate and set walking pose.
   default-pose-method is initial pose, reset-manip-pose by default.
   Generated pose is near from default-pose-method pose.
   root-link-height-offset is root height offset [mm] from default-pose-method.
   root-link-pitch-offset is root pitch offset [deg] from default-pose-method."
  (send robot default-pose-method)
  (send robot :fix-leg-to-coords fix-coords)
  (let ((lc (mapcar #'(lambda (l)
                        (send robot l :end-coords :copy-worldcoords))
                    '(:rleg :lleg))))
    (send robot :move-coords
          (send
           (send (send (car (send robot :links)) :copy-worldcoords)
                 :translate (float-vector 0 0 root-link-height-offset))
           :rotate (deg2rad root-link-pitch-offset) :y)
          (car (send robot :links)))
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

(warn ";; (setup-terrain-walk-real-hrp2jsknts)~%")
(warn ";; (setup-terrain-walk-real-hrp2jsknt)~%")

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
;;   $ rtmtest -t hrpsys_tools _gen_project.launch INPUT:=${CVSDIR}/OpenHRP/etc/HRP2JSKNT_for_OpenHRP3/HRP2JSKNTmain.wrl OUTPUT:=/tmp/HRP2JSKNT_for_DRCTestbedBlock.xml OBJECT_MODELS:="`rospack find hrpsys`/share/hrpsys/samples/environments/DRCTestbedTerrainJPBlock.wrl,0.4,-2.0,0,1,0,0,0, `rospack find openhrp3`/share/OpenHRP-3.1/sample/model/longfloor.wrl,0,0,0,1,0,0,0" CORBA_PORT:=15005 INTEGRATE:=false
;;   $ rtmlaunch hrpsys_ros_bridge_tutorials hrp2jsknt.launch PROJECT_FILE:=/tmp/HRP2JSKNT_for_DRCTestbedBlock.xml

(defun setup-drc-testbed-terrain-simulation-hrp2jsknt ()
  (unless (boundp '*terrain*) (make-drc-terrain-japanese-block-ver))
  (setup-terrain-walk-hrp2jsknt)
  (setup-terrain-walk-real-hrp2jsknts)
  (walking-pose *robot* :root-link-height-offset -70) ;; <= -70mm
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *terrain* :newcoords (make-coords :pos (float-vector 400 -2000 0)))
  (setq *rleg-sole* (make-sole-body *robot* :rleg))
  (setq *lleg-sole* (make-sole-body *robot* :lleg))
  (setq *terrain-faces* (send *terrain* :get-all-terrain-top-faces))
  (objects (list *robot* *rleg-sole* *lleg-sole* *terrain*))
  )

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

(defun test-drc-testbed-terrain-simulation-hrp2jsknt ()
  (setup-drc-testbed-terrain-simulation-hrp2jsknt)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one 280 280)
  (test-drc-testbed-terrain-simulation-hrp2jsknt-one 400 400)
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