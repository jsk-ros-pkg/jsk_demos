(load "package://drc_task_common/euslisp/irex-large-box/apc-box-model.l")
(load "package://drc_task_common/euslisp/irex-large-box/sift-box.l") ;; also generate *apc-box*

(defun make-jaxon-ri ()
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
)

(defun start-auto-balancer-no-fix ()
  (send *ri* :start-auto-balancer :limbs '(:rleg :lleg :rarm :larm))
  (send *ri* :set-auto-balancer-param :is-hand-fix-mode nil)
  )

(defun jaxon-apc-box-init (&key (box-pos (float-vector 850 0 0)))
  ;; (make-apc-box)
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *apc-box* :newcoords (make-coords :pos box-pos))
  (objects (list *robot* *apc-box*))
  (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) 5000) ?~%")
  (when (y-or-n-p)
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    )
  ;; (warn ";; (send *ri* :start-auto-balancer) ?~%")
  ;; (when (y-or-n-p)
  ;;   ;;(send *ri* :start-auto-balancer)
  ;;   ;;(send *ri* :stop-auto-balancer)
  ;;   (send *ri* :start-auto-balancer :limbs '(:rleg :lleg :rarm :larm))
  ;;   )
  ;; (warn ";; (send *ri* :start-st) ?~%")
  ;; (when (y-or-n-p)
  ;;   (send *ri* :start-st)
  ;;   )
  (warn ";; (send *ri* :start-impedance :arms) ?~%")
  (when (y-or-n-p)
    (send *ri* :start-impedance :arms)
    ;;(send *ri* :start-impedance :arms :k-p 3000 :d-p 600)
    )
  )

;; (defun hrp2jsk-apc-box-init (&key (box-pos (float-vector 650 0 0)))
;;   (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
;;   (hrp2jsk-init)
;;   (setq *robot* *hrp2jsk*)
;;   ;; (make-apc-box)
;;   (send *robot* :reset-manip-pose)
;;   (send *robot* :fix-leg-to-coords (make-coords))
;;   (send *apc-box* :newcoords (make-coords :pos box-pos))
;;   (objects (list *robot* *apc-box*))
;;   (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector)) ?~%")
;;   (when (y-or-n-p)
;;     (send *ri* :angle-vector (send *robot* :angle-vector))
;;     (send *ri* :wait-interpolation)
;;     )
;;   (warn ";; (send *ri* :start-auto-balancer) ?~%")
;;   (when (y-or-n-p)
;;     (send *ri* :start-auto-balancer)
;;     )
;;   (warn ";; (send *ri* :start-st) ?~%")
;;   (when (y-or-n-p)
;;     (send *ri* :start-st)
;;     )
;;   (warn ";; (send *ri* :start-impedance :arms) ?~%")
;;   (when (y-or-n-p)
;;     (send *ri* :start-impedance :arms)
;;     ;;(send *ri* :start-impedance :arms :k-p 3000 :d-p 600)
;;     )
;;   )

;; (defun hrp2jsknt-apc-box-init (&key (box-pos (float-vector 650 0 0)))
;;   (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
;;   (hrp2jsknt-init)
;;   (setq *robot* *hrp2jsknt*)
;;   ;; (make-apc-box)
;;   (send *robot* :reset-manip-pose)
;;   (send *robot* :fix-leg-to-coords (make-coords))
;;   (send *apc-box* :newcoords (make-coords :pos box-pos))
;;   (objects (list *robot* *apc-box*))
;;   (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector)) ?~%")
;;   (when (y-or-n-p)
;;     (send *ri* :angle-vector (send *robot* :angle-vector))
;;     (send *ri* :wait-interpolation)
;;     )
;;   (warn ";; (send *ri* :start-auto-balancer) ?~%")
;;   (when (y-or-n-p)
;;     (send *ri* :start-auto-balancer)
;;     )
;;   (warn ";; (send *ri* :start-st) ?~%")
;;   (when (y-or-n-p)
;;     (send *ri* :start-st)
;;     )
;;   (warn ";; (send *ri* :start-impedance :arms) ?~%")
;;   (when (y-or-n-p)
;;     (send *ri* :start-impedance :arms)
;;     ;;(send *ri* :start-impedance :arms :k-p 3000 :d-p 600)
;;     )
;;   )

(defun project-coords-to-ground (coords)
  (let* ((px (elt (send coords :worldpos) 0))
         (py (elt (send coords :worldpos) 1))
         (yaw (elt (elt (send coords :rpy-angle) 0) 0))
         )
    (make-coords :pos (float-vector px py 0) :angle yaw :axis :z)
    ))

;; (defun start-calib-box-pos (&key (box *apc-box*) (set-box-x-pos 650))
(defun start-calib-box-pos (&key (box *apc-box*) (set-box-x-pos 850))
  (let* ((orig-box-coords (project-coords-to-ground (send box :copy-worldcoords)))
         (orig-box-x (elt (send orig-box-coords :worldpos) 0))
         (orig-box-y (elt (send orig-box-coords :worldpos) 1))
         (orig-box-yaw (elt (elt (send orig-box-coords :rpy-angle) 0) 0))
         x-calib-offset y-calib-offset yaw-calib-offset)
    (setq x-calib-offset (- set-box-x-pos orig-box-x))
    (setq y-calib-offset (- 0 orig-box-y))
    (setq yaw-calib-offset (- 0 orig-box-yaw))
    (setq *calib-offset-coords* (make-coords :pos (float-vector x-calib-offset y-calib-offset 0) :angle yaw-calib-offset :axis :z))
    (warn ";; set-box-x-pos = ~a[mm]~%" set-box-x-pos)
    (warn ";; x-calib-offset = ~a[mm]~%" x-calib-offset)
    (warn ";; y-calib-offset = ~a[mm]~%" y-calib-offset)
    (warn ";; yaw-calib-offset = ~a[rad]~%" yaw-calib-offset)
    (warn ";; *calib-offset-coords* = ~a~%" *calib-offset-coords*)
    (send *apc-box* :newcoords (send (send orig-box-coords :copy-worldcoords) :transform *calib-offset-coords*))
    (send *irtviewer* :draw-objects)
    ))

(defun apply-box-pose (&key (calib-offset nil))
  (let* ((coords (get-primitive-marker-pose :name "apc_box" :frame-id "ground"))
         (box-coords (project-coords-to-ground coords))
         )
    (if calib-offset
        (progn
          (unless (boundp '*calib-offset-coords*)
            (start-calib-box-pos)
            )
          (send *apc-box* :newcoords (send box-coords :transform *calib-offset-coords*))
          )
      (send *apc-box* :newcoords box-coords)
      )
    (send *irtviewer* :draw-objects)
    ))

(defun update-box-pose-by-sift (&key (calib-offset nil))
  (set-pose-10) ;; set box pose by sift recog
  (apply-box-pose :calib-offset calib-offset)
  )

(defun loop-update-box-pose (&key (calib-offset nil))
  (let* ((i 0))
    (do-until-key
     (warn "Loop: ~a~%" i)
     (update-box-pose-by-sift :calib-offset calib-offset)
     (unix::usleep 10000) ;; 0.01sec
     (incf i)
     )
    )
  )

(defun jaxon-maai-hosei-by-go-pos (box-instance &key (ref-box-pos (float-vector 850 0 0)) (gp-stop-thre 1.5) (calib-offset t))
  (let* (proj-coords maai-coords gp-x gp-y gp-yaw)
    (setq proj-coords (project-coords-to-ground (send box-instance :copy-worldcoords)))
    (setq maai-coords (send proj-coords :translate (v- ref-box-pos) :local))
    (setq gp-x-m (* 0.001 (elt (send maai-coords :worldpos) 0)))
    (setq gp-y-m (* 0.001 (elt (send maai-coords :worldpos) 1)))
    (setq gp-yaw-deg (rad2deg (elt (elt (send maai-coords :rpy-angle) 0) 0)))
    (if (> (norm (float-vector gp-x-m gp-y-m)) gp-stop-thre)
        (progn
          (warn "Emergency: too far to go-pos !!!~%")
          (return-from jaxon-maai-hosei-by-go-pos)
          ))
    (warn ";; (send *ri* :go-pos ~a ~a ~a) OK?~%" gp-x-m gp-y-m gp-yaw-deg)
    (when (y-or-n-p)
      (send *ri* :go-pos gp-x-m gp-y-m gp-yaw-deg)
      (send *ri* :wait-interpolation)
      (update-box-pose-by-sift :calib-offset calib-offset)
      )
    ))

;; (defun hrp2jsk-maai-hosei-by-go-pos (box-instance &key (ref-box-pos (float-vector 650 0 0)) (gp-stop-thre 1.0) (calib-offset t))
;;   (let* (proj-coords maai-coords gp-x gp-y gp-yaw)
;;     (setq proj-coords (project-coords-to-ground (send box-instance :copy-worldcoords)))
;;     (setq maai-coords (send proj-coords :translate (v- ref-box-pos) :local))
;;     (setq gp-x-m (* 0.001 (elt (send maai-coords :worldpos) 0)))
;;     (setq gp-y-m (* 0.001 (elt (send maai-coords :worldpos) 1)))
;;     (setq gp-yaw-deg (rad2deg (elt (elt (send maai-coords :rpy-angle) 0) 0)))
;;     (if (> (norm (float-vector gp-x-m gp-y-m)) gp-stop-thre)
;;         (progn
;;           (warn "Emergency: too far to go-pos !!!~%")
;;           (return-from hrp2jsk-maai-hosei-by-go-pos)
;;           ))
;;     (warn ";; (send *ri* :go-pos ~a ~a ~a) OK?~%" gp-x-m gp-y-m gp-yaw-deg)
;;     (when (y-or-n-p)
;;       (send *ri* :go-pos gp-x-m gp-y-m gp-yaw-deg)
;;       (send *ri* :wait-interpolation)
;;       (update-box-pose-by-sift :calib-offset calib-offset)
;;       )
;;     ))

;; (defun hrp2jsknt-maai-hosei-by-go-pos (box-instance &key (ref-box-pos (float-vector 650 0 0)) (gp-stop-thre 1.0) (calib-offset t))
;;   (let* (proj-coords maai-coords gp-x gp-y gp-yaw)
;;     (setq proj-coords (project-coords-to-ground (send box-instance :copy-worldcoords)))
;;     (setq maai-coords (send proj-coords :translate (v- ref-box-pos) :local))
;;     (setq gp-x-m (* 0.001 (elt (send maai-coords :worldpos) 0)))
;;     (setq gp-y-m (* 0.001 (elt (send maai-coords :worldpos) 1)))
;;     (setq gp-yaw-deg (rad2deg (elt (elt (send maai-coords :rpy-angle) 0) 0)))
;;     (if (> (norm (float-vector gp-x-m gp-y-m)) gp-stop-thre)
;;         (progn
;;           (warn "Emergency: too far to go-pos !!!~%")
;;           (return-from hrp2jsknt-maai-hosei-by-go-pos)
;;           ))
;;     (warn ";; (send *ri* :go-pos ~a ~a ~a) OK?~%" gp-x-m gp-y-m gp-yaw-deg)
;;     (when (y-or-n-p)
;;       (send *ri* :go-pos gp-x-m gp-y-m gp-yaw-deg)
;;       (send *ri* :wait-interpolation)
;;       (update-box-pose-by-sift :calib-offset calib-offset)
;;       )
;;     ))

(defun walking-pose
  (robot
   &key (root-link-height-offset 0)
   (root-link-pitch-offset 0)
   (head-link-pitch-offset 0)
   (root-link-roll-offset 0)
   (chest-link-pitch-offset 0)
   (chest-link-roll-offset 0)
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
    (if (find-method robot :torso-waist-p) (send robot :torso-waist-p :joint-angle chest-link-pitch-offset))
    (if (find-method robot :torso-waist-r) (send robot :torso-waist-r :joint-angle chest-link-roll-offset))
    (mapcar #'(lambda (l c)
                (send robot l :inverse-kinematics c))
            '(:rleg :lleg) lc)
    (send robot :move-centroid-on-foot :both '(:rleg :lleg))
    (if (find-method robot :head-neck-p) (send robot :head-neck-p :joint-angle head-link-pitch-offset))
    (send robot :angle-vector)
    ))

;; (defun jaxon-apc-box-reach (&key (mid-reach-pos (float-vector -100 0 100))
;;                                    (grasp-reach-pos (float-vector 40 0 30))
;;                                    (dt 8000) (rlpo 0) (hlpo 30) (back-mode t))
;;   (let* (avs)
;;     (send *robot* :reset-manip-pose)
;;     (push (send *robot* :angle-vector) avs)
;;   ;; move to pose-1
;;   (walking-pose *robot* :root-link-pitch-offset rlpo :head-link-pitch-offset hlpo)
;;   ;; (send *robot* :rarm-thumb-r :joint-angle 90.0)
;;   ;; (send *robot* :larm-thumb-r :joint-angle -90.0)
;;   (send *robot*
;;         :fullbody-inverse-kinematics
;;         (list
;;          (send (send (send *apc-box* :top-front-right-grasp-edge) :copy-worldcoords) :translate mid-reach-pos :world)
;;          (send (send (send *apc-box* :top-front-left-grasp-edge) :copy-worldcoords) :translate mid-reach-pos :world)
;;          (send *robot* :rleg :end-coords :copy-worldcoords)
;;          (send *robot* :lleg :end-coords :copy-worldcoords))
;;         :move-target
;;         (list
;;          (send *robot* :rarm :end-coords)
;;          (send *robot* :larm :end-coords)
;;          (send *robot* :rleg :end-coords)
;;          (send *robot* :lleg :end-coords))
;;         :link-list
;;         (mapcar #'(lambda (limb)
;;                     (send *robot* :link-list (send limb :parent)))
;;                 (list
;;                  (send *robot* :rarm :end-coords)
;;                  (send *robot* :larm :end-coords)
;;                  (send *robot* :rleg :end-coords)
;;                  (send *robot* :lleg :end-coords))
;;                 )
;;         :translation-axis (list t t t t)
;;         :rotation-axis (list :z :z t t)
;;         :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
;;         :debug-view :no-message)
;;   (send *irtviewer* :draw-objects)
;;   (push (send *robot* :angle-vector) avs)
;;   (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) ~a) ?~%" dt)
;;   (when (y-or-n-p)
;;     (send *ri* :angle-vector (send *robot* :angle-vector) dt)
;;     (send *ri* :wait-interpolation)
;;     )
;;   (warn ";; move to pose-reach-end ?~%")
;;   (unless (y-or-n-p)
;;     (return-from jaxon-apc-box-reach)
;;     )
;;   ;; move to pose-reach-end
;;   (walking-pose *robot* :root-link-pitch-offset rlpo :head-link-pitch-offset hlpo)
;;   ;; (send *robot* :rarm-thumb-r :joint-angle 90.0)
;;   ;; (send *robot* :larm-thumb-r :joint-angle -90.0)
;;   (send *robot*
;;         :fullbody-inverse-kinematics
;;         (list
;;          (send (send (send (send *apc-box* :top-front-right-grasp-edge) :copy-worldcoords) :translate grasp-reach-pos :world) :rotate (deg2rad 90) :z)
;;          (send (send (send (send *apc-box* :top-front-left-grasp-edge) :copy-worldcoords) :translate grasp-reach-pos :world) :rotate (deg2rad -90) :z)
;;          (send *robot* :rleg :end-coords :copy-worldcoords)
;;          (send *robot* :lleg :end-coords :copy-worldcoords))
;;         :move-target
;;         (list
;;          (send *robot* :rarm :end-coords)
;;          (send *robot* :larm :end-coords)
;;          (send *robot* :rleg :end-coords)
;;          (send *robot* :lleg :end-coords))
;;         :link-list
;;         (mapcar #'(lambda (limb)
;;                     (send *robot* :link-list (send limb :parent)))
;;                 (list
;;                  (send *robot* :rarm :end-coords)
;;                  (send *robot* :larm :end-coords)
;;                  (send *robot* :rleg :end-coords)
;;                  (send *robot* :lleg :end-coords))
;;                 )
;;         :translation-axis (list t t t t)
;;         :rotation-axis (list t t t t)
;;         :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
;;         :debug-view :no-message)
;;   (send *irtviewer* :draw-objects)
;;   (push (send *robot* :angle-vector) avs)
;;  (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) ~a) ?~%" dt)
;;  (when (y-or-n-p)
;;     (send *ri* :angle-vector (send *robot* :angle-vector) dt)
;;     (send *ri* :wait-interpolation)
;;     )
;;  (when back-mode
;;    (warn ";; Back Mode !!!~%")
;;    (pop avs)
;;    (dolist (av avs)
;;      (warn ";; move to next ?~%")
;;      (unless (y-or-n-p)
;;        (return-from jaxon-apc-box-reach)
;;        )
;;      (send *robot* :angle-vector av)
;;      (send *robot* :fix-leg-to-coords (make-coords))
;;      (send *irtviewer* :draw-objects)
;;      (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) ~a) ?~%" dt)
;;      (when (y-or-n-p)
;;        (send *ri* :angle-vector (send *robot* :angle-vector) dt)
;;        (send *ri* :wait-interpolation)
;;        )
;;      )
;;    )
;;  ))

;; (defun hrp2jsk-apc-box-reach (&key (mid-reach-pos (float-vector -100 0 100))
;;                                    (grasp-reach-pos (float-vector 40 0 30))
;;                                    (dt 8000) (rlpo 10) (hlpo 30) (back-mode t))
;;   (let* (avs)
;;     (send *robot* :reset-manip-pose)
;;     (push (send *robot* :angle-vector) avs)
;;   ;; move to pose-1
;;   (walking-pose *robot* :root-link-pitch-offset rlpo :head-link-pitch-offset hlpo)
;;   (send *robot* :rarm-thumb-r :joint-angle 90.0)
;;   (send *robot* :larm-thumb-r :joint-angle -90.0)
;;   (send *robot*
;;         :fullbody-inverse-kinematics
;;         (list
;;          (send (send (send *apc-box* :top-front-right-grasp-edge) :copy-worldcoords) :translate mid-reach-pos :world)
;;          (send (send (send *apc-box* :top-front-left-grasp-edge) :copy-worldcoords) :translate mid-reach-pos :world)
;;          (send *robot* :rleg :end-coords :copy-worldcoords)
;;          (send *robot* :lleg :end-coords :copy-worldcoords))
;;         :move-target
;;         (list
;;          (send *robot* :rarm :end-coords)
;;          (send *robot* :larm :end-coords)
;;          (send *robot* :rleg :end-coords)
;;          (send *robot* :lleg :end-coords))
;;         :link-list
;;         (mapcar #'(lambda (limb)
;;                     (send *robot* :link-list (send limb :parent)))
;;                 (list
;;                  (send *robot* :rarm :end-coords)
;;                  (send *robot* :larm :end-coords)
;;                  (send *robot* :rleg :end-coords)
;;                  (send *robot* :lleg :end-coords))
;;                 )
;;         :translation-axis (list t t t t)
;;         :rotation-axis (list t t t t)
;;         :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
;;         :debug-view :no-message)
;;   (send *irtviewer* :draw-objects)
;;   (push (send *robot* :angle-vector) avs)
;;   (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) ~a) ?~%" dt)
;;   (when (y-or-n-p)
;;     (send *ri* :angle-vector (send *robot* :angle-vector) dt)
;;     (send *ri* :wait-interpolation)
;;     )
;;   (warn ";; move to pose-reach-end ?~%")
;;   (unless (y-or-n-p)
;;     (return-from hrp2jsk-apc-box-reach)
;;     )
;;   ;; move to pose-reach-end
;;   (walking-pose *robot* :root-link-pitch-offset rlpo :head-link-pitch-offset hlpo)
;;   (send *robot* :rarm-thumb-r :joint-angle 90.0)
;;   (send *robot* :larm-thumb-r :joint-angle -90.0)
;;   (send *robot*
;;         :fullbody-inverse-kinematics
;;         (list
;;          (send (send (send *apc-box* :top-front-right-grasp-edge) :copy-worldcoords) :translate grasp-reach-pos :world)
;;          (send (send (send *apc-box* :top-front-left-grasp-edge) :copy-worldcoords) :translate grasp-reach-pos :world)
;;          (send *robot* :rleg :end-coords :copy-worldcoords)
;;          (send *robot* :lleg :end-coords :copy-worldcoords))
;;         :move-target
;;         (list
;;          (send *robot* :rarm :end-coords)
;;          (send *robot* :larm :end-coords)
;;          (send *robot* :rleg :end-coords)
;;          (send *robot* :lleg :end-coords))
;;         :link-list
;;         (mapcar #'(lambda (limb)
;;                     (send *robot* :link-list (send limb :parent)))
;;                 (list
;;                  (send *robot* :rarm :end-coords)
;;                  (send *robot* :larm :end-coords)
;;                  (send *robot* :rleg :end-coords)
;;                  (send *robot* :lleg :end-coords))
;;                 )
;;         :translation-axis (list t t t t)
;;         :rotation-axis (list t t t t)
;;         :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
;;         :debug-view :no-message)
;;   (send *irtviewer* :draw-objects)
;;   (push (send *robot* :angle-vector) avs)
;;  (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) ~a) ?~%" dt)
;;  (when (y-or-n-p)
;;     (send *ri* :angle-vector (send *robot* :angle-vector) dt)
;;     (send *ri* :wait-interpolation)
;;     )
;;  (when back-mode
;;    (warn ";; Back Mode !!!~%")
;;    (pop avs)
;;    (dolist (av avs)
;;      (warn ";; move to next ?~%")
;;      (unless (y-or-n-p)
;;        (return-from hrp2jsk-apc-box-reach)
;;        )
;;      (send *robot* :angle-vector av)
;;      (send *robot* :fix-leg-to-coords (make-coords))
;;      (send *irtviewer* :draw-objects)
;;      (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) ~a) ?~%" dt)
;;      (when (y-or-n-p)
;;        (send *ri* :angle-vector (send *robot* :angle-vector) dt)
;;        (send *ri* :wait-interpolation)
;;        )
;;      )
;;    )
;;  ))

;; (defun hrp2jsknt-apc-box-reach (&key (mid-reach-pos (float-vector -100 0 100))
;;                                    (grasp-reach-pos (float-vector -15 0 0))
;;                                    (dt 8000) (rlpo 15) (hlpo 30) (back-mode t))
;;   (let* (avs)
;;     (send *robot* :reset-manip-pose)
;;     (push (send *robot* :angle-vector) avs)
;;   ;; move to pose-1
;;   (walking-pose *robot* :root-link-pitch-offset rlpo :head-link-pitch-offset hlpo)
;;   ;; (send *robot* :rarm-thumb-r :joint-angle 90.0)
;;   ;; (send *robot* :larm-thumb-r :joint-angle -90.0)
;;   ;; (send *robot* :hand :arms :hook-pose)
;;   (send *robot* :hand :arms :angle-vector #f(90.0 85.0 0.0 0.0 -40.0 -40.0))
;;   (send *robot*
;;         :fullbody-inverse-kinematics
;;         (list
;;          (send (send (send *apc-box* :top-front-right-grasp-edge) :copy-worldcoords) :translate mid-reach-pos :world)
;;          (send (send (send *apc-box* :top-front-left-grasp-edge) :copy-worldcoords) :translate mid-reach-pos :world)
;;          (send *robot* :rleg :end-coords :copy-worldcoords)
;;          (send *robot* :lleg :end-coords :copy-worldcoords))
;;         :move-target
;;         (list
;;          (send *robot* :rarm :end-coords)
;;          (send *robot* :larm :end-coords)
;;          (send *robot* :rleg :end-coords)
;;          (send *robot* :lleg :end-coords))
;;         :link-list
;;         (mapcar #'(lambda (limb)
;;                     (send *robot* :link-list (send limb :parent)))
;;                 (list
;;                  (send *robot* :rarm :end-coords)
;;                  (send *robot* :larm :end-coords)
;;                  (send *robot* :rleg :end-coords)
;;                  (send *robot* :lleg :end-coords))
;;                 )
;;         :translation-axis (list t t t t)
;;         :rotation-axis (list t t t t)
;;         :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
;;         :debug-view :no-message)
;;   (send *irtviewer* :draw-objects)
;;   (push (send *robot* :angle-vector) avs)
;;   (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) ~a) ?~%" dt)
;;   (when (y-or-n-p)
;;     (send *ri* :angle-vector (send *robot* :angle-vector) dt)
;;     (send *ri* :wait-interpolation)
;;     )
;;   (warn ";; move to pose-reach-end ?~%")
;;   (unless (y-or-n-p)
;;     (return-from hrp2jsknt-apc-box-reach)
;;     )
;;   ;; move to pose-reach-end
;;   (walking-pose *robot* :root-link-pitch-offset rlpo :head-link-pitch-offset hlpo)
;;   ;; (send *robot* :rarm-thumb-r :joint-angle 90.0)
;;   ;; (send *robot* :larm-thumb-r :joint-angle -90.0)
;;   ;; (send *robot* :hand :arms :hook-pose)
;;   (send *robot* :hand :arms :angle-vector #f(90.0 85.0 0.0 0.0 -40.0 -40.0))
;;   (send *robot*
;;         :fullbody-inverse-kinematics
;;         (list
;;          (send (send (send *apc-box* :top-front-right-grasp-edge) :copy-worldcoords) :translate grasp-reach-pos :world)
;;          (send (send (send *apc-box* :top-front-left-grasp-edge) :copy-worldcoords) :translate grasp-reach-pos :world)
;;          (send *robot* :rleg :end-coords :copy-worldcoords)
;;          (send *robot* :lleg :end-coords :copy-worldcoords))
;;         :move-target
;;         (list
;;          (send *robot* :rarm :end-coords)
;;          (send *robot* :larm :end-coords)
;;          (send *robot* :rleg :end-coords)
;;          (send *robot* :lleg :end-coords))
;;         :link-list
;;         (mapcar #'(lambda (limb)
;;                     (send *robot* :link-list (send limb :parent)))
;;                 (list
;;                  (send *robot* :rarm :end-coords)
;;                  (send *robot* :larm :end-coords)
;;                  (send *robot* :rleg :end-coords)
;;                  (send *robot* :lleg :end-coords))
;;                 )
;;         :translation-axis (list t t t t)
;;         :rotation-axis (list t t t t)
;;         :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
;;         :debug-view :no-message)
;;   (send *irtviewer* :draw-objects)
;;   (push (send *robot* :angle-vector) avs)
;;  (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) ~a) ?~%" dt)
;;  (when (y-or-n-p)
;;     (send *ri* :angle-vector (send *robot* :angle-vector) dt)
;;     (send *ri* :wait-interpolation)
;;     )
;;  (when back-mode
;;    (warn ";; Back Mode !!!~%")
;;    (pop avs)
;;    (dolist (av avs)
;;      (warn ";; move to next ?~%")
;;      (unless (y-or-n-p)
;;        (return-from hrp2jsknt-apc-box-reach)
;;        )
;;      (send *robot* :angle-vector av)
;;      (send *robot* :fix-leg-to-coords (make-coords))
;;      (send *irtviewer* :draw-objects)
;;      (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) ~a) ?~%" dt)
;;      (when (y-or-n-p)
;;        (send *ri* :angle-vector (send *robot* :angle-vector) dt)
;;        (send *ri* :wait-interpolation)
;;        )
;;      )
;;    )
;;  ))

;; (defun torque-ratio-vector ()
;;   (let* ()
;;     (setq torque-vector
;;           (send *robot* :torque-vector
;;                 :force-list (list #f(0 0 700) #f(0 0 700) #f(0 0 100) #f(0 0 100))
;;                 :moment-list (list #f(0 0 0) #f(0 0 0) #f(0 0 0) #f(0 0 0))
;;                 :target-coords (append (send *robot* :legs :end-coords) (send *robot* :arms :end-coords))))
;;     (send *robot* :torque-ratio-vector :torque torque-vector)
;;     ))

(defun hrp2jsk-apc-box-okkake ()
  (update-box-pose-by-sift :calib-offset t)
  (hrp2jsk-maai-hosei-by-go-pos *apc-box*)
  )

(defun hrp2jsk-box-okkake-reaching-demo ()
  (warn ";; 1 kai~%")
  (hrp2jsk-apc-box-okkake)
  (unix::sleep 2)
  (warn ";; 2 kai~%")
  (hrp2jsk-apc-box-okkake)
  (unix::sleep 2)
  (warn ";; 3 kai~%")
  (hrp2jsk-apc-box-okkake)
  (unix::sleep 2)
  (warn ";; reaching~%")
  (hrp2jsk-apc-box-reach)
  )


;; Set ref-force for arms
;; (send *ri* :set-ref-force (float-vector hoge hoge hoge) 5000 :arms)
;; go-pos
;; (send *ri* :go-pos 0.1 0 0)

;; (defun push-fn-1 ()
;; (send *robot*
;;         :fullbody-inverse-kinematics
;;         (list
;;          (send (send (send (send *apc-box* :top-front-right-grasp-edge) :copy-worldcoords) :translate (float-vector 40 60 130) :world) :rotate (deg2rad 90) :z)
;;          (send (send (send (send *apc-box* :top-front-left-grasp-edge) :copy-worldcoords) :translate (float-vector 40 -60 130) :world) :rotate (deg2rad -90) :z)
;;          (send *robot* :rleg :end-coords :copy-worldcoords)
;;          (send *robot* :lleg :end-coords :copy-worldcoords))
;;         :move-target
;;         (list
;;          (send *robot* :rarm :end-coords)
;;          (send *robot* :larm :end-coords)
;;          (send *robot* :rleg :end-coords)
;;          (send *robot* :lleg :end-coords))
;;         :link-list
;;         (mapcar #'(lambda (limb)
;;                     (send *robot* :link-list (send limb :parent)))
;;                 (list
;;                  (send *robot* :rarm :end-coords)
;;                  (send *robot* :larm :end-coords)
;;                  (send *robot* :rleg :end-coords)
;;                  (send *robot* :lleg :end-coords))
;;                 )
;;         :translation-axis (list t t t t)
;;         :rotation-axis (list t t t t)
;;         :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
;;         :debug-view :no-message)
;; )


;; hako wo oku


;; (send *robot*
;;         :fullbody-inverse-kinematics
;;         (list
;;          (send (send (send (send *apc-box* :top-front-right-grasp-edge) :copy-worldcoords) :translate (float-vector 40 60 (- 130 50 20)) :world) :rotate (deg2rad 90) :z)
;;          (send (send (send (send *apc-box* :top-front-left-grasp-edge) :copy-worldcoords) :translate (float-vector 40 -60 (- 130 50 20)) :world) :rotate (deg2rad -90) :z)
;;          (send *robot* :rleg :end-coords :copy-worldcoords)
;;          (send *robot* :lleg :end-coords :copy-worldcoords))
;;         :move-target
;;         (list
;;          (send *robot* :rarm :end-coords)
;;          (send *robot* :larm :end-coords)
;;          (send *robot* :rleg :end-coords)
;;          (send *robot* :lleg :end-coords))
;;         :link-list
;;         (mapcar #'(lambda (limb)
;;                     (send *robot* :link-list (send limb :parent)))
;;                 (list
;;                  (send *robot* :rarm :end-coords)
;;                  (send *robot* :larm :end-coords)
;;                  (send *robot* :rleg :end-coords)
;;                  (send *robot* :lleg :end-coords))
;;                 )
;;         :translation-axis (list t t t t)
;;         :rotation-axis (list t t t t)
;;         :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
;;         :debug-view :no-message)




;; (send *robot*
;;         :fullbody-inverse-kinematics
;;         (list
;;          (send (send (send (send *apc-box* :top-front-right-grasp-edge) :copy-worldcoords) :translate (float-vector 40 60 (- 130 50 20)) :world) :rotate (deg2rad 80) :z)
;;          (send (send (send (send *apc-box* :top-front-left-grasp-edge) :copy-worldcoords) :translate (float-vector 40 -60 (- 130 50 20)) :world) :rotate (deg2rad -80) :z)
;;          (send *robot* :rleg :end-coords :copy-worldcoords)
;;          (send *robot* :lleg :end-coords :copy-worldcoords))
;;         :move-target
;;         (list
;;          (send *robot* :rarm :end-coords)
;;          (send *robot* :larm :end-coords)
;;          (send *robot* :rleg :end-coords)
;;          (send *robot* :lleg :end-coords))
;;         :link-list
;;         (mapcar #'(lambda (limb)
;;                     (send *robot* :link-list (send limb :parent)))
;;                 (list
;;                  (send *robot* :rarm :end-coords)
;;                  (send *robot* :larm :end-coords)
;;                  (send *robot* :rleg :end-coords)
;;                  (send *robot* :lleg :end-coords))
;;                 )
;;         :translation-axis (list t t t t)
;;         :rotation-axis (list t t t t)
;;         :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
;;         :debug-view :no-message)




;; (defun push-fn-2 ()
;; (send *robot*
;;         :fullbody-inverse-kinematics
;;         (list
;;          (send (send (send (send *apc-box* :top-front-right-grasp-edge) :copy-worldcoords) :translate (float-vector 40 60 (- 130 50 20 10 5)) :world) :rotate (deg2rad 75) :z)
;;          (send (send (send (send *apc-box* :top-front-left-grasp-edge) :copy-worldcoords) :translate (float-vector 40 -60 (- 130 50 20 10 5)) :world) :rotate (deg2rad -75) :z)
;;          (send *robot* :rleg :end-coords :copy-worldcoords)
;;          (send *robot* :lleg :end-coords :copy-worldcoords))
;;         :move-target
;;         (list
;;          (send *robot* :rarm :end-coords)
;;          (send *robot* :larm :end-coords)
;;          (send *robot* :rleg :end-coords)
;;          (send *robot* :lleg :end-coords))
;;         :link-list
;;         (mapcar #'(lambda (limb)
;;                     (send *robot* :link-list (send limb :parent)))
;;                 (list
;;                  (send *robot* :rarm :end-coords)
;;                  (send *robot* :larm :end-coords)
;;                  (send *robot* :rleg :end-coords)
;;                  (send *robot* :lleg :end-coords))
;;                 )
;;         :translation-axis (list t t t t)
;;         :rotation-axis (list t t t t)
;;         :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
;;         :debug-view :no-message)
;; )



;; (defun push-fn-3 ()
;; (send *robot*
;;         :fullbody-inverse-kinematics
;;         (list
;;          (send (send (send (send *apc-box* :top-front-right-grasp-edge) :copy-worldcoords) :translate (float-vector (+ 40 30 20) 60 (- 130 50 20 10 5)) :world) :rotate (deg2rad 75) :z)
;;          (send (send (send (send *apc-box* :top-front-left-grasp-edge) :copy-worldcoords) :translate (float-vector (+ 40 30 20) -60 (- 130 50 20 10 5)) :world) :rotate (deg2rad -75) :z)
;;          (send *robot* :rleg :end-coords :copy-worldcoords)
;;          (send *robot* :lleg :end-coords :copy-worldcoords))
;;         :move-target
;;         (list
;;          (send *robot* :rarm :end-coords)
;;          (send *robot* :larm :end-coords)
;;          (send *robot* :rleg :end-coords)
;;          (send *robot* :lleg :end-coords))
;;         :link-list
;;         (mapcar #'(lambda (limb)
;;                     (send *robot* :link-list (send limb :parent)))
;;                 (list
;;                  (send *robot* :rarm :end-coords)
;;                  (send *robot* :larm :end-coords)
;;                  (send *robot* :rleg :end-coords)
;;                  (send *robot* :lleg :end-coords))
;;                 )
;;         :translation-axis (list t t t t)
;;         :rotation-axis (list t t t t)
;;         :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
;;         :debug-view :no-message)
;; )


(defun jaxon-apc-box-reach-side (&key (mid1-reach-pos-r (float-vector -120 -50 -60)) (mid1-reach-pos-l (float-vector -120 50 -60))
                                      (mid2-reach-pos-r (float-vector 10 -50 -60)) (mid2-reach-pos-l (float-vector 10 50 -60))
                                      (grasp-reach-pos-r (float-vector 0 10 -60)) (grasp-reach-pos-l (float-vector 0 -10 -60))
                                      (hand-rotation 45) (box-pos (float-vector 850 0 0))
                                      (dt 8000) (rlpo 25) (hlpo 30) (back-mode t))
  ;; (send *apc-box* :locate (float-vector 600 0 0) :world)
  (send *apc-box* :newcoords (make-coords :pos box-pos))
  (let* (avs)
    (send *robot* :reset-manip-pose)
    (push (send *robot* :angle-vector) avs)
  ;; move to pose-1
  (walking-pose *robot* :root-link-pitch-offset rlpo :head-link-pitch-offset hlpo)
  ;; (send *robot* :rarm-thumb-r :joint-angle 90.0)
  ;; (send *robot* :larm-thumb-r :joint-angle -90.0)
  (send *robot*
        :fullbody-inverse-kinematics
        (list
         ;;(send (send (send (send *apc-box* :side-front-right-grasp-edge) :copy-worldcoords) :translate mid-reach-pos :world) :rotate (deg2rad 90) :z)
         ;;(send (send (send (send *apc-box* :side-front-left-grasp-edge) :copy-worldcoords) :translate mid-reach-pos :world) :rotate (deg2rad -90) :z)
         (send (send (send (send *apc-box* :side-front-right-grasp-edge) :copy-worldcoords) :translate mid1-reach-pos-r :world) :rotate (deg2rad hand-rotation) :z)
         (send (send (send (send *apc-box* :side-front-left-grasp-edge) :copy-worldcoords) :translate mid1-reach-pos-l :world) :rotate (deg2rad (- hand-rotation)) :z)
         (send *robot* :rleg :end-coords :copy-worldcoords)
         (send *robot* :lleg :end-coords :copy-worldcoords))
        :move-target
        (list
         (send *robot* :rarm :end-coords)
         (send *robot* :larm :end-coords)
         (send *robot* :rleg :end-coords)
         (send *robot* :lleg :end-coords))
        :link-list
        (mapcar #'(lambda (limb)
                    (send *robot* :link-list (send limb :parent)))
                (list
                 (send *robot* :rarm :end-coords)
                 (send *robot* :larm :end-coords)
                 (send *robot* :rleg :end-coords)
                 (send *robot* :lleg :end-coords))
                )
        :additional-weight-list
        (list (list (send *robot* :rleg :crotch-p :child-link) 0.5)
              (list (send *robot* :rleg :crotch-p :child-link) 0.5)
              (list (send *robot* :rleg :knee-p :child-link) 0.5)
              (list (send *robot* :lleg :knee-p :child-link) 0.5))
        :translation-axis (list t t t t)
        :rotation-axis (list t t t t)
        :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
        :debug-view :no-message)
  (send *irtviewer* :draw-objects)
  (push (send *robot* :angle-vector) avs)
  (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) ~a) ?~%" (+ dt 4000))
  (when (y-or-n-p)
    (send *ri* :angle-vector (send *robot* :angle-vector) (+ dt 4000))
    (send *ri* :wait-interpolation)
    )

  (warn ";; move to pose-2 ?~%")
  (unless (y-or-n-p)
    (return-from jaxon-apc-box-reach-side)
    )

  ;; move to pose-2
  (walking-pose *robot* :root-link-pitch-offset rlpo :head-link-pitch-offset hlpo)
  ;; (send *robot* :rarm-thumb-r :joint-angle 90.0)
  ;; (send *robot* :larm-thumb-r :joint-angle -90.0)
  (send *robot*
        :fullbody-inverse-kinematics
        (list
         (send (send (send (send *apc-box* :side-front-right-grasp-edge) :copy-worldcoords) :translate mid2-reach-pos-r :world) :rotate (deg2rad hand-rotation) :z)
         (send (send (send (send *apc-box* :side-front-left-grasp-edge) :copy-worldcoords) :translate mid2-reach-pos-l :world) :rotate (deg2rad (- hand-rotation)) :z)
         (send *robot* :rleg :end-coords :copy-worldcoords)
         (send *robot* :lleg :end-coords :copy-worldcoords))
        :move-target
        (list
         (send *robot* :rarm :end-coords)
         (send *robot* :larm :end-coords)
         (send *robot* :rleg :end-coords)
         (send *robot* :lleg :end-coords))
        :link-list
        (mapcar #'(lambda (limb)
                    (send *robot* :link-list (send limb :parent)))
                (list
                 (send *robot* :rarm :end-coords)
                 (send *robot* :larm :end-coords)
                 (send *robot* :rleg :end-coords)
                 (send *robot* :lleg :end-coords))
                )
        :additional-weight-list
        (list (list (send *robot* :rleg :crotch-p :child-link) 0.5)
              (list (send *robot* :rleg :crotch-p :child-link) 0.5)
              (list (send *robot* :rleg :knee-p :child-link) 0.5)
              (list (send *robot* :lleg :knee-p :child-link) 0.5))
        :translation-axis (list t t t t)
        :rotation-axis (list t t t t)
        :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
        :debug-view :no-message)
  (send *irtviewer* :draw-objects)
  (push (send *robot* :angle-vector) avs)
  (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) ~a) ?~%" dt)
  (when (y-or-n-p)
    (send *ri* :angle-vector (send *robot* :angle-vector) dt)
    (send *ri* :wait-interpolation)
    )

  (warn ";; move to pose-reach-end ?~%")
  (unless (y-or-n-p)
    (return-from jaxon-apc-box-reach-side)
    )

  ;; move to pose-reach-end
  (walking-pose *robot* :root-link-pitch-offset rlpo :head-link-pitch-offset hlpo)
  ;; (send *robot* :rarm-thumb-r :joint-angle 90.0)
  ;; (send *robot* :larm-thumb-r :joint-angle -90.0)
  (send *robot*
        :fullbody-inverse-kinematics
        (list
         ;;(send (send (send (send *apc-box* :side-front-right-grasp-edge) :copy-worldcoords) :translate mid-reach-pos :world) :rotate (deg2rad 90) :z)
         ;;(send (send (send (send *apc-box* :side-front-left-grasp-edge) :copy-worldcoords) :translate mid-reach-pos :world) :rotate (deg2rad -90) :z)
         (send (send (send (send *apc-box* :side-front-right-grasp-edge) :copy-worldcoords) :translate grasp-reach-pos-r :world) :rotate (deg2rad hand-rotation) :z)
         (send (send (send (send *apc-box* :side-front-left-grasp-edge) :copy-worldcoords) :translate grasp-reach-pos-l :world) :rotate (deg2rad (- hand-rotation)) :z)
         (send *robot* :rleg :end-coords :copy-worldcoords)
         (send *robot* :lleg :end-coords :copy-worldcoords))
        :move-target
        (list
         (send *robot* :rarm :end-coords)
         (send *robot* :larm :end-coords)
         (send *robot* :rleg :end-coords)
         (send *robot* :lleg :end-coords))
        :link-list
        (mapcar #'(lambda (limb)
                    (send *robot* :link-list (send limb :parent)))
                (list
                 (send *robot* :rarm :end-coords)
                 (send *robot* :larm :end-coords)
                 (send *robot* :rleg :end-coords)
                 (send *robot* :lleg :end-coords))
                )
        :additional-weight-list
        (list (list (send *robot* :rleg :crotch-p :child-link) 0.5)
              (list (send *robot* :rleg :crotch-p :child-link) 0.5)
              (list (send *robot* :rleg :knee-p :child-link) 0.5)
              (list (send *robot* :lleg :knee-p :child-link) 0.5))
        :translation-axis (list t t t t)
        :rotation-axis (list t t t t)
        :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
        :debug-view :no-message)
  (send *irtviewer* :draw-objects)
  (push (send *robot* :angle-vector) avs)
  (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) ~a) ?~%" dt)
  (when (y-or-n-p)
    (send *ri* :angle-vector (send *robot* :angle-vector) dt)
    (send *ri* :wait-interpolation)
    )

 (when back-mode
   (warn ";; Back Mode !!!~%")
   (pop avs)
   (dolist (av avs)
     (warn ";; move to next ?~%")
     (unless (y-or-n-p)
       (return-from jaxon-apc-box-reach-side)
       )
     (send *robot* :angle-vector av)
     (send *robot* :fix-leg-to-coords (make-coords))
     (send *irtviewer* :draw-objects)
     (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) ~a) ?~%" dt)
     (when (y-or-n-p)
       (send *ri* :angle-vector (send *robot* :angle-vector) dt)
       (send *ri* :wait-interpolation)
       )
     )
   )
 ))

(defun jaxon-apc-box-reach-side-back-only (&key (mid1-reach-pos-r (float-vector -120 -50 -60)) (mid1-reach-pos-l (float-vector -120 50 -60))
                                      (mid2-reach-pos-r (float-vector 10 -50 -60)) (mid2-reach-pos-l (float-vector 10 50 -60))
                                      (grasp-reach-pos-r (float-vector 0 10 -60)) (grasp-reach-pos-l (float-vector 0 -10 -60))
                                      (hand-rotation 45) (box-pos (float-vector 850 0 0))
                                      (dt 8000) (rlpo 25) (hlpo 30))
  ;; (send *apc-box* :locate (float-vector 600 0 0) :world)
  (send *apc-box* :newcoords (make-coords :pos box-pos))
  (let* (avs)
    (send *robot* :reset-manip-pose)
    (push (send *robot* :angle-vector) avs)
  ;; move to pose-1
  (walking-pose *robot* :root-link-pitch-offset rlpo :head-link-pitch-offset hlpo)
  ;; (send *robot* :rarm-thumb-r :joint-angle 90.0)
  ;; (send *robot* :larm-thumb-r :joint-angle -90.0)
  (send *robot*
        :fullbody-inverse-kinematics
        (list
         ;;(send (send (send (send *apc-box* :side-front-right-grasp-edge) :copy-worldcoords) :translate mid-reach-pos :world) :rotate (deg2rad 90) :z)
         ;;(send (send (send (send *apc-box* :side-front-left-grasp-edge) :copy-worldcoords) :translate mid-reach-pos :world) :rotate (deg2rad -90) :z)
         (send (send (send (send *apc-box* :side-front-right-grasp-edge) :copy-worldcoords) :translate mid1-reach-pos-r :world) :rotate (deg2rad hand-rotation) :z)
         (send (send (send (send *apc-box* :side-front-left-grasp-edge) :copy-worldcoords) :translate mid1-reach-pos-l :world) :rotate (deg2rad (- hand-rotation)) :z)
         (send *robot* :rleg :end-coords :copy-worldcoords)
         (send *robot* :lleg :end-coords :copy-worldcoords))
        :move-target
        (list
         (send *robot* :rarm :end-coords)
         (send *robot* :larm :end-coords)
         (send *robot* :rleg :end-coords)
         (send *robot* :lleg :end-coords))
        :link-list
        (mapcar #'(lambda (limb)
                    (send *robot* :link-list (send limb :parent)))
                (list
                 (send *robot* :rarm :end-coords)
                 (send *robot* :larm :end-coords)
                 (send *robot* :rleg :end-coords)
                 (send *robot* :lleg :end-coords))
                )
        :additional-weight-list
        (list (list (send *robot* :rleg :crotch-p :child-link) 0.5)
              (list (send *robot* :rleg :crotch-p :child-link) 0.5)
              (list (send *robot* :rleg :knee-p :child-link) 0.5)
              (list (send *robot* :lleg :knee-p :child-link) 0.5))
        :translation-axis (list t t t t)
        :rotation-axis (list t t t t)
        :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
        :debug-view :no-message)
  ;; (send *irtviewer* :draw-objects)
  (push (send *robot* :angle-vector) avs)
  ;; (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) ~a) ?~%" (+ dt 4000))
  ;; (when (y-or-n-p)
  ;;   (send *ri* :angle-vector (send *robot* :angle-vector) (+ dt 4000))
  ;;   (send *ri* :wait-interpolation)
  ;;   )

  ;; (warn ";; move to pose-2 ?~%")
  ;; (unless (y-or-n-p)
  ;;   (return-from jaxon-apc-box-reach-side-back-only)
  ;;   )

  ;; move to pose-2
  (walking-pose *robot* :root-link-pitch-offset rlpo :head-link-pitch-offset hlpo)
  ;; (send *robot* :rarm-thumb-r :joint-angle 90.0)
  ;; (send *robot* :larm-thumb-r :joint-angle -90.0)
  (send *robot*
        :fullbody-inverse-kinematics
        (list
         (send (send (send (send *apc-box* :side-front-right-grasp-edge) :copy-worldcoords) :translate mid2-reach-pos-r :world) :rotate (deg2rad hand-rotation) :z)
         (send (send (send (send *apc-box* :side-front-left-grasp-edge) :copy-worldcoords) :translate mid2-reach-pos-l :world) :rotate (deg2rad (- hand-rotation)) :z)
         (send *robot* :rleg :end-coords :copy-worldcoords)
         (send *robot* :lleg :end-coords :copy-worldcoords))
        :move-target
        (list
         (send *robot* :rarm :end-coords)
         (send *robot* :larm :end-coords)
         (send *robot* :rleg :end-coords)
         (send *robot* :lleg :end-coords))
        :link-list
        (mapcar #'(lambda (limb)
                    (send *robot* :link-list (send limb :parent)))
                (list
                 (send *robot* :rarm :end-coords)
                 (send *robot* :larm :end-coords)
                 (send *robot* :rleg :end-coords)
                 (send *robot* :lleg :end-coords))
                )
        :additional-weight-list
        (list (list (send *robot* :rleg :crotch-p :child-link) 0.5)
              (list (send *robot* :rleg :crotch-p :child-link) 0.5)
              (list (send *robot* :rleg :knee-p :child-link) 0.5)
              (list (send *robot* :lleg :knee-p :child-link) 0.5))
        :translation-axis (list t t t t)
        :rotation-axis (list t t t t)
        :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
        :debug-view :no-message)
  ;; (send *irtviewer* :draw-objects)
  (push (send *robot* :angle-vector) avs)
  ;; (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) ~a) ?~%" dt)
  ;; (when (y-or-n-p)
  ;;   (send *ri* :angle-vector (send *robot* :angle-vector) dt)
  ;;   (send *ri* :wait-interpolation)
  ;;   )

  ;; (warn ";; move to pose-reach-end ?~%")
  ;; (unless (y-or-n-p)
  ;;   (return-from jaxon-apc-box-reach-side-back-only)
  ;;   )

  ;; move to pose-reach-end
  (walking-pose *robot* :root-link-pitch-offset rlpo :head-link-pitch-offset hlpo)
  ;; (send *robot* :rarm-thumb-r :joint-angle 90.0)
  ;; (send *robot* :larm-thumb-r :joint-angle -90.0)
  (send *robot*
        :fullbody-inverse-kinematics
        (list
         ;;(send (send (send (send *apc-box* :side-front-right-grasp-edge) :copy-worldcoords) :translate mid-reach-pos :world) :rotate (deg2rad 90) :z)
         ;;(send (send (send (send *apc-box* :side-front-left-grasp-edge) :copy-worldcoords) :translate mid-reach-pos :world) :rotate (deg2rad -90) :z)
         (send (send (send (send *apc-box* :side-front-right-grasp-edge) :copy-worldcoords) :translate grasp-reach-pos-r :world) :rotate (deg2rad hand-rotation) :z)
         (send (send (send (send *apc-box* :side-front-left-grasp-edge) :copy-worldcoords) :translate grasp-reach-pos-l :world) :rotate (deg2rad (- hand-rotation)) :z)
         (send *robot* :rleg :end-coords :copy-worldcoords)
         (send *robot* :lleg :end-coords :copy-worldcoords))
        :move-target
        (list
         (send *robot* :rarm :end-coords)
         (send *robot* :larm :end-coords)
         (send *robot* :rleg :end-coords)
         (send *robot* :lleg :end-coords))
        :link-list
        (mapcar #'(lambda (limb)
                    (send *robot* :link-list (send limb :parent)))
                (list
                 (send *robot* :rarm :end-coords)
                 (send *robot* :larm :end-coords)
                 (send *robot* :rleg :end-coords)
                 (send *robot* :lleg :end-coords))
                )
        :additional-weight-list
        (list (list (send *robot* :rleg :crotch-p :child-link) 0.5)
              (list (send *robot* :rleg :crotch-p :child-link) 0.5)
              (list (send *robot* :rleg :knee-p :child-link) 0.5)
              (list (send *robot* :lleg :knee-p :child-link) 0.5))
        :translation-axis (list t t t t)
        :rotation-axis (list t t t t)
        :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
        :debug-view :no-message)
  ;; (send *irtviewer* :draw-objects)
  (push (send *robot* :angle-vector) avs)
  ;; (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) ~a) ?~%" dt)
  ;; (when (y-or-n-p)
  ;;   (send *ri* :angle-vector (send *robot* :angle-vector) dt)
  ;;   (send *ri* :wait-interpolation)
  ;;   )

   (pop avs)
   (dolist (av avs)
     (warn ";; move to next ?~%")
     (unless (y-or-n-p)
       (return-from jaxon-apc-box-reach-side-back-only)
       )
     (send *robot* :angle-vector av)
     (send *robot* :fix-leg-to-coords (make-coords))
     (send *irtviewer* :draw-objects)
     (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector) ~a) ?~%" dt)
     (when (y-or-n-p)
       (send *ri* :angle-vector (send *robot* :angle-vector) dt)
       (send *ri* :wait-interpolation)
       )
     )
 ))

(warn "~%")
(warn ";; Servo On with Reset Landing Pose (from dashboard)~%")
(warn "(make-jaxon-ri)~%")
(warn "(start-auto-balancer-no-fix)~%")
(warn ";; remove ForceSensorOffset (from dashboard)~%")
(warn ";; chakuchi~%")
(warn ";; reset-pose (from dashboard)~%")
(warn ";; start Stabilizer (from dashboard)~%")
(warn "~%")
(warn "(jaxon-apc-box-init) ;; For jaxon initialize~%")
;; (warn "(hrp2jsk-apc-box-init) ;; For hrp2jsk initialize~%")
;; (warn "(hrp2jsknt-apc-box-init) ;; For hrp2jsknt initialize~%")
(warn "~%")
(warn "(start-calib-box-pos) ;; Calibrate box pose: Get value *calib-offset-coords*~%")
(warn "~%")
(warn "(update-box-pose-by-sift :calib-offset t) ;; Update box pos by sift recognition~%")
(warn "(loop-update-box-pose :calib-offset t) ;; Loop update box pos~%")
;; (warn "~%")
;; (warn "(hrp2jsk-box-okkake-reaching-demo)~%")
(warn "~%")
(warn "(jaxon-maai-hosei-by-go-pos *apc-box*) ;; Adjust box maai before reaching~%")
;; (warn "(hrp2jsk-maai-hosei-by-go-pos *apc-box*) ;; Adjust box maai before reaching~%")
;; (warn "(hrp2jsknt-maai-hosei-by-go-pos *apc-box*) ;; Adjust box maai before reaching~%")
;; (warn "~%")
;; (warn "(jaxon-apc-box-reach) ;; Generate reach motion by jaxon~%")
;; (warn "(hrp2jsk-apc-box-reach) ;; Generate reach motion by hrp2jsk~%")
;; (warn "(hrp2jsknt-apc-box-reach) ;; Generate reach motion by hrp2jsknt~%")
(warn "~%")
(warn "(jaxon-apc-box-reach-side)~%")
(warn "~%")
(warn "(push-main-all :push-distance 0.5)~%")
(warn "(push-main-all :push-distance 3.0) ;; for demo~%")


;; Change hand-fix-mode to true
;; (send *ri* :set-auto-balancer-param :is-hand-fix-mode t)

;; Set parameters
;; (send *ri* :set-object-turnaround-detector-param :wrench-cutoff-freq 5.0 :dwrench-cutoff-freq 5.0)

;; Setq ref-force to move
;; (setq *box-push-ref-force* (send *ri* :set-object-turnaround-ref-force :max-ref-force 50 :max-time 4000 :axis (float-vector -1 0 0) :set-ref-force-p nil))

;; Pushing go-pos
;; (progn (send *ri* :set-ref-force (car *box-push-ref-force*) 2000 :arms) (send *ri* :wait-interpolation-seq) (send *ri* :go-pos 0.5 0 0) (send *ri* :set-ref-force (float-vector 0 0 0) 2000 :arms) (send *ri* :wait-interpolation-seq))

;; Change hand-fix-mode to nil
;; (send *ri* :set-auto-balancer-param :is-hand-fix-mode nil)


(defun push-main-all (&key (push-distance 0.5))
  ;; Reaching motion
  (warn ";; (jaxon-apc-box-reach-side :back-mode nil) ?~%")
  (if (y-or-n-p)
      (jaxon-apc-box-reach-side :back-mode nil)
    (return-from push-main-all))

  ;; Change hand-fix-mode to true
  (warn ";; (send *ri* :set-auto-balancer-param :is-hand-fix-mode t) ?~%")
  (if (y-or-n-p)
      (send *ri* :set-auto-balancer-param :is-hand-fix-mode t)
    (return-from push-main-all))

  ;; Set parameters
  (warn ";; (send *ri* :set-object-turnaround-detector-param :wrench-cutoff-freq 5.0 :dwrench-cutoff-freq 5.0) ?~%")
  (if (y-or-n-p)
      (send *ri* :set-object-turnaround-detector-param :wrench-cutoff-freq 5.0 :dwrench-cutoff-freq 5.0)
    (return-from push-main-all))

  ;; Setq ref-force to move
  (warn ";; (setq *box-push-ref-force* (send *ri* :set-object-turnaround-ref-force :max-ref-force 50 :max-time 4000 :axis (float-vector -1 0 0) :set-ref-force-p nil)) ?~%")
  (if (y-or-n-p)
      (setq *box-push-ref-force* (send *ri* :set-object-turnaround-ref-force :max-ref-force 50 :max-time 4000 :axis (float-vector -1 0 0) :set-ref-force-p nil))
    (return-from push-main-all))

  ;; Pushing go-pos
  (warn ";; (progn (send *ri* :set-ref-force (car *box-push-ref-force*) 2000 :arms) (send *ri* :wait-interpolation-seq) (send *ri* :go-pos ~a 0 0) (send *ri* :set-ref-force (float-vector 0 0 0) 2000 :arms) (send *ri* :wait-interpolation-seq)) ?~%" push-distance)
  (if (y-or-n-p)
      (progn (send *ri* :set-ref-force (car *box-push-ref-force*) 2000 :arms) (send *ri* :wait-interpolation-seq) (send *ri* :go-pos push-distance 0 0) (send *ri* :set-ref-force (float-vector 0 0 0) 2000 :arms) (send *ri* :wait-interpolation-seq))
    (return-from push-main-all))
  
  ;; Change hand-fix-mode to nil
  (warn ";; (send *ri* :set-auto-balancer-param :is-hand-fix-mode nil) ?~%")
  (if (y-or-n-p)
      (send *ri* :set-auto-balancer-param :is-hand-fix-mode nil)
    (return-from push-main-all))

  ;; Release motion
  (warn ";; (jaxon-apc-box-reach-side-back-only) ?~%")
  (if (y-or-n-p)
      (jaxon-apc-box-reach-side-back-only)
    (return-from push-main-all))
  )
