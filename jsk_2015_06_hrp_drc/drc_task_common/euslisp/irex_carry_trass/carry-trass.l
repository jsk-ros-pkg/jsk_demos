(load "package://drc_task_common/euslisp/robot-util.l")
(load "primitive-marker-for-trass.l")

(load "trass.l")
(load "motion-sequence-server.l"))

;; (load "get-trass-coords.l")

;; (defun model2real (&key (wait? t) (time 5000))
;;   (print "model2real"))
(require :speak "package://pr2eus/speak.l")

(defmethod robot-model
  (:reset-manip-pose-fix ()
    (let (init-fmc)
      (setq init-fmc (send (send self :foot-midcoords) :copy-worldcoords))
      (send self :reset-manip-pose)
      (send self :fix-leg-to-coords init-fmc)
      ))
  (:looking-pose ()
    (send self :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 45.9594 -31.8029 30.4303 -96.3903 -26.3567 -31.0451 -55.2799 60.0 45.9594 31.8029 -30.4303 -96.3903 26.3567 31.0451 -55.2799 -60.0))
    )
  (:reaching-trass-pose 
   ()
   (let (init-fmc)
     (setq init-fmc (send (send self :foot-midcoords) :copy-worldcoords))
     (send self :angle-vector #f(-0.830455 -2.558 -72.6499 74.4602 -19.8032 2.68934 -0.830455 -2.558 -72.6499 74.4602 -19.8032 2.68934 10.9731 35.0 -11.5157 0.0 30.0 73.7243 -9.046 -78.5304 12.9514 -4.59948 60.1932 5.31151 -25.0292 -67.1939 -3.82631 70.1478 12.5305 -2.10181 31.8227 -24.6574 2.5476))
     (send self :fix-leg-to-coords init-fmc)
     )))

(defun init (&key (robot-name "jaxon_red") (trass-pos #f(0 0 0)) (robot-pos nil) (real? nil))
  (let ()
    (if (equal real? :choreonoid)
        (progn 
          (load "package://hrpsys_ros_bridge_jvrc/euslisp/jaxon_jvrc-interface.l")
          (jaxon_jvrc-init)
          (setq *robot* *jaxon_jvrc*)
          (setq *robot-name* "JAXON_RSD")
          )
      (generate-robot robot-name :generate-ri? real?))
    (send *robot* :reset-manip-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (when (not robot-pos)
      (if (equal (send *robot* :name) "JAXON_RED")
          (setq robot-pos #f(-300 0 0))
        (setq robot-pos #f(-300 0 0))))
    (make-trass)
    (when trass-pos
      (send *trass* :translate trass-pos))
    (send *trass* :translate-larm #f(-20 -20 15))
    (send *trass* :translate-rarm #f(-20 -20 15))
    
    (send *robot* :translate robot-pos)
    (send *trass* :set-default-coords)
    (objects (list *robot* *trass*))
    (send *irtviewer* :title "irex trass demo")
    (setq *initial-foot-midcoords* (send (send *robot* :foot-midcoords) :copy-worldcoords))
    ))

(defun reach-trass (&key (hand-pos-to-trass #f(0 0 0))
                         (trass-pos #f(0 0 0))
                         (relative? nil)
                         (hip-contact-coords nil)
                         (init-av nil)
                         (r-translate #f(0 0 0))
                         (l-translate #f(0 0 0))
                         (crotch-p -60)
                         )
  (let (move-target move-target-arms target-coords link-list fmc init-trass-coords ret additional-weight-list translation-axis rotation-axis)
    (setq fmc (send (send *robot* :foot-midcoords) :copy-worldcoords))
    (unless init-av
      (progn 
        (send *robot* :reset-manip-pose)
        (send *robot* :legs :crotch-p :joint-angle crotch-p)
        (send *robot* :fix-leg-to-coords *initial-foot-midcoords*)) 
      (send *robot* :angle-vector init-av))
    (send *robot* :fix-leg-to-coords fmc)
    (setq init-trass-coords (send (send *trass* :worldcoords) :copy-worldcoords))
    (send *trass* :init-coords)
    (send *trass* :translate trass-pos)
    (when relative? (send *trass* :newcoords (send (send (send *robot* :foot-midcoords) :copy-worldcoords)
                                                   :translate trass-pos)))
    
    ;;options for full-body-ik
    ;;move-target
    (setq move-target
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)))
    (setq move-target-arms
          (if (equal (send *robot* :name) "HRP2JSKNT")
              (mapcar #'(lambda (k tr rot) (make-cascoords
                                            :coords (send (send (send (send *robot* k :end-coords) 
                                                                      :copy-worldcoords)
                                                                :translate tr)
                                                          :rotate rot :z)
                                            :parent (send (send *robot* k :end-coords) :parent)))
                      (list :rarm :larm) (list #f(40 10 0) #f(40 -10 0)) 
                      (list (deg2rad 60) (deg2rad -60)))
            (reverse (send *robot* :arms :end-coords))
            ))
    (setq move-target (append move-target move-target-arms))
    ;;target-coords
    (setq target-coords
          (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                (send (send (send (send *trass* :rarm) :copy-worldcoords) :rotate pi :x) :translate hand-pos-to-trass :world)
                (send (send (send *trass* :larm) :copy-worldcoords) :translate hand-pos-to-trass :world)
                ))
    ;;additional weight list
    (if (equal (send *robot* :name) "HRP2JSKNT")
        (setq additional-weight-list 
              (list (list (send *robot* :chest_joint0 :child-link) 0)
                    (list (send *robot* :rleg :toe-p :child-link) 0)
                    (list (send *robot* :lleg :toe-p :child-link) 0)))
      (setq additional-weight-list 
            (list
             (list (list (send *robot* :rleg  :ankle-p :child-link) 0.0))
             (list (list (send *robot* :lleg  :ankle-p :child-link) 0.0))
             (list (list (send *robot* :torso :waist-r :child-link) 0))
             (list (list (send *robot* :torso :waist-y :child-link) 0))
             )))
    ;;translation-axis
    (setq translation-axis (list t t t t))
    ;;rotation-axis
    (setq rotation-axis (list t t t t))
    
    (setq thre (list 1 1 1 1))
    (setq rthre (list 0.017453 0.017453 0.017453 0.017453))
    ;;hip coords
    (when hip-contact-coords
      (send hip-contact-coords :draw-on :flush t)
      (setq move-target (append move-target (list (send *robot* :get :hip-contact-coords))))
      (setq target-coords (append target-coords (list hip-contact-coords)))
      (setq translation-axis (append translation-axis (list :xz)))
      (setq rotation-axis (append rotation-axis (list t)))
      (setq thre (append thre (list 60)))
      (setq rthre (append rthre (list (deg2rad 30))))
      )
    ;;link-list
    (setq link-list
          (mapcar #'(lambda (limb)
                      (send *robot* :link-list (send limb :parent))) move-target))
    ;;solve ik
    (setq ret
          (send *robot* :fullbody-inverse-kinematics target-coords
                :move-target move-target
                  :link-list link-list
                  :translation-axis translation-axis
                  :rotation-axis rotation-axis
                  :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
                  :additional-weight-list additional-weight-list
                  :debug-view nil;;:no-message
                  :stop 100
                  ;; :thre thre
                  ;; :rthre rthre
                  ))
    (when (not ret) (send *trass* :newcoords init-trass-coords))
    (send *irtviewer* :draw-objects)
    (mapcar #'(lambda (x) (send x :draw-on :flush t)) move-target-arms)
    ))
(defun set-ref-force-for-trass ()
  (send *ri* :set-ref-force (float-vector 0 0 -15) 2000 :arms)
  (send *ri* :wait-interpolation-seq)
  (let ((flag t))
    (while flag
      (send *ri* :set-object-turnaround-detector-param
            ;;:detect-time-thre 0.2 :start-time-thre 0.2)
            :detect-time-thre 0.3 :start-time-thre 0.3)
      (send *ri* :set-object-turnaround-ref-force :max-ref-force (* 9.8 7.0))
      (if (y-or-n-p ";; set ref force OK? (near from ~A?) (Y=>set ref force. N=>try estimate force again.)~%"
                    (list (float-vector 0 0 (* -1 6.0 9.8 0.5)) (float-vector 0 0 (* -1 6.0 9.8 0.5))))
          (setq flag nil)))))

(defun walking-pose ()
  (send *robot* :reset-manip-pose-fix)
  ;; (reach-trass :trass-pos #f(300 0 550) :relative? t :crotch-p -20)
  (prog1
   (reach-trass :trass-pos #f(300 0 550) :relative? t :crotch-p -40)
   (let ((fmc (send *robot* :foot-midcoords)))
     ;;(send *robot* :legs :move-end-pos #f(0 0 50) :world)
     (send *robot* :legs :move-end-pos #f(0 0 20) :world)
     (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
     (send *robot* :fix-leg-to-coords fmc)
     ))
  )
(defun look-trass ()
  (send *robot* :reset-manip-pose-fix)
  (send *robot* :CHEST_JOINT1 :joint-angle 34.5)
  (let ((cds (send *robot* :foot-midcoords)))
    (send *robot* :legs :crotch-p :joint-angle -27)
    (send *robot* :fix-leg-to-coords cds)
    (send *robot* :move-centroid-on-foot :both (list :lleg :rleg))
    )
  (send *robot* :look-at-target (send *trass* :centroid-worldcoords))
  (send *irtviewer* :draw-objects)
  )

(defun start-demo ()
  (print "robot in in the air?  (y/n)")
  (when (equal (read-line) "y")
    (send *ri* :start-auto-balancer :limbs '(:rleg :lleg :rarm :larm))
    (send *ri* :set-auto-balancer-param :is-hand-fix-mode nil)
    ;; (send *ri* :start-default-unstable-controllers :abc-limbs '(:rleg :lleg :rarm :larm))
    ))
(defun demo ()
  (send *ri* :set-ref-force #f(0 0 0) 3000 :arms)
  (erase-all-primitive-marker)
  (set-pose-trass)
  (set-trass-coords-by-pf)
  ;;primitive marker
  ;;(set-primitive-marker-color :r 1 :a 0.8)
  ;; rosservice call /transformable_interactive_server/set_color '{target_name: "trass",  color: {r: 1.0, g: 0.0,  b: 0.0, a: 1.0}}'
  
  (print "look trass.ok?")
  (when (equal (read-line) "y")
    (look-trass)
    (model2real)
    (set-trass-coords-by-pf)
    (print "please adjust. Do you have done?")
    (when (equal (read-line) "y")
      (adjust-robot-by-primitive-marker)
      ;;walk
      (walk-to-target :real? t :k 0.9)
      ))
  
  ;; (print "look trass.ok?")
  ;; (unless (equal (read-line) "y") (return-from demo nil))
  ;; (look-trass)
  ;; (model2real)
  
  (set-trass-coords-by-pf)
  (print "please adjust. Do you have done?")
  (when (equal (read-line) "y")
    (adjust-robot-by-primitive-marker)
    
    (print "let's walk again! OK? (y/n)")
    (when (equal (read-line) "y")
      (walk-to-target :real? t)))
  
  (print "please adjust. Do you have done?")
  (when (equal (read-line) "y")
    (adjust-robot-by-primitive-marker)
    
    (execute-motion)
    
    ))


(defun plan-motion (&optional (check? t))
  (send *trass* :reset-coords)
  (setq *rsl* (instance motion-sequence-server :init))
  
  ;;init
  ;; (send *rsl* :save-rs :name :init :command 
  ;;       #'(lambda()
  ;;           (send *ri* :start-default-unstable-controllers :abc-limbs '(:rleg :lleg :rarm :larm))))

  ;;reset-manip-pose
  (send *rsl* :save-rs :name :force-offset :command #'(lambda() (progn (send *ri* :reset-force-moment-offset-arms))))
  ;; (send *robot* :reset-manip-pose-fix)
  ;; (send *rsl* :save-rs :name :reset-manip-pose)

  ;; ;;open-hand
  ;; (send *rsl* :save-rs :name :open-hand :command #'(lambda() (set-hand :state :open :real? t)))
 
  ;;pre-reach
  (reach-trass :trass-pos #f(0 0 150))
  (send *rsl* :save-rs :name :pre-reach :time 10000)
  ;; comment out by nozawa 20151129
  ;; (send *rsl* :save-rs :name :preshape :command #'(lambda()
  ;;                                                   (send *ri* :move-gripper :rarm 0.1)
  ;;                                                   (send *ri* :move-gripper :larm 0.1)))
  ;;reach-trass
  (reach-trass :trass-pos #f(0 0 0))
  (send *rsl* :save-rs :name :reach)

  ;;close-hand
  (send *rsl* :save-rs :name :close-hand :command #'(lambda() (set-hand :state :close :real? t)))
  
  ;;set-ref-force
  (send *rsl* :save-rs :name :set-ref-force :command #'set-ref-force-for-trass)
  
  ;;lift
  (reach-trass :trass-pos #f(300 0 150) :relative? t)
  (send *rsl* :save-rs :name :lift :yes nil)
  
  ;;walking-pose
  (walking-pose)
  (send *rsl* :save-rs :name :walking-pose)

  ;;go-pos-0-0-0
  (send *rsl* :save-rs :name :go-pos-0-0-90 :command #'(lambda() (send *ri* :go-pos 0 0 90)))

  ;;pre-put
  (reach-trass :trass-pos #f(300 0 150) :relative? t)
  (send *rsl* :save-rs :name :pre-put)
  ;;put
  (reach-trass :trass-pos #f(300 0 0) :relative? t)
  (send *rsl* :save-rs :name :put)

  ;;set-ref-force-zero
  (send *rsl* :save-rs :name :set-ref-force-ZERO :command #'(lambda () (send *ri* :set-ref-force (float-vector 0 0 0) 2000 :arms)) :yes nil)
  
  ;;open-hand
  (send *rsl* :save-rs :name :open-hand :command #'(lambda() (set-hand :state :open :real? t)) :yes nil)
  
  ;;unreach-hand
  (send *rsl* :set-rs :name :pre-put)
  ;;(reach-trass :trass-pos #f(300 0 300) :relative? t)
  (send *rsl* :save-rs :name :unreach-hand :time 10000)
  
  ;;reset-manip-pose
  (send *robot* :reset-manip-pose-fix)
  (send *irtviewer* :draw-objects)
  (send *rsl* :save-rs :name :reset-manip-pose)
  
  
  ;; ;;lift-300
  ;; (reach-trass :trass-pos #f(0 0 300))
  ;; (send *rsl* :save-rs :name :lift-300 :time 10000)
  
  ;; ;;lift-500
  ;; (reach-trass :trass-pos #f(-100 0 500))
  ;; (send *rsl* :save-rs :name :lift-500)

  ;;(send *rsl* :play-for-check)
  )

(defun set-impedance-param-for-trass ()
  ;;(start-soft-impedance)
  ;;(send *ri* :start-impedance :arms :m-p 10 :d-p 2000 :k-p 1000 :m-r 2.0 :d-r 200 :k-r 80)
  ;; before 1129 reha 2
  ;;(send *ri* :start-impedance :arms :m-p 10 :d-p 1500 :k-p 300 :m-r 2.0 :d-r 100 :k-r 40)
  (send *ri* :start-impedance :arms :m-p 10 :d-p 1500 :k-p 300 :m-r 2.0 :d-r 100 :k-r 40 :moment-gain #f(0 0 0))
  ;;(send *ri* :start-impedance :arms :m-p 10 :d-p 1500 :k-p 600 :m-r 2.0 :d-r 200 :k-r 100)
  )

(defun execute-motion (&key(real? nil))
  ;; ;; (set-pose-trass)
  ;; ;;  (adjust-robot-by-bounding-box)
  
  ;; ;;(walk-to-target :real? real?)
  ;; ;;(set-pose-trass)
  
  ;; ;; (adjust-robot-by-primitive-marker)
  
  ;; ;;(adjust-robot-by-bounding-box)
  (set-impedance-param-for-trass)
  (plan-motion)
  (send *rsl* :play-in-real)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-impedance :arms)
  ;; ;; (set-ref-force)
  ;; ;; (send *rsl* :play-in-real :play-from-first nil)
  (send *ri* :set-ref-force #f(0 0 0) 3000 :arms)
  )


(defun set-hand (&key(real? t) (state :open))
  (cond 
   ((equal (send *robot* :name) "HRP2JSKNT")
    (if (eq state :open)
        (progn (send (send *robot* :hand :larm) :hook-pose)
               (send (send *robot* :hand :rarm) :hook-pose))
      (progn (send (send *robot* :hand :larm) :grasp-pose)
             (send (send *robot* :hand :rarm) :grasp-pose)))
    (send *irtviewer* :draw-objects)
    (when real?
      (send *ri* :hand-angle-vector
            (apply #'concatenate float-vector
                   (send *robot* :hand :arms :angle-vector)) 1000)))
   (t
    (when real?
      (if (eq state :open)
          (progn (send *ri* :stop-grasp :arms)
                 (send *ri* :set-ref-force (float-vector 0 0 0) 2000 :arms))
        (send *ri* :start-grasp :arms)
        )))
   ))


;; start-[default/soft/hard]-impedanceを利用する
;; (defun impedance (&key (state :standard))
;;   (case state
;;     (:soft 
;;      (send *ri* :start-impedance :arms :k-p 200 :d-p 180 :m-p 5 :m-r 1 :d-r 15 :k-r 20)
;;      :force-gain (float-vector 1 1 1) :moment-gain (float-vector 1.0 1.0 1.0))
;;     (:standard 
;;      (send *ri* :start-impedance :arms :k-p 1000 :d-p 500 :m-p 7.5 :m-r 5 :d-r 50 :k-r 50 
;;            :force-gain (float-vector 1 1 1) :moment-gain (float-vector 1.0 1.0 1.0)))
;;     (:stop 
;;       (send *ri* :stop-impedance :arms))
;;     ))




(defun torque-ratio-vector ()
  (let (robot-weight g half-robot-gravity torque-vector)
    (setq robot-weight (* 1e-3 (send *robot* :weight)))
    (setq g (* (elt *g-vec* 2) 1e-3))
    (setq half-robot-gravity (* g robot-weight 0.5))
    (setq torque-vector
          (send *robot* :torque-vector
                :force-list (list (float-vector 0 0 half-robot-gravity) (float-vector 0 0 half-robot-gravity))
                :moment-list (list #f(0 0 0) #f(0 0 0))
                :target-coords (append (send *robot* :legs :end-coords))))
    (print torque-vector)    
    (send *robot* :torque-ratio-vector :torque torque-vector)
    ))
(defun set-pose-trass ()
  (make-interactive-marker)
  )

(defun adjust-robot-by-trass-coords (trass-coords)
  (let (robot-real-c pos rot fm dz
                     cds icds)
    ;;robot->trass_cog
    (setq cds (make-coords :pos (let ((p (send trass-coords :worldpos))) (setf (elt p 2) 0) p) 
                           :rpy (list (caar (send trass-coords :rpy-angle)) 0 0)))
    ;; ;;robot->trass_wc
    (setq x/2 (/ (elt (send *trass* :size) 0) 2))
    (send cds :translate (float-vector (- x/2) 0 0))
    
    ;;trass_cog->robot
    (setq icds (send cds :inverse-transformation))
    
    (send *robot* :fix-leg-to-coords icds)
    
    ;; (setq robot-real-c (send (send (send trass-coords :copy-worldcoords) :inverse-transformation) 
    ;;                          :transform (send *robot* :foot-midcoords)))
    ;; (setq pos (send robot-real-c :worldpos))
    ;; (setq rot (send robot-real-c :difference-rotation (make-coords)))
    ;; (setf (elt rot 0) 0)
    ;; (setf (elt rot 1) 0)
    
    ;; (send *robot* :fix-leg-to-coords 
    ;;       (send (send (send *trass* :centroid-worldcoords) :translate pos) :rotate (norm rot) :z))
    (setq fm (send (send (send *robot* :foot-midcoords) :copy-worldcoords) :worldpos))
    (setq dz (elt fm 2))
    ;;(send *robot* :translate (float-vector 0 0 (- dz)))
    (send *irtviewer* :draw-objects)
    (send *trass* :set-default-coords)
    ))

(defun primitive-marker-coords (&key (name "trass"))
  ;;(let (coords-to-fmc)
  ;;(setq coords-to-fmc (get-primitive-marker-pose :name name :frame-id "ground" :wait-duration 1.0))
  ;;(send (send (send *robot* :foot-midcoords) :copy-worldcoords) :transform coords-to-fmc)
  ;;)
  ;;(get-primitive-marker-pose :name name :frame-id "ground" :wait-duration 1.0))
  (get-primitive-marker-pose :name name :frame-id "ground"))

(defun adjust-robot-by-primitive-marker (&key (name "trass"))
  (adjust-robot-by-trass-coords (primitive-marker-coords))
  )

(defun adjust-robot-by-bounding-box ()
  (print "adjust!")
  (adjust-robot-by-trass-coords (get-trass-coords))
  )


(defun walk-to-target (&key (target-coords *initial-foot-midcoords*) (real? nil) (k 1.0))
  (let (tcds);;(current-coords diff pos-diff rot-z loacl-pos-diff dx dy rot-z-deg)
    ;; (adjust-robot-by-bounding-box)
    
    (setq current-coords (send (send *robot* :foot-midcoords) :copy-worldcoords))
    (setq pos-diff (send current-coords :difference-position target-coords))
    (setq rot-diff (send current-coords :difference-rotation target-coords))
    
    (setq rot-z (elt rot-diff 2))
    
    ;;check target position
    (setq tcds (send (send (send (send *robot* :foot-midcoords) :copy-worldcoords) 
                           :translate pos-diff) :rotate rot-z :z))
    (send tcds :draw-on :flush t)
    
    (setq dx (* k (* 0.001 (elt pos-diff 0))))
    (setq dy (* k (* 0.001 (elt pos-diff 1))))
    (setq rot-z-deg (rad2deg rot-z))
    
    (when real? 
      (format t "go-po ~A ~A ~A ~%" dx dy rot-z-deg)
      (if (y-or-n-p "walk OK?")
          (progn 
            (send *ri* :go-pos dx dy rot-z-deg))))    
    tcds
    ))



