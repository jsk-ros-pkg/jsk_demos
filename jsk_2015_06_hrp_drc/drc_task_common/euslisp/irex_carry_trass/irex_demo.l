(load "carry-trass.l")
(defun prepare-carry-trass-demo()
  (send *ri* :set-ref-force #f(0 0 0) 3000 :arms)
  (set-impedance-param-for-trass)
  (erase-all-primitive-marker)
  (set-pose-trass)
  (set-trass-coords-by-pf)
  )
;;to-do: 最初にトラスをどう見るか？　認識が合うまで待つか？
(defun walk-to-trass ()
  (print "look trass.ok?")
  (send *robot* :translate #f(-300 0 0))
  (look-trass)
  (when (y-or-n-p "look trass. OK?")
    (model2real))
  ;;(unix::sleep 5)
  (set-trass-coords-by-pf-10)
  (when (y-or-n-p "please adjust. Do you have done?")
    (adjust-robot-by-primitive-marker)
    ;;walk
    (send *robot* :fix-leg-to-coords (walk-to-target :real? t :k 0.9))
    (look-trass)
    (model2real)
    )
  (look-trass)
  ;;(unix::sleep 5)
  (set-trass-coords-by-pf-10)
  (when (y-or-n-p "please adjust. Do you have done?")
    (adjust-robot-by-primitive-marker)
    ;;walk
    (send *robot* :fix-leg-to-coords (walk-to-target :real? t))
    (look-trass)
    (model2real)
    )
  ;;(unix::sleep 5)
  (set-trass-coords-by-pf-10)
  (when (y-or-n-p "please adjust. Do you have done?")
    (adjust-robot-by-primitive-marker))
  )
(defun reach-trass-exec (&key (real? t))
  ;;reset-force-moment-offset-arms
  (send *ri* :reset-force-moment-offset-arms)
  ;;pre-reach
  (send *rsl* :set-rs :name :pre-reach)
  (model2real :time 10000)
  ;;reach-trass
  (send *rsl* :set-rs :name :reach)
  (model2real :time 5000)

  ;;close-hand
  (set-hand :state :close :real? real?)
  )
(defun lift-trass (&key (real? t))
  ;;set-ref-force
  (set-ref-force-for-trass)
  ;;lift
  (start-hard-impedance)
  (send *rsl* :set-rs :name :lift)
  (model2real)
  ;;walking-pose
  (send *rsl* :set-rs :name :walking-pose)
  (model2real))
(defun carry-trass (&key (real? t))
  ;;go-pos-0-0-0
  (when real?
    (let* ((org-str-prm (send (send *ri* :get-gait-generator-param) :stride_parameter))
           (str-prm (copy-object org-str-prm)))
      (setf (elt str-prm 2) 20.0) ;; Rotation
      (setf (elt str-prm 3) 0.15) ;; Backward
      (send *ri* :set-gait-generator-param :stride-parameter str-prm)
      (send *ri* :go-pos 0 0 90)
      (send *ri* :go-pos 0.15 0 0)
      (send *ri* :set-gait-generator-param :stride-parameter org-str-prm)
      )))
(defun put-trass (&key (real? t))
  ;;pre-put
  (send *rsl* :set-rs :name :pre-put)
  (model2real)
  (set-impedance-param-for-trass)
  ;;put
  (send *rsl* :set-rs :name :put)
  (model2real)
  ;;set-ref-force-zero
  (send *ri* :set-ref-force (float-vector 0 0 0) 2000 :arms)
  )
(defun release-trass-exec (&key (real? t))
  ;;open-hand
  (set-hand :state :open :real? real?)
  ;;unreach-hand
  ;;(reach-trass :trass-pos #f(300 0 300) :relative? t)
  (send *rsl* :set-rs :name :unreach-hand)
  (model2real)
  ;;reset-manip-pose
  (send *rsl* :set-rs :name :reset-manip-pose)
  (model2real)
  )
(defun next-to-valve (&key (real? t))
  (let* ((org-str-prm (send (send *ri* :get-gait-generator-param) :stride_parameter))
         (str-prm (copy-object org-str-prm)))
    (setf (elt str-prm 2) 20.0) ;; Rotation
    (setf (elt str-prm 3) 0.15) ;; Backward
    (send *ri* :set-gait-generator-param :stride-parameter str-prm)
    (print "go-pos-0.3 0 0")
    (send *ri* :go-pos -0.3 0 0)
    (print "go-pos 0 0 -90")
    (send *ri* :go-pos 0 0 -90)
    (send *ri* :set-gait-generator-param :stride-parameter org-str-prm)
    ))

(defun demo-carry-trass (&key (move-to-trass t))
  (if move-to-trass
      (progn
        (prepare-carry-trass-demo)
        (walk-to-trass))
    (progn
      (set-trass-coords-by-pf-10)
      (when (y-or-n-p "please adjust. Do you have done?")
        (adjust-robot-by-primitive-marker))
      ))
  (plan-motion)
  (when (y-or-n-p "reach trass. ok?")
    (reach-trass-exec))
  (when (y-or-n-p "lift trass. ok?")
    (lift-trass))
  (when (y-or-n-p "walk & carry trass. ok?")
    (carry-trass))
  (when (y-or-n-p "put trass. ok?")
    (put-trass))
  (unless (equal (send *ri* :state :emergency-mode) 0)
    (when (y-or-n-p "release-emergency-mode. ok?")
      (send *ri* :emergency-release-motion)))
  (when (y-or-n-p "release trass. ok?")
    (release-trass-exec))
  (when (y-or-n-p "walk for nex task. ok?")
    (next-to-valve))
  )
(warn ";; (init :real? t)~%")
(warn ";; (demo-carry-trass)~%")
(warn ";; (demo-carry-trass :move-to-trass nil)~%")

;; for emergency
;; 0. hand hiraku
;; 1. (send *rsl* :set-rs :name :pre-put)
;; 2. (progn (send *ri* :angle-vector (send *robot* :angle-vector) 5000) (send *ri* :wait-interpolation))
;; 3. emergency wo release
;; 4. (progn (send *robot* :reset-manip-pose) (send *ri* :angle-vector (send *robot* :angle-vector) 5000) (send *ri* :wait-interpolation))
