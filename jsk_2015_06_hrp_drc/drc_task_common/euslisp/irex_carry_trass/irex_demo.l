(load "carry-trass.l")
(defun prepare-demo()
  (send *ri* :set-ref-force #f(0 0 0) 3000 :arms)
  (erase-all-primitive-marker)
  (set-pose-trass)
  (set-trass-coords-by-pf)
  )
;;to-do: 最初にトラスをどう見るか？　認識が合うまで待つか？
(defun walk-to-trass ()
  (print "look trass.ok?")
  (look-trass)
  (when (y-or-n-p "look trass. OK?")
    (model2real))
  (set-trass-coords-by-pf-10)
  (when (y-or-n-p "please adjust. Do you have done?")
    (adjust-robot-by-primitive-marker)
    ;;walk
    (walk-to-target :real? t :k 0.9)
    )
  (look-trass)
  (set-trass-coords-by-pf-10)
  (when (y-or-n-p "please adjust. Do you have done?")
    (adjust-robot-by-primitive-marker)
    ;;walk
    (walk-to-target :real? t))
  (when (y-or-n-p "please adjust. Do you have done?")
    (adjust-robot-by-primitive-marker))
  )
(defun reach (&key (real? t))
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
(defun lift (&key (real? t))
  ;;set-ref-force
  (set-ref-force)
  ;;lift
  (send *rsl* :set-rs :name :lift)
  (model2real)
  ;;walking-pose
  (send *rsl* :set-rs :name :walking-pose)
  (model2real))
(defun carry-trass (&key (real? t))
  ;;go-pos-0-0-0
  (when real?
    (send *ri* :go-pos 0 0 90)
    (send *ri* :go-pos 0.15 0 0)
    ))
(defun put (&key (real? t))
  ;;pre-put
  (send *rsl* :set-rs :name :pre-put)
  (model2real)
  ;;put
  (send *rsl* :set-rs :name :put)
  (model2real)
  ;;set-ref-force-zero
  (send *ri* :set-ref-force (float-vector 0 0 0) 2000 :arms)
  )
(defun release (&key (real? t))
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
(defun next (&key (real? t))
  (print "go-pos-0.5 0 0")
  (send *ri* :go-pos -0.5 0 0)
  (print "go-pos 0 0 -90")
  (send *ri* :go-pos #f(0 0 -90))
  )

(defun demo2 ()
  (prepare-demo)
  (walk-to-trass)
  (plan-motion)
  (when (y-or-n-p "reach trass. ok?")
    (reach))
  (when (y-or-n-p "lift trass. ok?")
    (lift))
  (when (y-or-n-p "walk & carry trass. ok?")
    (carry-trass))
  (when (y-or-n-p "put trass. ok?")
    (put))
  (when (y-or-n-p "release trass. ok?")
    (release))
  (when (y-or-n-p "walk for nex task. ok?")
    (next))
  )
