(load "package://jsk_maps/src/eng2-scene.l")

(unless (boundp '*scene*) (setq *scene* (make-eng2-scene)))

(warn "move-to-spot")
(defun move-to-spot (spot &key (scene  *scene*)
                          (angle-vector) (time 2000) (trajectory)
                          ((:clear-costmap clear-cmap) t)
                          (with-tilt-laser-obstacle-cloud t))
  (send *ri* :ros-wait 0.0 :spin t :spin-self t) ;; attention-check ...

  (unless with-tilt-laser-obstacle-cloud
    (use-tilt-laser-obstacle-cloud nil))

  (cond
   (angle-vector
    (send *ri* :angle-vector (send *pr2* :angle-vector) time))
   (trajectory
    (send *ri* :angle-vector-sequence
          (car trajectory) (cdr trajectory))
    ))

  (if clear-cmap (clear-costmap))
  (send *pr2* :move-to (send scene :spot spot) :world)
  (prog1 (send *ri* :move-to (send scene :spot spot))
    (unless with-tilt-laser-obstacle-cloud
      (use-tilt-laser-obstacle-cloud t)))
  )

(warn "goto-front-of-fridge")
(defun goto-front-of-fridge (&key (torso-lift 50) (head-pitch 0))
  (if (and (boundp '*use-voicetext*) *use-voicetext*)
      (cond
       ((and (boundp '*use-english*) *use-english*)
        (speak-jp "I will move to the refrigerator."))
       (t (speak-jp "移動します。")))
    (speak-jp "いどう します"));;
  (let ((co (send *scene* :spot "/eng2/7f/room73B2-fridge-front")))
    (send co :translate (float-vector 0 0 0) :world) ;; for opening door
    (send *ri* :move-to co)
    (send *ri* :go-pos 0 -0.04 0) ;; ???
    ;;
    (send *pr2* :head :angle-vector (float-vector 0 head-pitch))
    (send *pr2* :torso :angle-vector (float-vector torso-lift))
    (send *ri* :ros-wait 0.0 :spin t :spin-self t) ;; attention-check ...
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    ;;(send *pr2* :move-to (send *ri* :state :worldcoords) :world)
    t
    ))
