(load "package://jsk_maps/src/eng2-scene.l")

(warn "goto-front-of-fridge")
(defun goto-front-of-fridge ()
  (speak-jp "いどう します");;
  (let ((co (send *scene* :spot "/eng2/7f/room73B2-fridge-front")))
    (send co :translate (float-vector 0 0 0) :world) ;; for opening door
    (send *ri* :move-to co)
    (send *ri* :go-pos 0 -0.04 0) ;; ???
    ;;
    (send *pr2* :head :angle-vector (float-vector 0 0))
    (send *pr2* :torso :angle-vector (float-vector 50))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    ;;(send *pr2* :move-to (send *ri* :state :worldcoords) :world)
    t
    ))



