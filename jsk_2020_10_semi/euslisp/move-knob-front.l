; (move-to-place "/eng2/7f/room73B2-fetch-dock-front")

;; move to in front of 73A3
(move-to-place "/eng2/7f/room73A3-front")
(send *ri* :wait-interpolation)

(setq *door-mark* (send (send *fetch* :copy-worldcoords) :transform (search-mark "/sample1")))
(setq *knob-front* 
    (send 
        (send *fetch* :copy-worldcoords)
    :locate (float-vector (- (- (elt (send *fetch* :worldpos) 0) (elt (send *door-mark* :worldpos) 0)) 750)
                          (- (elt (send *fetch* :worldpos) 1) (elt (send *door-mark* :worldpos) 1))
                          0)))


; (send *door-mark* :draw-on :flush t :size 1000 :color #f(1 0 0)))

