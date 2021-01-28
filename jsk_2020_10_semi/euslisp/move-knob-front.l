;;;; require ../launch/ubereats.launch to be run

(setq *door-mark* (send (send *fetch* :copy-worldcoords) :transform (search-mark "/sample1")))
(setq *door-knob-base* 
    (send 
        (send *fetch* :copy-worldcoords)
    :locate (float-vector (- (- (elt (send *fetch* :worldpos) 0) (elt (send *door-mark* :worldpos) 0)) 750)
                          (- (- (elt (send *fetch* :worldpos) 1) (elt (send *door-mark* :worldpos) 1)) 50)
                          0)))

(send *door-knob-base* :draw-on :flush t :size 1000 :color #f(1 0 0)))
; (send *door-mark* :draw-on :flush t :size 1000 :color #f(1 0 0)))

(move-to-coord *door-knob-base*)
