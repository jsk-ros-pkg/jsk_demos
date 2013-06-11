#!/usr/bin/env roseus

(load "package://jsk_2013_04_pr2_610/euslisp/setup.l")

(setup)

(pr2-reset-pose)

(while (not (move-to-laundry)))

(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(send *pr2* :head :neck-y :joint-angle -4.353)
(send *pr2* :head :neck-p :joint-angle 24.422)
(send *ri* :angle-vector (send *pr2* :angle-vector))
(send *ri* :wait-interpolation)

(send *laundry* :move-to (laundry-detection *laundry*) :world)

(while (not (move-to-broom)))

(pick-broom)

(send *ri* :go-pos-unsafe -0.1 0 -140)

(speak-jp "おわりました")