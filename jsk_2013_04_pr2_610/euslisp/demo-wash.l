#!/usr/bin/env roseus

(load "package://jsk_2013_04_pr2_610/euslisp/setup.l")

(setup)

(while (not (move-to-chair)))
(pick-cloth)

(while (not (move-to-laundry)))
(put-cloth)

(pr2-reset-pose)

(send *ri* :go-pos-unsafe -0.2 0)
(send *ri* :go-pos-unsafe 0 0 -140)

(speak-jp "おわりました")