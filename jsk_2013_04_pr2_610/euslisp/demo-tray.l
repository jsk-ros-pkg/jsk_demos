#!/usr/bin/env roseus

(load "package://jsk_2013_04_pr2_610/euslisp/setup.l")

(setup)
(pr2-reset-pose)
(while (not (move-to-table)))
(pick-tray)
(while (not (move-to-sink)))
(place-tray)

(send *ri* :go-pos-unsafe -0.3 0)
(send *ri* :go-pos-unsafe 0 0 180)

(speak-jp "おわりました")