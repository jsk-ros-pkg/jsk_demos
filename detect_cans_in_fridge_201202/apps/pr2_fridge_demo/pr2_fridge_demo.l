#!/usr/bin/env roseus

(load "package://detect_cans_in_fridge_201202/euslisp/main.l")

(init)
(send *ri* :clear-costmap)
(demo)
(exit)
