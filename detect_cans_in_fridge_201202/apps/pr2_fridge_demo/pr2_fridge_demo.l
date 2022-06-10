#!/usr/bin/env roseus

(require :app-utils "package://jsk_demo_common/euslisp/app-utils.l")

(load "package://detect_cans_in_fridge_201202/euslisp/main.l")

(send *ri* :clear-costmap)
(run-exit (demo :type "georgia"))
