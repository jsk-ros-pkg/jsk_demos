#!/usr/bin/env roseus

;; This is a test code for template matching
(load "package://spotkinovaeus/spotkinova-interface.l")
(spotkinova-init)

(send *ri* :speak-jp "パターンマッチのテスト中" :wait t)
