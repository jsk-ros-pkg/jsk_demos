#!/usr/bin/env roseus
;; -*- coding: utf-8 -*-

(load "package://fetcheus/fetch-interface.l")
(unless (boundp '*ri*) (fetch-init))

(defun catch-hat ()
  (send *fetch* :angle-vector #f(5.52373 52.5326 3.4729 77.8638 -59.6255 10.2535 -78.2225 74.2234 0.022649 2.23)))

(defun raise-hand ()
  (send *fetch* :angle-vector #f(5.56187 88.5458 -44.9768 -1.1279 -58.6368 -7.30264 -35.244 74.1355 0.294559 2.44973)))

(defun last-pose ()
  (send *fetch* :angle-vector #f(5.56952 88.5897 -42.0105 0.87161 -59.8672 3.90341 -76.157 7.51439 0.27191 2.42776)))

(defun grasp-finger ()
  (send *ri* :go-grasp :pos 0.03))

(defun release-finger ()
  (send *ri* :stop-grasp))

(defun reset-pose ()
       ;;(send *fetch* :reset-pose)
       (send *ri* :angle-vector (send *fetch* :reset-pose) 10000)))

(defun wear-hat ()
  (send *ri* :speak-jp "帽子をかぶります")
  (send *ri* :angle-vector (catch-hat) 10000)
  (send *ri* :wait-interpolation)
  (unix:sleep 3)
  (grasp-finger)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (raise-hand) 7000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (last-pose) 5000)
  (send *ri* :wait-interpolation)
  (release-finger)
  (send *ri* :wait-interpolation)
  ;;元に戻る。
  (send *ri* :angle-vector (last-pose) 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (raise-hand) 7000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (catch-hat) 7000)
  (send *ri* :wait-interpolation)
  (reset-pose)
  (send *ri* :wait-interpolation))

       


  
