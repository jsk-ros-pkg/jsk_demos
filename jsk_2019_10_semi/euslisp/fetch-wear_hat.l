#!/usr/bin/env roseus
;; -*- coding: utf-8 -*-

(load "package://fetcheus/fetch-interface.l")
;;unless=when not
(unless (boundp '*ri*) (fetch-init))
(objects (list *fetch*))

(defun before-catch ()
  (send *fetch* :angle-vector #f(102.021 58.0258 3.4729 77.8199 -59.5596 10.3194 -78.2664 74.1575 0.305897 15.3257)))
  
(defun catch-hat ()
  (send *fetch* :angle-vector #f(102.021 52.4447 3.62671 77.8638 -59.5816 10.2755 -78.2444 74.0916 0.362546 16.8858)))
	
(defun raise-hand ()
  (send *fetch* :angle-vector #f(9.65126 88.5018 -44.9988 3.72806 -58.6148 3.74963 -33.0027 -4.39479 0.283234 14.4029)))
 
(defun wear-hat ()
  (send *fetch* :angle-vector #f(12.4436 88.5897 -41.2854 2.98099 -58.6807 -0.908599 -83.1883 -4.35084 0.362546 13.9854)))

(defun after-wear ()
  (send *fetch* :angle-vector #f(12.4436 88.5238 -41.2854 2.95901 -58.6368 -0.820709 -28.2786 -4.35084 0.203922 13.9854)))

(defun reset-pose ()
  (send *ri* :angle-vector (send *fetch* :reset-pose) 10000))

(defun grasp-finger ()
  ;;effortが足りていればstart-graspでよい。
  ;;(send *ri* :start-grasp :wait t))
  (send *ri* :go-grasp :effort 70 :pos 0 :wait t))

(defun release-finger ()
  ;;(send *ri* :stop-grasp :wait t))
  (send *ri* :go-grasp :effort 70 :pos 10 :wait t))


(defun fetch-wear-hat ()
  (release-finger)
  (reset-pose)
  ;;(send *ri* :speak-jp "帽子をかぶります")
  (send *ri* :angle-vector (before-catch) 7000)
  (send *ri* :wait-interpolation)
  ;;赤色を認識し、帽子が置いてある机まで移動
  (send *ri* :angle-vector (catch-hat) 6000)
  (send *ri* :wait-interpolation)
  (unix:sleep 3)
  (grasp-finger)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (raise-hand) 10000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (wear-hat) 6000)
  (send *ri* :wait-interpolation)
  (release-finger)
  (send *ri* :wait-interpolation)
  ;;机から離れる、あるいは向きを変えるなどして腕が机にぶつからないようにする。
  (send *ri* :angle-vector (after-wear) 7000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (catch-hat) 10000)
  (send *ri* :wait-interpolation)
  (reset-pose)
  )
