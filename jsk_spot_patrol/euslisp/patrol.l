#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")

(defun eng2-7FEvFront-behavior ()
  (send *ri* :go-to-spot "eng2_7FElevator")
  ;; TODO
  ;; look at trashcan
  )

(defun eng2-3FEvFront-behavior ()
  (send *ri* :go-to-spot "eng2_3FElevator")
  ;; TODO
  ;; look at trashcan?
  )
(defun eng2-2FEvFront-behavior ()
  (send *ri* :go-to-spot "eng2_2FElevator")
  ;; TODO
  ;; look at trashcan
  )

(defun eng2-MechOffice-behavior ()
  (send *ri* :go-to-spot "eng2_Mech_Office")
  ;; take a photo of bulletin boards
  ;; look at post
  )

(defun eng2-73A4-behavior ()
  (send *ri* :go-to-spot "eng2_73A4")
  )

(defun hongo-maingate-behavior ()
  (send *ri* :go-to-spot "HongoMainGate"))

(defun eng2-patrol ()
  (eng2_7FEvFront_Behavior)
  (eng2_3FEvFront_Behavior)
  (eng2_MechOffice_Behavior)
  (eng2_2FEvFront_Behavior)

  ;;(eng2_81C1_Behavior)
  )

(defun hongo-patrol ()
  (send *ri* :go-to-spot "eng2_MainEntrance_upper")
  (hongo-maingate-behavior)
  )
