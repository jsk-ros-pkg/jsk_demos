#!/usr/bin/env roseus

(load "package://jsk_2013_04_pr2_610/euslisp/setup.l")

(defun demo-all ()
  (demo)
  (speak-jp "おわりました")
  )

(defun demo-tray ()
  (setup)
  (pr2-reset-pose)

  (while (not (move-to-table-side)))
  (while (not (pick-tray-table)))
  (while (not (move-to-sink)))
  (place-tray-sink)

  (send *ri* :go-pos-unsafe -0.3 0)
  (send *ri* :go-pos-unsafe 0 0 180)

  (speak-jp "おわりました")
  )

(defun demo-wash ()
  (setup)

  (while (not (move-to-chair)))
  (pick-cloth-chair)

  (while (not (move-to-laundry)))
  (put-cloth-laundry)

  (pr2-reset-pose)

  (send *ri* :go-pos-unsafe -0.2 0)
  (send *ri* :go-pos-unsafe 0 0 -140)

  (speak-jp "おわりました")
  )

(defun demo-pick-broom ()
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

  (pick-broom-laundry)

  (send *ri* :go-pos-unsafe -0.1 0 -140)

  (speak-jp "おわりました")
  )


(defun demo-sweep ()
  (setup)
  (pr2-reset-pose)

  ;;pick broom
  (speak-jp "モップをとります")
  (while (not (move-to-broom)))
  (pick-broom-laundry)
  (send *ri* :go-pos-unsafe 0 0 -90)

  (speak-jp "部屋の掃除をします")
  (while (not (sweep-room)))

  (speak-jp "いすをひきます")
  (while (not (move-to-chair2)))
  (pull-chair-table)
  ;;

  (speak-jp "モップをかけます")
  (while (not (move-to-table-front)))
  (sweep-under-table-front)

  (while (not (move-to-laundry)))
  (place-broom-laundry)

  (speak-jp "おわりました")
  (use-tilt-laser-obstacle-cloud t)
)

