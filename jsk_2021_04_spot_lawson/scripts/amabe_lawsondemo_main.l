#!/usr/bin/env roseus
(load "package://spoteus/spot-interface.l")
(load "elevator-spot.l")

(spot-init nil)

(send *ri* :speak-jp "買い物に行きます")

(setq *path* "/home/spot/デスクトップ/73B2toElevator.walk")
(send *ri* :upload-path *path*)
(send *ri* :initial-localization-waypoint 0)
(send *ri* :speak-jp "出発します")
(send *ri* :navigate-to -1)

(send *ri* :speak-jp "7階エレベーター前につきました")

(elevator-spot 7 2)

;;go ahead to recognize tag
(send *ri* :go-pos -3.0 0 0)

(send *ri* :speak-jp "ローソンに向かいます")

(setq *path* "/home/spot/デスクトップ/2021-04-20 164658 2felevto100rho.walk")
(send *ri* :upload-path *path*)
(send *ri* :initial-localization-waypoint 0)
(send *ri* :navigate-to -1)


;;do something in lawson
(send *ri* :speak-jp "ローソンに着きました")
(unix::sleep 5)

(send *ri* :speak-jp "エレベータ前に向かいます")
(send *ri* :navigate-to 0)

(send *ri* "エレベータに乗ります")
(elevator-spot 2 7)

(send *ri* "研究室へ向かいます")
(setq *path* "/home/spot/デスクトップ/73B2toElevator.walk")

(send *ri* :upload-path *path*)
(send *ri* :initial-localization-waypoint -1)

(send *ri* :navigate-to 0)
(send *ri* :speak-jp "買ってきました")
