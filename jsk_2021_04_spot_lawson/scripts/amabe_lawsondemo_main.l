#!/usr/bin/env roseus
(load "package://spoteus/spot-interface.l")
(load "elevator-spot.l")
(load "amabe_nazodance.l")
(spot-init nil)

(send *ri* :speak-jp "買い物に行きます")

(setq *path* "/home/amabe/spot_ws/src/jsk_demos/jsk_2021_04_spot_lawson/autowalk/amabe_73b2toele.walk")
(send *ri* :upload-path *path*)
(send *ri* :initial-localization-waypoint 0)
(send *ri* :speak-jp "出発します")
(send *ri* :navigate-to -1)

(send *ri* :speak-jp "7階エレベーター前につきました")

(elevator-spot 7 2)

(send *ri* :speak-jp "ローソンに向かいます")

(setq *path* "/home/amabe/spot_ws/src/jsk_demos/jsk_2021_04_spot_lawson/autowalk/amabe_2feleto100law.walk")
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
(send *ri* :go-pos 0 1.5 0)

(send *ri* :speak-jp "研究室へ向かいます")
(setq *path* "/home/amabe/spot_ws/src/jsk_demos/jsk_2021_04_spot_lawson/autowalk/amabe_eleto73b2.walk")

(send *ri* :upload-path *path*)
(send *ri* :initial-localization-waypoint 0)

(send *ri* :navigate-to -1)
(send *ri* :speak-jp "買ってきました")
