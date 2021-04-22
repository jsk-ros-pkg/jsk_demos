(load "package://spoteus/spot-interface.l")

(spot-init nil)

(setq *path* "/home/spot/デスクトップ/73B2toElevator.walk")

(send *ri* :upload-path *path*)
(send *ri* :initial-localization-waypoint 0)

;;(send *ri* :initial-localization-fiducial)
(send *ri* :navigate-to -1)

;;エレベータに乗る

(setq *path* "/home/spot/デスクトップ/2021-04-20 164658 2felevto100rho.walk")
(send *ri* :upload-path *path*)
(send *ri* :initial-localization-waypoint 0)

(send *ri* :navigate-to -1)

(unix::sleep 5)

(send *ri* :navigate-to 0)
