(load "package://spoteus/spot-interface.l")
(if t ;実機につなぐなら
    (spot-init)
    (setq *spot* (instance spot-robot :init))
)
;(send *ri* :speak-jp "こんにちは")

(objects (list *spot*))

;(setq pose (send *ri* :potentio-vector))

;初期姿勢
(send *spot* :reset-pose)
(send *ri* :angle-vector (send *spot* :angle-vector) 2000)

(send *ri* :gripper-open)

;手を地面付近に下ろす
(setq endcoords (send (send *spot* :arm :end-coords) :copy-worldcoords)) ;手先の座標系を取得し、コピーする
(objects (list endcoords *spot*))   ;座標系を表示

(send endcoords :rotate (/ (- pi) 2) :z)  ;座標系の回転移動
(send *spot* :inverse-kinematics endcoords :move-target (send *spot* :arm :end-coords) :rotation-axis t :revert-if-fail nil)

(send endcoords :translate #f(300 0 0) :world)  ;座標系の平行移動
(send *spot* :inverse-kinematics endcoords :move-target (send *spot* :arm :end-coords) :rotation-axis t :revert-if-fail nil)

(send endcoords :translate #f(0 0 -600) :world)
(send *spot* :inverse-kinematics endcoords :move-target (send *spot* :arm :end-coords) :rotation-axis t :revert-if-fail nil)


(send *ri* :wait-interpolation)
(send *ri* :angle-vector (send *spot* :angle-vector) 2000)

;手を対象物に持っていく
(setq endcoords (send (send *spot* :arm :end-coords) :copy-worldcoords)) ;手先の座標系を取得し、コピーする
(objects (list endcoords *spot*))   ;座標系を表示

(send endcoords :translate #f(0 0 -100) :world)
(send *spot* :inverse-kinematics endcoords :move-target (send *spot* :arm :end-coords) :rotation-axis nil :revert-if-fail nil)

(send *ri* :wait-interpolation)
(send *ri* :angle-vector (send *spot* :angle-vector) 2000)

;もとに戻る
(send *ri* :wait-interpolation)
(send *ri* :gripper-close)

(send *spot* :reset-pose)

(send *ri* :wait-interpolation)
(send *ri* :angle-vector (send *spot* :angle-vector) 2000)
