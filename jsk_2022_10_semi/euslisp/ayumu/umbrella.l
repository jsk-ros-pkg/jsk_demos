;; ファイルの読み込み
(load "package://spoteus/spot-interface.l")

;; spotオブジェクトの作成
(spot-init)
;;(setq *spot* (instance spot-robot :init))
(objects (list *spot*))

(send *ri* :speak-jp "お散歩に行きましょう")

;; 傘を持つ姿勢
(send *spot* :angle-vector #f(0 45 -90 0 45 -90 0 45 -90 0 45 -90 90 -100 70 0 30 -90 -80))

(send *ri* :angle-vector (send *spot* :angle-vector) 8000)
(send *ri* :wait-interpolation)

;; 手の開閉
(send *ri* :gripper-open)
(send *ri* :wait-interpolation)

(send *ri* :speak-jp "傘をセットしてください")
(unix::sleep 3)

(send *ri* :gripper-close)
(send *ri* :wait-interpolation)

;: spotの手先座標系をコピーしend-coordsに代入
(setq end-coords (send (send *spot* :arm :end-coords) :copy-worldcoords))
(objects (list *spot* end-coords))

;; end-coordsを絶対座標系におけるz軸正の方向に100mm平行移動する
(send end-coords :translate #f(0 0 100) :world)

(send *spot* :inverse-kinematics
      end-coords    ;; 目標位置/姿勢
      :move-target (send *spot* :arm :end-coords)    ;; 動かすもの
      :revert-if-fail t	
      :rotational-axis :z
      :translation-axis t)

(send *ri* :angle-vector (send *spot* :angle-vector) 5000)
(send *ri* :wait-interpolation)

;; end-coordsを絶対座標系におけるz軸負の方向に600mm平行移動する
(send end-coords :translate #f(0 0 -600) :world)

(send *spot* :inverse-kinematics
      end-coords
      :move-target (send *spot* :arm :end-coords)
      :revert-if-fail t
      :rotational-zxis :z
      :translation-axis t)

(send *ri* :angle-vector (send *spot* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *spot* :angle-vector #f(0 45 -90 0 45 -90 0 45 -90 0 45 -90 90 -100 70 0 30 -90 -80))

(send *ri* :angle-vector (send *spot* :angle-vector) 5000)
(send *ri* :wait-interpolation)

;; end-coordsを絶対座標系におけるx軸負の方向に600mm平行移動する
(send end-coords :translate #f(-100 0 500) :world)

(send *spot* :inverse-kinematics
      end-coords
      :move-target (send *spot* :arm :end-coords)
      :revert-if-fail t
      :rotational-zxis :z
      :translation-axis t)

(send *ri* :angle-vector (send *spot* :angle-vector) 5000)
(send *ri* :wait-interpolation)
