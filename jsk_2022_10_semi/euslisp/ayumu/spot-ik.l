;; ファイルの読み込み
(load "package://spoteus/spot-interface.l")

;; spotオブジェクトの作成
;;(spot-init)
(setq *spot* (instance spot-robot :init))
;;(objects (list *spot*))

;: spotの手先座標系をコピーしend-coordsに代入
(setq end-coords (send (send *spot* :arm :end-coords) :copy-worldcoords))
(objects (list *spot* end-coords))

;; end-coordsを絶対座標系におけるz軸方向に100mm平行移動する
(send end-coords :translate #f(0 0 100) :world)

(send *spot* :inverse-kinematics
      end-coords    ;; 目標位置/姿勢
      :move-target (send *spot* :arm :end-coords)    ;; 動かすもの
      :revert-if-fail t
      :rotational-axis t
      :translation-axis t)
