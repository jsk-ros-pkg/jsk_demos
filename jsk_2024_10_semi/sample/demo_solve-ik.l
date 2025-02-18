#!/usr/bin/env roseus

;; PR2のモデルを読み込む
(require "package://pr2eus/pr2.l")

;; PR2のインスタンスを作成
(if (not (boundp '*pr2*)) (setq *pr2* (pr2)))

;; 1辺200mmの立方体を出現させる
(setq *cube* (make-cube 200 200 200)) ;; NOTE 実際はハンガーなど，デモに使用するモデルを作って出現させるとよいかもしれない
;; 立方体を(400, 0, 800)移動
(send *cube* :translate (float-vector 400 0 800)) ;; NOTE 画像認識結果から，ハンガーの位置を取得して移動するとよいかもしれない

;; 立方体の左側面に座標系を設定
(send *cube* :put :left-coords
      (make-cascoords
       :coords (send (send *cube* :copy-worldcoords) :translate (float-vector 0 100 0))
       :parent *cube*))

;; ビューワを表示
(objects (list *pr2* *cube*))

;; 左腕について，立方体の両側面に向かって逆運動学を解く
(send *pr2* :larm :inverse-kinematics
      (send (send *cube* :get :left-coords) :copy-worldcoords)
      :rotation-axis :z)

;; PR2の左腕で立方体を持ち上げて前に突き出す
;; PR2の左腕と立方体の左側を連結する．
(send *pr2* :larm :end-coords :assoc *cube*)
;; 立方体が現在の位置から相対で(100, 0, 200)に移動するように逆運動学を解く
(send *pr2*
      :inverse-kinematics (send (send (send *cube* :get :left-coords) :copy-worldcoords)
                                      :translate (float-vector 100.0 0.0 100.0))
      :move-target (send *cube* :get :left-coords)
      :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
      :rotation-axis t
      :debug-view t)

;;(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
;;(send *ri* :wait-interpolation)


;; NOTE 両腕で持って渡すみたいな動作は，結構難しい? ref: https://github.com/euslisp/jskeus/issues/582
;; :assocは閉リンクを形成できないので，1つの箱に両腕を連結することはできない?
