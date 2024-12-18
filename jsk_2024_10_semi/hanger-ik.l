#!/usr/bin/env roseus

;; PR2のモデルを読み込む
(require "package://pr2eus/pr2.l")
(require "package://pr2eus/pr2-utils.l")
(require "package://pr2eus/pr2-interface.l")
(require "package://pr2eus/speak.l")

(pr2-init)
;; PR2のインスタンスを作成
(if (not (boundp '*pr2*)) (setq *pr2* (pr2)))


;; 三角柱を作成（薄いハンガー形状を模擬）
(setq *hanger* (make-prism 
                (list (float-vector 0 00 200)       ;; 下の頂点
                      (float-vector 200 0 0)    ;; 右上の頂点
                      (float-vector -200 0 0))  ;; 左上の頂点
                20))  ;; 厚さ20mm

;; 三角柱を適切な位置に移動
(send *hanger* :translate (float-vector 1100 100 1100))

;; 三角柱の下の角に座標系を設定（把持の手前の位置）
(send *hanger* :put :left-coords
      (make-cascoords
       :coords (send (send *hanger* :copy-worldcoords) 
                    :translate (float-vector -220 0 0))  ;; ハンガーの下端から相対位置
              :rot #2f((0 0 1)   ;; y軸方向を向くための回転行列
                (-1 0 0)
                (0 -1 0))

       :parent *hanger*))

; (send *hanger* :put :right-coords
;       (make-cascoords
;        :coords (send (send *hanger* :copy-worldcoords)
;                     :translate (float-vector -20 0 0))  ;; ハンガーの下端から相対位置
;        :rot #2f((1 0 0) (0 1 0) (0 0 1))   ;; 地面と平行な姿勢
;        :parent *hanger*))


;; ビューワを表示
(objects (list *pr2* *hanger*))

(send *ri* :speak-jp "今日は何を着たい?")

(send *pr2* :reset-pose)
(send *pr2* :larm :collar-y :joint-angle 0)
(send *pr2* :larm :shoulder-p :joint-angle 0)
(send *pr2* :larm :shoulder-r :joint-angle 0)
(send *pr2* :larm :elbow-p :joint-angle -90)
(send *pr2* :larm :elbow-r :joint-angle 180)
(send *pr2* :larm :wrist-p :joint-angle -30)
(send *pr2* :larm :wrist-r :joint-angle 180)
(send *pr2* :larm :gripper :joint-angle 40)
(send *irtviewer* :draw-objects)

; これ書けばロボットが動く
(send *ri* :stop-grasp :arms)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)

; ;; 左腕で三角柱を掴む
(send *pr2* :larm :inverse-kinematics
      (send (send *hanger* :get :left-coords) :copy-worldcoords)
      :rotation-axis :z)

(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)

; ;; PR2の左腕で三角柱を持ち上げて前に突き出す
; (send *pr2* :larm :end-coords :assoc *hanger*)
; (send *pr2*
;       :inverse-kinematics (send (send (send *hanger* :get :left-coords) :copy-worldcoords)
;                                :translate (float-vector 100.0 0.0 100.0))
;       :move-target (send *hanger* :get :left-coords)
;       :link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
;       :rotation-axis t
;       :debug-view t)



(defun step2-grasp ()
  "ハンドを閉じる動作（シミュレーション用）"
  ;(send *pr2* :larm :gripper :joint-angle 0)
  (send *pr2* :larm :end-coords :assoc *hanger*)
  (send *ri* :start-grasp :larm)
  (send *irtviewer* :draw-objects))


; (defun step3-lift ()
;   "斜め上に移動"
;   (send *pr2* :larm :move-end-pos #f(-50 0 -50) :world
;         :debug-view nil)
;   (send *irtviewer* :draw-objects))

; (defun step4-pull ()
;   "手元に引く"
;   (send *pr2* :larm :move-end-pos #f(-300 0 0) :world
;         :debug-view nil)
;   (send *irtviewer* :draw-objects))



(defun step3-lift ()
  "斜め上に移動"
  (send *pr2*
	:larm
        :inverse-kinematics (send (send (send *hanger* :get :left-coords) :copy-worldcoords)
                                 :translate (float-vector -100 0 100))  ;; 5cm上、5cm手前
        ;;:move-target (send *hanger* :get :left-coords)
        ;;:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
        :rotation-axis t
        :debug-view nil
	)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *irtviewer* :draw-objects)
  )

(defun step4-pull ()
  "手元に引く"
  (send *pr2*
        :inverse-kinematics (send (send (send *hanger* :get :left-coords) :copy-worldcoords)
                                 :translate (float-vector -300 0 50))  ;; さらに30cm手前
        ;:move-target (send *hanger* :get :left-coords)
        ;:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
        :rotation-axis t
        :debug-view t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *irtviewer* :draw-objects))

(defun step5-release ()
  "ハンドを開く動作（シミュレーション用）"
  ;(send *pr2* :larm :gripper :joint-angle 40)
  (send *ri* :stop-grasp :larm)
  (send *irtviewer* :draw-objects)
)




; (step2-grasp)
; (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
; (send *ri* :wait-interpolation)

; (step3-lift)

; (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
; (send *ri* :wait-interpolation)

; (step4-pull)
; (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
; (send *ri* :wait-interpolation)
