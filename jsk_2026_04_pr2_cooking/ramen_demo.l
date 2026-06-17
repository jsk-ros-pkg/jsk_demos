#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus/pr2.l")
(load "models/room73b2-scene.l")
(pr2-init)

(setq *73b2* (room73b2))

(ros::roseus "motomochi")
(ros::load-ros-manifest "jsk_recognition_msgs")

(objects (list *pr2* *73b2*))

(defun apply-angle-vector (filename)
(load (concatenate string "angle_vector/" filename ".l"))
(setq av (symbol-value (intern (string-upcase filename))))
(send *pr2* :angle-vector av)
;; (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
;; (send *ri* :wait-interpolation)
)

;; 初期姿勢(腕を上げた位置)
(apply-angle-vector "initial_uphand_pose.l")

;; キッチン前(sink0、おそらくIHの前?)の位置を設定
(setq *sink0-model* (make-cascoords :pos #f(1175 1559 300) :rpy (list pi 0 0)))
(setq *sink0* (make-cascoords :pos #f(1175 8459 300) :rpy (list pi 0 0)))

(setq *sink-front* (make-cascoords :pos #f(1355 9350 300) :rpy (list pi 0 0)))
;; (send *ri* :move-to *sink-front* :frame "/map")

;; キッチンの前まで移動
(send *pr2* :move-to *sink0-model* :world) 
;; モデル上で確認。モデル上と実際は、y軸方向に6900mmだけずれている
(send *ri* :move-to *sink0* :frame "/map")

;; 火をつける
(apply-angle-vector "fire-on")

;; 麺が入った皿の上まで右アームを移動
(apply-angle-vector "above-noodle-plate")

;; 麺が入った皿を掴む
(apply-angle-vector "grab-noodle-plate")

;; 鍋の上まで移動
(apply-angle-vector "above-pot")

;; 麺を入れる
(apply-angle-vector "drop-noodle")

;; 麺をほぐす(後で追加)

;; 皿を置く
(apply-angle-vector "put-plate")

;; 皿を離す
(apply-angle-vector "release-plate")

;; 火を止める
(apply-angle-vector "fire-off")

;; 両手鍋を掴む
(apply-angle-vector "grab-double-handled-pot")

;; グリッパを閉じる
(send *ri* :start-grasp)

;; 両手を同時に上に上げる(計120mm)
(send *pr2* :rarm :move-end-pos #f(0 0 50) :world)
(send *pr2* :larm :move-end-pos #f(0 0 50) :world)
(send *pr2* :rarm :move-end-pos #f(0 0 30) :world)
(send *pr2* :larm :move-end-pos #f(0 0 30) :world)
(send *pr2* :rarm :move-end-pos #f(0 0 30) :world)
(send *pr2* :larm :move-end-pos #f(0 0 30) :world)
(send *pr2* :rarm :move-end-pos #f(0 0 10) :world)
(send *pr2* :larm :move-end-pos #f(0 0 10) :world)
;; (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
;; (send *ri* :wait-interpolation)

;; 右に300mm移動
(send *ri* :go-pos 0 -0.3 0)
(send *ri* :wait-interpolation)

;; 鍋を傾けてラーメンを皿に入れる
(setq rav (send *pr2* :rarm :angle-vector))
(setq lav (send *pr2* :larm :angle-vector))

(setf (elt rav 4) (+ (elt rav 4) 10.0))
(setf (elt lav 4) (+ (elt lav 4) -10.0))

(setf (elt rav 6) (+ (elt rav 6) -60.0))
(setf (elt lav 6) (+ (elt lav 6) 60.0))

(send *pr2* :rarm :angle-vector rav)
(send *pr2* :larm :angle-vector lav)

;; (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
;; (send *ri* :wait-interpolation)

;; 鍋の傾きを戻す
(setq rav (send *pr2* :rarm :angle-vector))
(setq lav (send *pr2* :larm :angle-vector))

(setq rav (send *pr2* :rarm :angle-vector))
(setq lav (send *pr2* :larm :angle-vector))

(setf (elt rav 4) (+ (elt rav 4) -10.0))
(setf (elt lav 4) (+ (elt lav 4) 10.0))

(setf (elt rav 6) (+ (elt rav 6) 60.0))
(setf (elt lav 6) (+ (elt lav 6) -60.0))

(send *pr2* :rarm :angle-vector rav)
(send *pr2* :larm :angle-vector lav)

;; (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
;; (send *ri* :wait-interpolation)

;; 鍋をコンロの上に戻す
(send *ri* :go-pos 0 0.3 0)
(send *ri* :wait-interpolation)

;; 鍋を置く
(send *pr2* :rarm :move-end-pos #f(0 0 -120) :world)
(send *pr2* :larm :move-end-pos #f(0 0 -120) :world)
;; (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
;; (send *ri* :wait-interpolation)

