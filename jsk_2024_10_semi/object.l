#!/usr/bin/env irteusgl

(defclass trajectory
  :super cascaded-link
  :slots (end-coords tp1 tp2 tp3 tp4 tp5 start-coords))

(defmethod trajectory
  (:init ()
    (let (b)
      (send-super :init)
      ;; tp5
      (setq b (make-cube 1 1 1))
      (send b :locate #f(0 0 50))
      (send b :set-color :black)
      (setq tp5 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'tp5))
      (setq end-coords (make-cascoords :pos #f(0 0 20)))
      (send tp5 :assoc end-coords)
      (send tp5 :locate #f(0 0 100))
      ;; tp4
      (setq b (make-cube 1 1 1))
      (send b :locate #f(0 0 4))
      (send b :set-color :red)
      (setq tp4 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'tp4))
      (send tp4 :assoc tp5)
      (send tp4 :locate #f(0 0 4))
      ;; tp3
      (setq b (make-cube 1 1 1))
      (send b :locate #f(0 0 3))
      (send b :set-color :blue)
      (setq tp3 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'tp3))
      (send tp3 :assoc tp4)
      (send tp3 :locate #f(0 0 4))
      ;; tp2
      (setq b (make-cube 1 1 1))
      (send b :locate #f(0 0 2))
      (send b :set-color :red)
      (setq tp2 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'tp2))
      (send tp2 :assoc tp3)
      (send tp2 :locate #f(0 0 20))
      ;; tp1
      (setq b (make-cube 1 1 1))
      (send b :locate #f(0 0 1))
      (send b :set-color :white)
      (setq tp1 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'tp1))
      (send tp1 :assoc tp2)

      (setq links (list tp1 tp2 tp3 tp4 tp5))
      (send self :init-ending)
      self))

  (:end-coords (&rest args) (forward-message-to end-coords args))

  ;; 等間隔にポイントを配置するメソッド
  (:set-equal-intervals (start-coord end-coord)
    (let* ((num-points 5)  ;; tp1 から tp5 までの5点
           (start-pos (send start-coord :pos))
           (end-pos (send end-coord :pos))
           (delta-x (/ (- (aref end-pos 0) (aref start-pos 0)) (float (1- num-points))))
           (delta-y (/ (- (aref end-pos 1) (aref start-pos 1)) (float (1- num-points))))
           (delta-z (/ (- (aref end-pos 2) (aref start-pos 2)) (float (1- num-points)))))

      ;; 各ポイントを計算して配置
      (setq tp1 (send self :update-point tp1 start-pos delta-x delta-y delta-z 0))
      (setq tp2 (send self :update-point tp2 start-pos delta-x delta-y delta-z 1))
      (setq tp3 (send self :update-point tp3 start-pos delta-x delta-y delta-z 2))
      (setq tp4 (send self :update-point tp4 start-pos delta-x delta-y delta-z 3))
      (setq tp5 (send self :update-point tp5 start-pos delta-x delta-y delta-z 4))

      ;; 更新されたターゲットポイントをリンクに再設定
      (send tp1 :assoc tp2)
      (send tp2 :assoc tp3)
      (send tp3 :assoc tp4)
      (send tp4 :assoc tp5)
      self))

  ;; 各ターゲットポイントを更新するサブルーチン
  (:update-point (point start-pos delta-x delta-y delta-z index)
    (let* ((new-pos (list (+ (aref start-pos 0) (* delta-x index))
                         (+ (aref start-pos 1) (* delta-y index))
                         (+ (aref start-pos 2) (* delta-z index)))))
      ;; new-pos をベクターとして変換
      (setq new-pos (vector (aref new-pos 0) (aref new-pos 1) (aref new-pos 2)))  ;; ベクターに変換
      (send point :locate new-pos)
      point))
  )

;; 使用例
(setq r (instance trajectory :init))
(setq start-coord (make-cascoords :pos #f(0 0 0)))
(setq end-coord (make-cascoords :pos #f(0 0 100)))
;(send r :set-equal-intervals start-coord end-coord)
(objects (list r (send r :end-coords)))
(send *irtviewer* :draw-objects)




#|
(defclass trajectory
  :super cascaded-link
  :slots (end-coords tp1 tp2 tp3 tp4 tp5))

(defmethod trajectory
  (:init ()
    (let (b)
      (send-super :init)
      ;; tp5
      (setq b (make-cube 1 1 1))
      (send b :locate #f(0 0 50))
      (send b :set-color :black)
      (setq tp5 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'tp5))
      (setq end-coords (make-cascoords :pos #f(0 0 20)))
      (send tp5 :assoc end-coords)
      (send tp5 :locate #f(0 0 100))
      ;; tp4
      (setq b (make-cube 1 1 1))
      (send b :locate #f(0 0 4))
      (send b :set-color :red)
      (setq tp4 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'tp4))
      (send tp4 :assoc tp5)
      (send tp4 :locate #f(0 0 4))
      ;; tp3
      (setq b (make-cube 1 1 1))
      (send b :locate #f(0 0 3))
      (send b :set-color :blue)
      (setq tp3 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'tp3))
      (send tp3 :assoc tp4)
      (send tp3 :locate #f(0 0 4))
      ;; tp2
      (setq b (make-cube 1 1 1))
      (send b :locate #f(0 0 2))
      (send b :set-color :red)
      (setq tp2 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'tp2))
      (send tp2 :assoc tp3)
      (send tp2 :locate #f(0 0 20))
      ;; tp1
      (setq b (make-cube 1 1 1))
      (send b :locate #f(0 0 1))
      (send b :set-color :white)
      (setq tp1 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'tp1))
      (send tp1 :assoc tp2)
      
      (setq links (list tp1 tp2 tp3 tp4 tp5))
      (send self :init-ending)
      self))
  (:end-coords (&rest args) (forward-message-to end-coords args))
  )

(setq r (instance trajectory :init))
(objects (list r (send r :end-coords)))
(send *irtviewer* :draw-objects)
