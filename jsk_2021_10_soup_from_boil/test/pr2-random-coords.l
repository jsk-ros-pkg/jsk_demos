(defun random-coords (initial-cds &key (x 100) (y 50) (theta 30))
  ;; initail-cdsの座標をx y thetaで指定した範囲内でランダムに移動させた座標を返すプログラム
  (let (ans-cds x-num y-num theta-num)
    (setq ans-cds (send initial-cds :copy-worldcoords))
    (setq x-num (random x))
    (if (= (random 2) 1)
        (setq x-num (* x-num -1)))
    (setq y-num (random y))
    (if (= (random 2) 1)
        (setq y-num (* y-num -1)))
    (setq theta-num (random theta))
    (if (= (random 2) 1)
        (setq theta-num (* theta-num -1)))
    (send ans-cds :translate (float-vector x-num y-num 0) :world)
    (send ans-cds :rotate (deg2rad theta-num) :z :world)
    ans-cds))

(defun make-random-cds-list (initial-cds &key (x 100) (y 50) (theta 30) (num 10))
  ;; TODO？指定した範囲の境界部分，最大最小？四隅？とかは絶対に入るようにする？回転はどうしようか．
  (let (ans-cds-list list)
    (dotimes (i num)
      (setq ans-cds-list (append ans-cds-list (list (random-coords initial-cds :x x :y y :theta theta))))
      )
    ans-cds-list))

;; ;; 使用例
;; (setq pr2-spot-list (make-random-cds-list (make-coords :pos #f(1050 8500 30000) :rpy (float-vector pi 0 0)) :x 10 :y 10 :theta 5 :num 30)) ;; 前後左右50mm，回転±10degの範囲内でランダムに30箇所でチェック（今は範囲は適当）
;; (dolist (spot pr2-spot-list)
;;   (send *pr2* :move-to (send spot :copy-worldcoords) :world)
;;   ;; do ik-check
;;   )

