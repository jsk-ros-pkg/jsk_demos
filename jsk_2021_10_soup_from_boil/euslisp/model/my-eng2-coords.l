;; my spot
(setq *kitchen-spot* (make-coords :pos (float-vector 1200 8500 30000) :rpy (float-vector pi 0 0)))
(setq *kitchen-spot* (make-coords :pos (float-vector 1110 8500 30000) :rpy (float-vector pi 0 0)))
(setq *sink-spot* (make-coords :pos (float-vector 1110 9100 30000) :rpy (float-vector pi 0 0)))
(setq *stove-spot* (make-coords :pos (float-vector 1200 8100 30000) :rpy (float-vector pi 0 0)))
;; (setq *microwave-spot* (make-coords :pos (float-vector 1600 9300 30000) :rpy (float-vector 0 0 0)))
(setq *microwave-spot* (make-coords :pos (float-vector 1600 8500 30000) :rpy (float-vector 0 0 0)))
(setq *seasoning-spot* (make-coords :pos (float-vector 1700 8900 30000) :rpy (float-vector 0 0 0)))
(setq *arrange-spot* (make-coords :pos (float-vector 1200 8200 30000) :rpy (float-vector pi 0 0)))
(setq *garbage-spot* (make-coords :pos (float-vector 1700 9000 30000) :rpy (float-vector 0 0 0)))

;; tool coords
;; (setq *knife-coords* (make-coords :pos (float-vector 580 8715 30857) :rpy (float-vector 3.14 0.073 1.498))) ;; 包丁
(setq *knife-coords* (make-coords :pos (float-vector 600 8715 30857) :rpy (float-vector 3.14 0.073 1.498))) ;; 包丁
(setq *peeler-coords* (make-coords :pos (float-vector 580 8815 30870) :rpy (float-vector 3.14 0.049 1.51))) ;; ピーラー
(setq *potato-peeler-coords* (make-coords :pos (float-vector 580 8915 30865) :rpy (float-vector 3.14 0.013 1.545))) ;; じゃがいもピーラー
;; (setq *spatula-coords-1* (make-coords :pos (float-vector 592 8549 30872) :rpy (float-vector 3.084 -0.042 1.542))) ;; ヘラ，平置き
(setq *spatula-coords-1* (make-coords :pos (float-vector 630 8549 30872) :rpy (float-vector 3.084 -0.042 1.542))) ;; ヘラ，平置き
;; (setq *ladle-coords-1* (make-coords :pos (float-vector 599 8313 30877) :rpy (float-vector 3.128 -0.098 1.514))) ;; おたま，平置き
(setq *ladle-coords-1* (make-coords :pos (float-vector 630 8313 30877) :rpy (float-vector 3.128 -0.098 1.514))) ;; おたま，平置き

;; (setq *spatula-coords-0* (make-coords :pos (float-vector 340 8720 31450) :rpy (float-vector -3.14 -0.008 1.529))) ;; ヘラ，棚の中
(setq *spatula-coords-0* (make-coords :pos (float-vector 340 8730 31460) :rpy (float-vector -3.14 -0.008 1.529))) ;; ヘラ，棚の中
;; (setq *spatula-coords-0* (make-coords :pos (float-vector 335 8730 31465) :rpy (float-vector -3.14 -0.008 1.529))) ;; ヘラ，棚の中 ;; 位置変えた時
;; (setq *ladle-coords-0* (make-coords :pos (float-vector 346 8637 31450) :rpy (float-vector -3.14 -0.008 1.529))) ;; おたま，棚の中
(setq *ladle-coords-0* (make-coords :pos (float-vector 340 8630 31460) :rpy (float-vector -3.14 -0.008 1.529))) ;; おたま，棚の中
;; (setq *ladle-coords-0* (make-coords :pos (float-vector 335 8630 31465) :rpy (float-vector -3.14 -0.008 1.529))) ;; おたま，棚の中 ;; 位置変えた時

(setq *spatula-coords-a* (make-coords :pos (float-vector 592 8900 30872) :rpy (float-vector 3.084 -0.042 1.542))) ;; ヘラ，盛り付け時
(setq *ladle-coords-a* (make-coords :pos (float-vector 599 8700 30877) :rpy (float-vector 3.128 -0.098 1.514))) ;; おたま，盛り付け時


;; kitchenware coords
;; (setq *pot-coords* (make-coords :pos (float-vector 450 8100 30840) :rpy (float-vector 3.142 0 0)))
;; (setq *pot-coords* (make-coords :pos (float-vector 450 8050 30850) :rpy (float-vector 3.142 0 0)))
(setq *pot-coords* (make-coords :pos (float-vector 450 8080 30850) :rpy (float-vector 3.142 0 0)))

;; (setq *cutting-board-coords* (make-coords :pos (float-vector 530 8420 30840) :rpy (float-vector 0 0 0)))
(setq *cutting-board-coords* (make-coords :pos (float-vector 560 8420 30840) :rpy (float-vector 0 0 0)))
(setq *bowl-1-coords* (make-coords :pos (float-vector 325 8460 30840) :rpy (float-vector -0.175 0 0)))
(setq *bowl-2-coords* (make-coords :pos (float-vector 295 8645 30840) :rpy (float-vector -0.349 0 0)))
(setq *bowl-2-set-coords* (make-coords :pos (float-vector 510 8430 30840) :rpy (float-vector 0 0 0)))
(setq *bowl-3-coords* (make-coords :pos (float-vector 635 7585 30840) :rpy (float-vector 1.048 0 0)))
(setq *bowl-4-coords* (make-coords :pos (float-vector 480 7845 30840) :rpy (float-vector -0.175 0 0)))

(setq *bowl-1-stove-coords* (make-coords :pos (float-vector 330 8661 30840) :rpy (float-vector -0.423 0 0)))
;; (setq *bowl-1-kitchen-coords* (make-coords :pos (float-vector 279 8747 30823) :rpy (float-vector -0.021 -0.056 0.025)))
(setq *bowl-1-kitchen-coords* (make-coords :pos (float-vector 279 8747 30823) :rpy (float-vector 0 0 0)))
(setq *bowl-2-stove-coords* (make-coords :pos (float-vector 480 7845 30850) :rpy (float-vector -0.175 0 0)))
(setq *bowl-3-stove-coords* (make-coords :pos (float-vector 555 7585 30840) :rpy (float-vector 1.048 0 0)))

;; (setq *mcup-coords* (make-coords :pos (float-vector 555 7674 30850) :rpy (float-vector -0.948 0 0)))
(setq *mcup-coords* (make-coords :pos (float-vector 530 7710 30850) :rpy (float-vector -0.948 0 0)))
(setq *rcup-coords* (make-coords :pos (float-vector 320 8525 30840)))
(setq *oil-dispenser-coords* (make-coords :pos (float-vector 300 8280 30845) :rpy (float-vector 3.14 0.0 0.0))) 
(setq *oil-dispenser-stove-coords* (make-coords :pos (float-vector 400 7570 30845) :rpy (float-vector 3.14 0.0 0.0))) ;; tmp

(setq *curry-dish-coords* (make-coords :pos (float-vector 530 8400 30840) :rpy (float-vector 0 0 0)))
(setq *sato-rice-coords* (make-coords :pos (float-vector 340 8425 30840) :rpy (float-vector 0 0 0)))

(setq *solt-coords* (make-coords :pos (float-vector 2500 8788 30910) :rpy (float-vector 0 0 0)))
(setq *dispenser-coords* (make-coords :pos (float-vector 2500 8676 30900) :rpy (float-vector 0 0 0)))
(setq *white-dish-coords* (make-coords :pos (float-vector 2523 9000 30870) :rpy (float-vector 0 0 0)))
(setq *rcup-seasoning-coords* (make-coords :pos (float-vector 2500 8520 30900) :rpy (float-vector -3.14 0.0 0.0)))


;; equipment coords
;; 棚
(setq *shelf-knob-closed-coords* (make-coords :pos (float-vector 425 8573 31445) :rpy (float-vector 3.105 0.012 3.092)))
(setq *shelf-knob-opened-coords* (make-coords :pos (float-vector 629 8859 31445) :rpy (float-vector -1.636 -0.095 -3.091)))

;; (setq *shelf-door-coords* (make-coords :pos (float-vector 400 8845 31445) :rpy (float-vector 3.14 0 0)))
(setq *shelf-door-coords* (make-coords :pos (float-vector 400 8840 31445) :rpy (float-vector 3.14 0 0)))
;; (setq *shelf-door-coords* (make-coords :pos (float-vector 400 8835 31445) :rpy (float-vector 3.14 0 0)))

;; 水道
;; (setq *fauset-lever-rarm-coords* (make-coords :pos (float-vector 227 9190 31014) :rpy (float-vector  -2.429 0.369 1.988))) ;; 水道のレバーを掴む座標
(setq *fauset-lever-rarm-coords* (make-coords :pos (float-vector 220 9190 31014) :rpy (float-vector  -2.429 0.369 1.988))) ;; 水道のレバーを掴む座標

;; (setq *fauset-coords* (make-coords :pos (float-vector 358 9114 30900) :rpy (float-vector 3.14 0 0)))
;; (setq *fauset-coords* (make-coords :pos (float-vector 350 9114 30900) :rpy (float-vector 3.14 0 0)))
(setq *fauset-coords* (make-coords :pos (float-vector 350 9140 30900) :rpy (float-vector 3.14 0 0)))

;; コンロ
(setq *ih-switch-on-coords* (make-coords :pos (float-vector 680 8171 30750) :rpy (float-vector -3.047 -0.011 1.512)))
(setq *ih-switch-off-coords* (make-coords :pos (float-vector 680 8161 30750) :rpy (float-vector -3.047 -0.011 1.512)))
(setq *ih-knob-coords* (make-coords :pos (float-vector 680 8154 30715) :rpy (float-vector -3.047 -0.011 1.512)))

;; ゴミ箱
(setq *trashbox0-coords* (make-coords :pos (float-vector 1600 9300 30000) :rpy (float-vector 0 0 0)))
