;;roseus を起動
;; (load "fetch_grasp_box.l")
;; (grasp-box)
;;で実行


;;無い場合はパッケージの読み込み
(if (not (boundp '*fetch*))
    (progn (load "package://fetcheus/fetch-interface.l")
	    (fetch-init)))

;;fetch君と机と箱を表示
(defun init-mode ()
  (setq box1 (make-cube 150 50 250));;箱は15cm×25cm×5cmくらい
  (setq desk (make-cube 500 700 700));;机
  (send desk :locate #f(1000 0 350))
  (send box1 :locate #f(1000 0 825))
  (objects (list *fetch* desk box1)))

(init-mode)

(defun grasp-box ()
  ;;初期姿勢
  (print 'first-pose)
  (send *fetch* :angle-vector #f(12.436 88.5458 -41.3513 2.95901 -58.6368 -0.820709 -28.3226 -4.37281 0.21526 13.9634))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  ;;上段の構え
  (print 'joudan)
  (send *fetch* :angle-vector #f(298.136 43.1722 -39.9011 62.2852 -56.835 81.379 -104.018 -40.364 0.305897 13.8975))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  ;;(send *ri* :go-pos 0.5 0 0)
  ;;つかむ
  (print 'grasp)
  (send *fetch* :angle-vector #f(298.136 34.9764 -4.37133 65.8667 -56.7911 95.2437 -108.347 -40.2761 0.48717 13.7657))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 6000)
  (send *ri* :wait-interpolation)
  (send *ri* :go-grasp :pos 0 :effort 70)
  (send *ri* :wait-interpolation)
  ;;離す
  (print 'release)
  (send *fetch* :angle-vector #f(298.159 70.8139 -40.2307 69.0308 -37.587 78.171 -121.399 -40.3421 0.079298 14.1392))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (unix::sleep 3)
  (send *ri* :go-grasp :pos 0.10)
  (send *ri* :wait-interpolation)
  ;;リセットポーズ
  (print 'reset)
  (send *fetch* :reset-pose)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
  (send *ri* :wait-interpolation))
  
