;;棚から箱を取ってくる動作

;;roscd pr2eus_moveit
(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
(if (not (boundp '*fetch*))
    (progn (load "package://fetcheus/fetch-interface.l")
	    (fetch-init)))

;;棚の幾何モデル
;;(defun init-mode ()
;;    (setq shelf-box (make-cube 330 580 1810))
;;    (setq shelf-hole (make-cube 320 540 880))
;;    (setq box (make-cube 150 50 250))
;;    (setq )

;;十分離れた位置から実行せよ
(defun pickup-box()
    ;;初期姿勢
    (print 'init)
    (send *ri* :angle-vector (send *fetch* :reset-pose) 6000)
    (send *ri* :stop-grasp)
    (send *ri* :wait-interpolation)

    ;;初期姿勢
    


    ;;腕つかみポーズ
    (print 'posing...)
    (send *fetch* :angle-vector #f(50.3086 56.4437 -39.4177 -118.352 125.011 135.3 73.3229 -94.175 -0.260599 15.919))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 12000)
    (send *ri* :wait-interpolation)
    ;;近づく
    (send *ri* :go-pos 0.3 0 0)
    (send *ri* :wait-interpolation)
    ;;腕のばす
    (print 'extending the arm)
    (send *fetch* :angle-vector #f(50.8792 56.4312 -39.3804 -118.334 124.978 -224.734 73.3216 -94.1375 -0.26059 15.919))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 6000)
    (send *ri* :wait-interpolation)
    (print 'catching)
    ;;キャッチ
    (send *fetch* :start-grasp)
    (send *ri* :wait-interpolation)
    ;;バック
    (send *ri* :go-pos -0.3 0 0)
    (send *ri* :wait-interpolation)
    ;;箱に突っ込む
)

