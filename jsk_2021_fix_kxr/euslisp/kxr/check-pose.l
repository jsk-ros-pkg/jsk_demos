(load "/home/amabe/prog/rcb4eus/rosrcb4.l")
(make-kxr-robot "kxrl2l5a3h2g")
(send *ri* :arm2-open)
(send *ri* :timer-on)
(send *ri* :hold-all)
;;(send *ri* :hold :rarm)

;;(send *ri* :angle-vector (send *robot* :angle-vector) 100)

;;2番サーボ修理の構え
#|(send *robot* :angle-vector #f(0.0 -10.0 40.0 -20.0 0.0 0.0 -10.0 40.0 -20.0 0.0 -90.0 -90.0 0.0 -27.0 -27.0 -90.0 90.0 0.0 -27.0 -27.0 0.0 10.0))
(send *ri* :angle-vector (send *robot* :angle-vector) 100)
|#

;;3番サーボ修理の構え(8番もいけそう)
#|(send *robot* :angle-vector #f(0.0 -10.0 40.0 -20.0 0.0 0.0 -10.0 40.0 -20.0 0.0 0.0 -90.0 0.0 -27.0 -27.0 0.0 90.0 0.0 -27.0 -27.0 0.0 10.0))
(send *ri* :angle-vector (send *robot* :angle-vector) 100)
|#

#| 20211012
ここでケーブルを抜く
(send *robot* :angle-vector #f(0.0 -10.0 40.0 -20.0 0.0 0.0 -10.0 40.0 -20.0 0.0 0.0 -90.0 0.0 -27.0 -27.0 0.0 90.0 0.0 -27.0 -27.0 0.0 10.0))
(send *ri* :angle-vector (send *robot* :angle-vector) 100)

;pandaが腕をいい位置にもってく

(send *ri* :free :rarm)

;pandaがケーブルを挿す

(send *ri* :hold :rarm)

;腕が動くことを示す
(send *robot* :angle-vector #f(0.0 -10.0 40.0 -20.0 0.0 0.0 -10.0 40.0 -20.0 0.0 -90.0 -90.0 0.0 -27.0 -27.0 0.0 90.0 0.0 -27.0 -27.0 0.0 10.0))
(send *ri* :angle-vector (send *robot* :angle-vector) 100)
|#