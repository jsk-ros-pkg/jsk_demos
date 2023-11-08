(load "package://fetcheus/fetch-interface.l")
(fetch-init)
(objects (list *fetch*))

(defun aiin (&optional (tm 10000))
  (send *fetch* :angle-vector #f(10 50 30 -10 -130 30 -90 0 30 0 0))
  (objects (list *fetch*))
  (send *ri* :angle-vector (send *fetch* :angle-vector) tm)
  (send *ri* :wait-interpolation)))

(defun guts_pose (&optional (tm 10000))
  (send *fetch* :angle-vector #f(10 30 30 0 -90 0 -90 0 0 0 0))
  (objects (list *fetch*))
  (send *ri* :angle-vector (send *fetch* :angle-vector) tm)
  (send *ri* :wait-interpolation)))

(defun nuetral_pose (&optional (tm 10000))
  (send *fetch* :angle-vector #f(10 30 30 0 -90 90 -90 0 0 0 0))
  (objects (list *fetch*))
  (send *ri* :angle-vector (send *fetch* :angle-vector) tm)
  (send *ri* :wait-interpolation))

(defun junken ()
  (setq myhand (random 3))
  (nuetral_pose)
  (print "最初はグー、じゃんけん")
  (print "0:グー、1:チョキ、2:パー")
  (setq ophand (read))
  (print "ポン!")
  (cond
   ((= myhand 0) (print "グー"))
   ((= myhand 1) (print "チョキ"))
   ((= myhand 2) (print "パー")))
   
  (setq res (mod (- myhand ophand) 3))
  (cond
   ((= res 0) (nuetral_pose))
   ((or (= res 1) (= res -2)) (aiin))
   ((or (= res -1) (= res 2)) (guts_pose))
    (t (print "なんやこれ"))))

