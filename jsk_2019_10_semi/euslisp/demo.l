;;プレゼントを聞くプログラムは別で走らせておく。
(print ";;start demo")

(defun go-listen-spot ()
  (format t ";;go-listen-splot")
  (load "move_to_listen_spot.l"))

(defun change-costume ()
  (format t ";;change costume")
  (load "listen_gift.l")
  (load "go_wear-spot.l")
  (load "get_color_and_wear.l"))

(defun catch-present ()
  (format t ";;catch present")
  (load "catch_item.l"))

(defun put-present-into-sock()
  (format t ";;put present")
  (load "kanazawa.l")
  (kanazawa))


(defun demo ()
  (go-listen-spot)
  (change-costume)
  (catch-present)
  (put-present-into-sock)
  )


(format t ";;start")
(demo)
(format t ";;finish")  
