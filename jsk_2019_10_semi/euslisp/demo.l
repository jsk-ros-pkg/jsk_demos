;;
(print ";; (demo) start demo")

(load "package://peppereus/pepper.l")
(setq *pepper* (pepper))

(defun demo-init ()
  (setq *white-bag* (body+ (make-sphere 300) (make-cylinder 40 300 :pos #f(0 10 250))))
  (send *white-bag* :set-color :white)
  (send *white-bag* :locate #f(-450 100 50))
  (send *white-bag* :rotate (deg2rad 70) :y)
  (objects (list *pepper* *white-bag*))
  )

(defun listen-wish ()
  (let ((wish "ps4"))
    (format t ";; listen wish~%")
    (format t ";; listened wish ~A~%" wish)
    wish))

(defun change-costume ()
  (format t ";; change costume~%"))

(defun move-to-present-shelf ()
  (format t ";; move-to...~%"))

(defun pick-present (present)
  (format t ";; pick present ~A~%" present)
  (format t ";; put present into bag~%"))

(defun move-to-child-room ()
  (format t ";; move-to child-room~%"))

(defun put-present-into-sock ()
  (format t ";; put present into sock~%"))

(defun demo ()
  (let (present)
    (demo-init)
    (format t "start demo~%~%~%")
    (setq present (listen-wish))
    (change-costume)
    (move-to-present-shelf)
    (pick-present present)
    (move-to-child-room)
    (put-present-into-sock)
    ))


(defun yoshi())
