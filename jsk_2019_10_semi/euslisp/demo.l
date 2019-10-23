;;
(print ";; (demo) start demo")

(load "package://peppereus/pepper.l")
(setq *pepepr* (pepper))

(defun demo-init ()
  (setq *white-bag* (body+ (make-sphere 300) (make-cylinder 40 300 :pos #f(0 10 250))))
  (send *white-bag* :set-color :white)
  (send *white-bag* :locate #f(-450 100 50))
  (send *white-bag* :rotate (deg2rad 70) :y)
  (objects (list *pepper* *white-bag*))
  )

(defun bring-present (present)
  (format t "bring ~A~%" present))

(defun demo ()
  (demo-init)
  (format t "start demo~%~%~%")
  (bring-present "ps4"))


