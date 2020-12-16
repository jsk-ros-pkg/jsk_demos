(defun add-door (x)
    (let ((door (make-cube 10 1000 2000)))
        (send door :translate (float-vector x 0 1000))
        (send *co* :add-attached-object door
            "base_link"
            :frame-id "base_link"
            :object-id "door")))

(defun del-door ()
    (send *co* :delete-attached-object-by-id "door")
    (send *co* :delete-object-by-id "door"))

(defun add-box (x y z)
    (let ((b2 (make-cube 310 460 460))
    (b1 (make-cube 290 440 440)))
    (send b1 :locate #f(0 0 240))
    (send b2 :locate #f(0 0 230))
    (setq b2 (body- b2 b1))
    (send b2 :locate (float-vector x y z))
    (send *co* :add-attached-object b2
        "base_link"
        :frame-id "base_link"
        :object-id "box")))

(defun del-box ()
    (send *co* :delete-attached-object-by-id "box")
    (send *co* :delete-object-by-id "box"))