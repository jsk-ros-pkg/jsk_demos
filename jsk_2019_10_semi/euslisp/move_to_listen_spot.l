(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")
(if (not (boundp '*fetch*))
    (progn (load "package://fetcheus/fetch-interface.l")
           (fetch-init)
           (objects (list *fetch*))
           )
          )

(send *ri* :speak-jp "子供部屋まで移動します")

(go-to-spot "/eng2/7f/room73B2-door-front" (make-coords :pos #f(200 0 0)))
(unix:sleep 1)
(go-to-spot "/eng2/7f/room73A3-front")
