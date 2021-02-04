(load "package://fetcheus/fetch-interface.l")
(fetch-init)

(send *ri* :angle-vector (send *fetch* :angle-vector) 2000 :head-controller)
