(load "/home/amabe/prog/rcb4eus/rosrcb4.l")

(make-kxr-robot "kxrl2l5a3h2g")

(send *ri* :ros-open)

(send *ri* :timer-on)

(send *ri* :hold-all)

(init-publish)
