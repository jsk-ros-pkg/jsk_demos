(load "package://pr2eus/pr2-interface.l")
(pr2-init)
(objects (list *pr2*))

(send *ri* :speak-en "Hello!")
(send *ri* :speak-jp "こんにちは")
