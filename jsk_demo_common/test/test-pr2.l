(load "package://jsk_pr2_demo/euslisp/pr2-move.l")
(load "package://jsk_pr2_demo/euslisp/pr2-action.l")
(load "package://pr2eus/pr2-interface.l")

(unless (boundp '*pr2*) (pr2))
(unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))

(pr2-tuckarm-pose :rarm)

(goto-front-of-fridge)
(open-fridge-door)
(open-fridge)


