(require :pr2-microwave-interface "package://microwave_pr2_201710/euslisp/lib/pr2-microwave-interface.l")

(defun demo
    ()
  (setq *pmi* (instance pr2-microwave-interface :init))
  (send *pmi* :execute)
  )
