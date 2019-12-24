(require :pr2-microwave-interface "package://microwave_pr2_201710/euslisp/lib/pr2-microwave-interface.l")

(defun demo
    ()
  (setq *pmi* (instance pr2-microwave-interface :init t))

  (send *pmi* :go-to-microwave-roughly)
  (send *pmi* :go-to-microwave-accurately)
  (send *pmi* :open-microwave t)
  (send *pmi* :put-object t)
  (send *pmi* :close-microwave t)
  )
