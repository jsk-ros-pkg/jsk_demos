(require :pr2-microwave-interface "package://microwave_pr2_201710/euslisp/lib/pr2-microwave-interface.l")

(defun demo
    ()
  ;; init
  (setq *pmi* (instance pr2-microwave-interface :init))

  ;; go to microwave
  (send *pmi* :go-to-microwave-roughly)
  (send *pmi* :go-to-microwave-accurately)

  ;; put object in microwave
  (send *pmi* :open-microwave)
  (send *pmi* :go-forward)
  (send *pmi* :put-object)
  (send *pmi* :go-backward)
  (send *pmi* :close-microwave)

  ;; push button
  (send *pmi* :go-to-microwave-accurately)
  (send *pmi* :push-heat-button)
  (send *pmi* :push-start-button)

  ;; wait for heating
  (unix::sleep (* 2 60))

  ;; take object from microwave
  (send *pmi* :open-microwave)
  (send *pmi* :go-forward)
  (send *pmi* :take-object)
  (send *pmi* :go-backward)
  (send *pmi* :close-microwave)
  )
