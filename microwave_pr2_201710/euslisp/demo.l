(require :pr2-microwave-interface "package://microwave_pr2_201710/euslisp/lib/pr2-microwave-interface.l")

(defun demo
    ()
  ;; init
  (setq *pmi* (instance pr2-microwave-interface :init))

  ;; go to microwave
  (send *pmi* :update-microwave-pose)

  ;; put object in microwave
  (send *pmi* :open-microwave)
  (send *pmi* :put-object)
  (send *pmi* :close-microwave)

  ;; push button
  (send *pmi* :push-500w-button)
  (send *pmi* :push-1min-button)
  (send *pmi* :push-start-button)

  #|
  ;; wait for heating
  (unix::sleep (* 2 60))

  ;; take object from microwave
  (send *pmi* :open-microwave)
  (send *pmi* :take-object)
  (send *pmi* :close-microwave)
  |#
  )
