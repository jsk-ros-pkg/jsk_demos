(defun check-stop-words (text)
  (let ((stop-words (list
                     "終了"
                     "終わり"
                     "おわり"
                     "おつかれ"
                     "お疲れ"
                     "もういいよ"
                     "止めて"
                     "とめて"
                     "ストップ"
                     )))
          (dolist (s stop-words)
            (when (string= text s)
              (return-from check-stop-words t)))))
