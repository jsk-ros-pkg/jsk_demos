#!/usr/bin/env roseus

;; 1. フラグ用の変数を初期化
(setq *stop-requested* nil)

;; 2. Roseusの仕様（4引数）に合わせたハンドラ定義
(defun ctrl-c-handler (code msg &optional line obj)
  ;; 念のためSIGINT（Ctrl+C）であることを確認
  (if (substringp "SIGINT" msg)
      (progn
        (format t "~%[Signal] Ctrl+C received. Setting stop flag...~%")
        (setq *stop-requested* t))
    ;; SIGINT以外のエラーはそのままシステムのエラー処理に任せるか、exitする
    (progn
      (format t "~%[Error] Unexpected error: ~A~%" msg)
      (exit 1))))

;; 3. ハンドラを登録
(install-error-handler 'ctrl-c-handler)

;; 4. メインループ
(format t "Processing... Press Ctrl+C to stop safely.~%")
(while (not *stop-requested*)
  ;; ここにファイル書き込みやロボットへの指令などのメイン処理を書く
  (format t "Working...~%")
  (unix:usleep 500000) ;; 0.5秒待機
  )

;; 5. ループを抜けた後のクリーンアップ処理
(format t "Finishing file and cleaning up...~%")
;; ここにファイルの一括保存やクローズ処理を書く

(format t "Done!~%")
(exit 0)