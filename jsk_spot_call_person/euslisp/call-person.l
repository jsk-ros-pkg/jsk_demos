#!/usr/bin/env roseus

(defun set-name ()
  (ros::ros-info "talk to Spot")
  (let ((message
         (elt (send (one-shot-subscribe
                     "/speech_to_text_jp"
                     speech_recognition_msgs::SpeechRecognitionCandidates) :transcript) 0)))
    (ros::ros-info "~A" message)
    (cond
      ((substringp "稲葉" message) "inaba")
      ((substringp "岡田" message) "okada")
      ((substringp "北川" message) "kitagawa")
      ((substringp "長谷川" message) "hasegawa")
      ((substringp "山口" message) "yamaguchi") 
      ((substringp "新城" message) "shinjo")
      ((substringp "金沢" message) "kanazawa")
      ((substringp "若林" message) "wakabayashi")
      ((substringp "種本" message) "tanemoto")
      ((substringp "ギガ" message) "guiga")
      ((substringp "石田" message) "ishida")
      ((substringp "平岡" message) "hiraoka")
      ((substringp "おびなた" message) "obinata")
      ((substringp "市倉" message) "ichikura")
      ((substringp "塚本" message) "tsukamoto")
      ((substringp "PR" message) "pr2")
      ((substringp "hr" message) "hrp2")
      ((substringp "3Dプリンター" message) "3dprinter")
      (t nil))))

;; set desk position to hash table
(defvar *desk-position* (make-hash-table :test #'equal))
(defun add-entry-desk-position (key value)
  (setf (gethash (intern key) *desk-position*) value))
(defun look-up-desk-position (key)
  (gethash (intern key) *desk-position*))

(defun make-hash-desk-position ()
  (add-entry-desk-position "yamaguchi" (list 0 0 0 0 73a1 "稲葉"))
  (add-entry-desk-position "yamaguchi" (list 0 0 0 0 73a2 "岡田"))
  (add-entry-desk-position "hrp2" (list 3 0 0 0 73b2"hrpつー"))
  (add-entry-desk-position "hiraoka" (list 4 0 -0.2 0 73b2 "平岡"))
  (add-entry-desk-position "obinata" (list 4 0 0 0 73b2 "おびなた"))
  (add-entry-desk-position "ishida" (list 5 0 0 90 73b2 "石田"))
  (add-entry-desk-position "tsukamoto" (list 6 0 0 0 73b2 "塚本"))
  (add-entry-desk-position "PR2" (list 10 0 0 0 73b2 "prつー"))
  (add-entry-desk-position "ichikura" (list 11 0 0 0 73b2 "いちくら"))
  (add-entry-desk-position "kitagawa" (list 13 0 0 0 73b2 "北川"))
  (add-entry-desk-position "3dprinter" (list 12 0 0.2 0 73b2 "すりーdぷりんたー"))
  (add-entry-desk-position "shinjo" (list 16 0 0 0 73b2 "新城"))
  (add-entry-desk-position "kanazawa" (list 16 0.2 0 0 73b2 "金沢"))
  (add-entry-desk-position "wakabayashi" (list 17 0 -0.2 0 73b2 "若林"))
  (add-entry-desk-position "tanemoto" (list 18 0 0 0 73b2 "種本"))
  (add-entry-desk-position "yamaguchi" (list 0 0 0 0 73a3 "山口"))
  (add-entry-desk-position "guiga" (list 18 0.3 0.2 30 73b2 "ギガ"))
  (add-entry-desk-position "yamaguchi" (list 0 0 0 0 73a3 "長谷川")))

(defun call-person ()
  (let (calling-name caller-name room)
    (send *ri* :speak-jp "あなたの名前を教えてください" :wait t)
    (while (not (setq caller-name (set-name)))
      (send *ri* :speak-jp "聞き取れませんでした" :wait t))
    (send *ri* :speak-jp (format nil "~Aさんですね" caller-name))
    (send *ri* :speak-jp "呼びたい人を教えてください" :wait t)
    (while (not (setq calling-name (set-name)))
      (send *ri* :speak-jp "聞き取れませんでした" :wait t))
    (send *ri* :speak-jp (format nil "~Aさんを呼びます" string name))
    (send *ri* :speak-jp "どこに呼びたいか教えてください" :wait t)
    (while (not (setq room (set-room)))
      (send *ri* :speak-jp "聞き取れませんでした" :wait t))
    (send *ri* :speak-jp
          (format nil "~Aさんを~Aに呼びに行きます" calling-name room) :wait t)

    ;; TODO navigate
    (setq msg (instance spot_behavior_manager_msgs::LeadPersonActionGoal :init))
    (send (send msg :goal) :target_node_id "eng2_73A4")
    (ros::publish "/spot_behavior_manager_server/execute_behaviors/goal" msg)

    (send *ri* :speak-jp
          (concatenate string caller-name "さん" caller-name "さんが呼んでいます") :wait t)
    (send *ri* :speak-jp (concatenate string room "に来てください") :wait t)
    ))

(defun main ()
  (ros::advertise "/spot_behavior_manager_server/execute_behaviors/goal"
                  spot_behavior_manager_msgs::LeadPersonActionGoal 1)
  ;; (autowalk-data-upload)
  (make-hash-desk-position)
  ;;(send *ri* :go-pos 1.0 1.2 0)

  (call-person)
  (while (not
          (string=
           "受け取りました"
           (elt (send (one-shot-subscribe
                       "/speech_to_text_jp"
                       speech_recognition_msgs::SpeechRecognitionCandidates) :transcript) 0))))
  (send *ri* :speak-jp "戻ります")
  (send *ri* :navigate-to 0))
