(defmacro move-to (place)
  `(funcall (read-from-string (concatenate string "move-to-" (symbol-name (quote ,place))))))

(defmacro pick (obj place)
  `(funcall (read-from-string (concatenate string "pick-" (symbol-name (quote ,obj)) "-" (symbol-name (quote ,place))))))

(defmacro put (obj place)
  `(funcall (read-from-string (concatenate string "put-" ,obj "-" ,place))))

(defmacro open (obj)
  `(funcall (read-from-string (concatenate string "open-" ,obj))))

(defmacro close (obj)
  `(funcall (read-from-string (concatenate string "close-" ,obj))))

(defmacro push-button (obj)
  `(funcall (read-from-string (concatenate string "push-button-" ,obj))))

(defmacro pull-chair (place)
  `(move-chair)) ;; TODO いろいろなものに対応する

(defmacro push-chair (place)
  `(move-chair-back))
