(defmacro move-to (place)
  `(funcall (read-from-string (concatenate string "move-to-" (symbol-name (quote ,place))))))

(defmacro pick (obj place)
  `(funcall (read-from-string (concatenate string "pick-" (symbol-name (quote ,obj)) "-" (symbol-name (quote ,place))))))

(defmacro place (obj place)
  `(funcall (read-from-string (concatenate string "place-" (symbol-name (quote ,obj)) "-" (symbol-name (quote ,place))))))

(defmacro put (obj place)
  `(funcall (read-from-string (concatenate string "put-" (symbol-name (quote ,obj)) "-" (symbol-name (quote ,place))))))

(defmacro open-door (obj)
  `(funcall (read-from-string (concatenate string "open-" (symbol-name (quote ,obj))))))

(defmacro close (obj)
  `(funcall (read-from-string (concatenate string "close-" (symbol-name (quote ,obj))))))

(defmacro sweep (obj)
  `(funcall (read-from-string (concatenate string "sweep-" (symbol-name (quote ,obj))))))

(defmacro sweep-under (obj)
  `(funcall (read-from-string (concatenate string "sweep-under-" (symbol-name (quote ,obj))))))


(defmacro push-button (obj)
  `(funcall (read-from-string (concatenate string "push-button-" (symbol-name (quote ,obj))))))

(defmacro pull-chair (place)
  `(pull-chair-table)) ;; TODO いろいろなものに対応する

(defmacro push-chair (place)
  `(move-back-chair))
