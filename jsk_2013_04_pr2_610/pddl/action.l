#!/usr/bin/env roseus

(defvar *pddl-debug* nil)

(defun move-to (&rest args)
  (let ((pl (car args)))
    (when *pddl-debug*
      (warning-message 1 "move-to ~A" args)
      (read-char)
      (return-from move-to :success))
    (funcall (read-from-string (format nil "move-to-~A" (symbol-string pl))))
    :success))

(defun pick (&rest args)
  (let ((obj (car args))
        (pl (cadr args)))
    (when *pddl-debug*
      (warning-message 1 "pick ~A" args)
      (read-char)
      (return-from pick :success))
    (funcall (read-from-string (format nil "pick-~A-~A" (symbol-string obj) (symbol-string pl))))
    :success))

(defun place (&rest args)
  (let ((obj (car args))
        (pl (cadr args)))
    (when *pddl-debug*
      (warning-message 1 "place ~A" args)
      (read-char)
      (return-from place :success))
    (funcall (read-from-string (format nil "place-~A-~A" (symbol-string obj) (symbol-string pl))))
    :success))

(defun put (&rest args)
  (let ((obj (car args))
        (pl (cadr  args)))
    (when *pddl-debug*
      (warning-message 1 "put ~A" args)
      (read-char)
      (return-from put :success))
    (funcall (read-from-string (format nil "put-~A-~A" (symbol-string obj) (symbol-string pl))))
    :success))

(defun open-door (&rest args)
  (let ((obj (car args)))
    (when *pddl-debug*
      (warning-message 1 "open-door ~A" args)
      (read-char)
      (return-from open-door :success))
    (funcall (read-from-string (format nil "open-~A" (symbol-string obj))))
    :success))

(defun close-door (&rest args)
  (let ((obj (car args)))
    (when *pddl-debug*
      (warning-message 1 "close-door ~A" args)
      (read-char)
      (return-from close-door :success))
    (funcall (read-from-string (format nil "close-~A" (symbol-string obj))))
    :success))

(defun push-button (&rest args)
  (let ((obj (car args)))
    (when *pddl-debug*
      (warning-message 1 "push-button ~A" args)
      (read-char)
      (return-from push-button :success))
    (funcall (read-from-string (format nil "push-button-~A" (symbol-string obj))))
    :success))

(defun sweep (&rest args)
  (let ((obj (car args)))
    (when *pddl-debug*
      (warning-message 1 "sweep ~A" args)
      (read-char)
      (return-from sweep :success))
    (funcall (read-from-string (format nil "sweep-~A" (symbol-string obj))))
    :success))

(defun sweep-under (&rest args)
  (let ((obj (car args)))
    (when *pddl-debug*
      (warning-message 1 "sweep-under ~A" args)
      (read-char)
      (return-from sweep-under :success))
    (funcall (read-from-string (format nil "sweep-under-~A" (symbol-string obj))))
    :success))

(defun pull-chair (&rest args)
  (when *pddl-debug*
    (warning-message 1 "pull-chair ~A" args)
    (read-char)
    (return-from pull-chair :success))
  (pull-chair-table)
  :success)

(defun push-chair (&rest args)
  (when *pddl-debug*
    (warning-message 1 "push-chair ~A" args)
    (read-char)
    (return-from push-chair :success))
  (move-back-chair)
  :success)
