#!/usr/bin/env roseus

(defvar *pddl-debug* nil)

(defun move-to (&rest args)
  (cond
   (*pddl-debug*
    (warning-message 1 "move-to ~A" args)
    (read-char)
    :success)
   (t
    (let ((pl (car args)))
      (funcall (read-from-string (format nil "move-to-~A" (symbol-string pl))))
      :success))
   ))

(defun pick (&rest args)
  (cond
   (*pddl-debug*
    (warning-message 1 "pick ~A" args)
    (read-char)
    :success)
   (t
    (let ((obj (car args))
          (pl (cadr args)))
      (funcall (read-from-string (format nil "pick-~A-~A" (symbol-string obj) (symbol-string pl))))
      :success))
   ))

(defun place (&rest args)
  (cond
   (*pddl-debug*
    (warning-message 1 "place ~A" args)
    (read-char)
    :success)
   (t
    (let ((obj (car args))
          (pl (cadr args)))
      (funcall (read-from-string (format nil "place-~A-~A" (symbol-string obj) (symbol-string pl))))
      :success))
   ))

(defun put (&rest args)
  (cond
   (*pddl-debug*
    (warning-message 1 "put ~A" args)
    (read-char)
    :success)
   (t
    (let ((obj (car args))
          (pl (cadr  args)))
      (funcall (read-from-string (format nil "put-~A-~A" (symbol-string obj) (symbol-string pl))))
      :success))
   ))

(defun open-door (&rest args)
  (cond
   (*pddl-debug*
    (warning-message 1 "open-door ~A" args)
    (read-char)
    :success)
   (t
    (let ((obj (car args)))
      (funcall (read-from-string (format nil "open-~A" (symbol-string obj))))
      :success))
   ))

(defun close-door (&rest args)
  (cond
   (*pddl-debug*
    (warning-message 1 "close-door ~A" args)
    (read-char)
    :success)
   (t
    (let ((obj (car args)))
      (funcall (read-from-string (format nil "close-~A" (symbol-string obj))))
      :success))
   ))

(defun push-button (&rest args)
  (cond
   (*pddl-debug*
    (warning-message 1 "push-button ~A" args)
    (read-char)
    :success)
   (t
    (let ((obj (car args)))
      (funcall (read-from-string (format nil "push-button-~A" (symbol-string obj))))
      :success))
   ))

(defun sweep (&rest args)
  (cond
   (*pddl-debug*
    (warning-message 1 "move-to ~A" args)
    (read-char)
    :success)
   (t
    (let ((obj (car args)))
      (funcall (read-from-string (format nil "sweep-~A" (symbol-string obj))))
      :success))
   ))

(defun sweep-under (&rest args)
  (cond
   (*pddl-debug*
    (warning-message 1 "sweep-under ~A" args)
    (read-char)
    :success)
   (t
    (let ((obj (car args)))
      (funcall (read-from-string (format nil "sweep-under-~A" (symbol-string obj))))
      :success))
   ))

(defun pull-chair (&rest args)
  (cond
   (*pddl-debug*
    (warning-message 1 "pull-chair ~A" args)
    (read-char)
    :success)
   (t (pull-chair-table)
      :success)
   ))

(defun push-chair (&rest args)
  (cond
   (*pddl-debug*
    (warning-message 1 "push-chair ~A" args)
    (read-char)
    :success)
   (t (move-back-chair)
      :success)
   ))
