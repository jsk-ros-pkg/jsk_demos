;;this is numerical library for euslisp.

;;numerical list
(defun count-down-step (start step n)
  (cond ((eq n 1) (cons start nil))
	(t
	 (cons (+ step (car (count-down-step start step (1- n)))) 
	       (count-down-step start step (1- n))))
	)
  )

(defun count-up-step (start step n)
  (reverse (count-down-step start step n))
  )

(defun count-up (start stop &key(step 1) (n))
  (setq n  (1+ (floor (abs (/ (- stop start) step)))))
  (when (> start stop) (setq step (- (abs step))))
  (count-up-step start step n)    
  )


(setq *random-seed* (integer-vector (elt (unix:localtime) 0) (elt (unix:localtime) 1)))
;;random list
;;make list of random number(>0)
;;*random-seed* is changed by random function.
(defun random-list-p (range n &key (bias 0))
  (if (< n 2)
      (list (+ (random range *random-seed*) bias))
    (if (eq n 2)
        (list (+ (random range *random-seed*) bias) (+ (random range *random-seed*) bias))
      (cons (+ (random range *random-seed*) bias) (random-list-p range (- n 1) :bias bias)))))
(defun random-list (range n &key (both-pn t) (bias 0))
"make list of random number which can be eather positive or negative"
  (cond ((eq both-pn t)
         (mapcar #'(lambda (x y) (* x y))
                 (random-list-p range n :bias bias)
                 (mapcar #'(lambda (x) (- (* x 2) 1)) (random-list-p 2 n))
                 ))
        (t (random-list-p range n :bias bias))))

(defun mapvector (f v)
"use vector like matlab. For example, mapvector abs #f(-1 -2 3) returns #f(1 2 3)"
  (let ((ret v temp))
    (setq ret (make-array (length v) :element-type :float))
    (do ((i 0 (1+ i)))
        ((>= i (length v)))
        (setq temp (funcall f (elt v i)))
        (setf (elt ret i) temp)
      )
    ret
    ))


(defun sign (x)
  (cond ((= x 0) 0)
        ((> x 0) 1)
        (t -1)))