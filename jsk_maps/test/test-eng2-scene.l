#!/usr/bin/env roseus
;; test-eng2-scene.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")

(init-unit-test)

(deftest eng2-scene-funcs ()
  (setq *scene* (make-eng2-scene))

  ;; spots
  (assert (> (length (send *scene* :spots)) 0))
  (assert (send *scene* :spot "/eng2/7f/elevator-outside"))

  ;; floors
  (assert (eq (length (send *scene* :floors)) 5))
  (assert (send *scene* :floor< "/eng2/7f" "/eng2/8f"))
  (assert (string= "/eng2/7f"
                   (send *scene* :current-floor (make-coords :pos #f(0 0 30000)))))

  ;; rooms
  (assert (every #'(lambda (r)
                     (substringp "/eng2/7f" (send r :name)))
                 (send *scene* :rooms "/eng2/7f")))
)

(deftest eng2-scene-db-enabled ()
  (setq *scene* (make-eng2-scene))
  (when (cdr (assoc 'db-enabled-p (send *scene* :slots)))
    (assert (null (send *scene* :spot "/eng2/7f/hogehoge-spot")))
    (setq *spot-length* (length (send *scene* :spots)))
    (setq *spot* (make-cascoords :pos #f(0 0 30000) :name "/eng2/7f/hogehoge-spot"))
    (assert (send *scene* :add-spot *spot*))
    (assert (send *scene* :spot "/eng2/7f/hogehoge-spot"))
    (assert (eq 1 (- (length (send *scene* :spots)) *spot-length*)))

    (setq *scene2* (make-eng2-scene))
    (assert (send *scene2* :spot "/eng2/7f/hogehoge-spot"))
    (assert (send *scene2* :remove-spot *spot*))
    (assert (null (send *scene2* :spot "/eng2/7f/hogehoge-spot")))
    (assert (null (send *scene* :spot "/eng2/7f/hogehoge-spot")))
    (assert (eq *spot-length* (length (send *scene* :spots))))))

(deftest eng2-scene-db-disabled ()
  (setq *scene* (make-eng2-scene))
  (unless (cdr (assoc 'db-enabled-p (send *scene* :slots)))
    (assert (null (send *scene* :spot "/eng2/7f/hogehoge-spot")))
    (setq *spot-length* (length (send *scene* :spots)))
    (setq *spot* (make-cascoords :pos #f(0 0 30000) :name "/eng2/7f/hogehoge-spot"))
    (assert (send *scene* :add-spot *spot*))
    (assert (send *scene* :spot "/eng2/7f/hogehoge-spot"))
    (assert (eq 1 (- (length (send *scene* :spots)) *spot-length*)))

    (setq *scene2* (make-eng2-scene))
    (assert (null (send *scene2* :spot "/eng2/7f/hogehoge-spot")))
    (assert (send *scene* :spot "/eng2/7f/hogehoge-spot"))
    ;; (assert (null (send *scene2* :remove-spot *spot*))) ;; this actually causes error
    (assert (null (send *scene2* :spot "/eng2/7f/hogehoge-spot")))
    (assert (send *scene* :spot "/eng2/7f/hogehoge-spot"))
    (assert (not (eq *spot-length* (length (send *scene* :spots)))))))

(run-all-tests)
(exit)
