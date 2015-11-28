#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)
(load "package://jsk_2013_04_pr2_610/euslisp/demo.l")

(defmethod pr2-interface
  (:find-object (nam) (find nam (send self :objects) :key #'(lambda (x) (send x :name)) :test #'equal)))

(deftest test-demo
  (demo)
  (format *error-output* "demo done~%")
  (format *error-output* "robot positoin ~A~%" (send *ri* :worldcoords))
  (assert (eps-v= (send (send *ri* :worldcoords) :worldpos) #f(6600 1000 0)) "robot position")
#|
  (format *error-output* "tray positoin ~A~%" (send *ri* :find-object "room610-tray"))
  (assert (eps-v= (send (send *ri* :find-object "room610-tray") :worldpos) #f(8090 350 850) 10) "tray position")

  (format *error-output* "sweeper positoin ~A~%" (send *ri* :find-object "room610-sweeper"))
  (assert (eps-v= (send (send *ri* :find-object "room610-sweeper") :worldpos)
                  (send (send *ri* :worldcoords) :worldpos) 1000) "broom position")
|#
  )

(run-all-tests)
(exit)
