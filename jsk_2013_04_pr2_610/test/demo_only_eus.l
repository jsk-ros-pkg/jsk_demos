(require :unittest "lib/llib/unittest.l")

(load "package://jsk_2013_04_pr2_610/euslisp/demo.l")

(init-unit-test)

(deftest dmeo ()
  (demo)
  t)

(run-all-tests)

(exit)
