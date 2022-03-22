(require :unittest "lib/llib/unittest.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/demo/take-tools-off-shelf.l")
;; If you want to assert error when IK fails, load this file before (pr2) is called.
;; (load "package://jsk_2021_10_soup_from_boil/test/pr2-test-robot.l")

(init-unit-test)

(deftest test-take-tools-off-shelf
  (setup)
  (main)
)

(run-all-tests)
(exit)
