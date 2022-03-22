(require :unittest "lib/llib/unittest.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/demo/soup-from-boil.l")

(init-unit-test)

(deftest test-soup-from-boil
  (setup)
  (main)
)

(run-all-tests)
(exit)
