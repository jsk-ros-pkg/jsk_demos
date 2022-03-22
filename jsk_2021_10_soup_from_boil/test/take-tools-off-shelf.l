(require :unittest "lib/llib/unittest.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/demo/take-tools-off-shelf.l")

(init-unit-test)

(deftest test-take-tools-off-shelf
  (setup)
  (main)
)

(run-all-tests)
(exit)
