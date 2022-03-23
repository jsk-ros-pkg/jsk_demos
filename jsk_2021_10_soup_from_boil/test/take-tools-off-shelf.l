(require :unittest "lib/llib/unittest.l")

(ros::roseus "take_tools_off_shelf")
(setq test-ik (ros::get-param "~test_ik" t))

(load "package://jsk_2021_10_soup_from_boil/euslisp/demo/take-tools-off-shelf.l")
;; If you want to assert error when IK fails, load this file and create *pr2* of pr2-test-robot
(when test-ik
  (load "package://jsk_2021_10_soup_from_boil/test/pr2-test-robot.l"))
;; If you want to speed up *ri* methods, load this file before and create *ri* of pr2-test-interface
(load "package://jsk_2021_10_soup_from_boil/test/pr2-test-interface.l")


(init-unit-test)

(deftest test-take-tools-off-shelf
  (setup)
  (main)
)

(run-all-tests)
(exit)
