#!/usr/bin/env roseus

(ros::roseus "jsk_fetch_plug_power_spot_demo")

(load "package://fetcheus/fetch-interface.l")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(load "package://jsk_fetch_plug_power_spot/euslisp/parameters.l")
(load "package://jsk_fetch_plug_power_spot/euslisp/utils.l")

(fetch-init)

(setq ret (unplug-power-connector))

(if ret
  (exit 0)
  (exit -1))
