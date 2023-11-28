#!/usr/bin/env roseus
(load "package://spoteus/spot-interface.l")
(load "human_recognize.l")
(spot-init nil)

;;do something in lawson
(send *ri* :speak-jp "ローソンに着きました")

(unix::sleep 3)

(send *ri* :speak-jp "お茶をください")

(unix::sleep 3)

(check-human)

(send *ri* :speak-jp "エレベータ前に向かいます")
