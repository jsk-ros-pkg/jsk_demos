#!/usr/bin/env roseus

(load "packge://spotkinovaeus/spotkinova-interface.l")
(spotkinova-init)

(send *spotkinova* :angle-vector #f(28.3015 81.8334 -160.021 -27.723 82.1618 -160.021 32.7776 80.807 -160.021 -32.405 80.9873 -160.021 5.44795 -60.5937 126.823 -85.8483 98.7568 -87.5314))

;;(send *spotkinova* :angle-vector #f(6.98482 45.9484 -85.6121 -7.04426 46.2331 -85.471 7.87209 45.291 -85.8227 -8.16397 45.4545 -85.7934 -0.016876 -148.045 -147.935 -89.7255 -88.7563 -89.7032))

(send *ri* :angle-vector (send *spotkinova* :angle-vector) :fast)
(send *ri* :speak-en "I'm waiting for task" :wait t)
