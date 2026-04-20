;; /hit_volume_threshold: 0.09

;; detect plastic bag
(send *ri* :angle-vector (send *fetch* :angle-vector #f(48.096 -2.47666 43.6074 -184.056 117.315 -180.372 73.6884 183.023 0.305897 15.7652)) 3000)
(send *ri* :wait-interpolation)

(send *ri* :angle-vector (send *fetch* :angle-vector #f(48.096 -2.53016 52.2699 -184.187 118.467 -180.893 66.1954 182.904 0.305897 15.7652)) 3000)
(send *ri* :wait-interpolation)
