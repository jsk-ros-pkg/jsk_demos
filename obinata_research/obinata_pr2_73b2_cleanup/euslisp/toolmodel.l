;; bottle
(setq bottle-2l (make-solid-of-revolution '(#f(0 0 300) #f(14 0 300) #f(14 0 280) #f(44 0 230) #f(44 0 0) #f(0 0 0)))) 
(send bottle-2l :set-color :white)

;; seasonings
(setq ketchup (make-solid-of-revolution '(#f(0 0 190) #f(15 0 190) #f(15 0 160) #f(35 0 90) #f(35 0 30) #f(30 0 0) #f(0 0 0))))
(send ketchup :set-color :red)
(setq sauce (make-solid-of-revolution '(#f(0 0 190) #f(15 0 190) #f(15 0 160) #f(35 0 90) #f(35 0 30) #f(30 0 0) #f(0 0 0))))
(send sauce :set-color :brown)

;; obinata tools
(setq eyeglasses-case (make-cube 170 60 40))
(send eyeglasses-case :set-color :black)
(setq obinata-wallet (make-cube 90 110 20))
(send obinata-wallet :set-color :gray)
(setq obinata-purse (make-cube 75 95 20))
(send obinata-purse :set-color :gray) 