どの変数がどこの関節角を表しているかを知りたいとき

```lisp
(send *pr2* :joint-list)
```

l_shoulder_pan_jointを単独で動かしたいとき
```lisp
(send *pr2* :l_shoulder_pan_joint :joint-angle 80)
```

https://www.clearpathrobotics.com/assets/downloads/pr2/pr2_manual_r321.pdf のP16に関節名が書いてある