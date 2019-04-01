# PR2, stabling Diablo horizontal with NeuralNetwork Controller
![video](https://github.com/takayuki5168/diabolo_pr2_201806/blob/master/gif/pr2-diabolo.gif)

### demo
```sh
$ roslaunch demo_idle_diabolo.launch
```

## idle with MPC
```sh
$ roslaunch idle_diabolo.launch
```
```sh
$ python diabolo_system.py -m ../log/diabolo_system/goodmodel_612_0.h5 -a 1 ;; simulate(calc and publish next state) and optimize_input(calc and publish next input) 
```
