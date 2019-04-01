# PR2, stabling Diablo horizontal with NeuralNetwork Controller
![video](https://github.com/takayuki5168/diabolo_pr2_201806/blob/master/gif/pr2-diabolo.gif)

### idle
```sh
$ roslaunch demo_idle_diabolo.launch
```

## idle with a NN controller
```sh
$ roslaunch idle_diabolo.launch
```
```sh
$ python diabolo_system.py -m ../log/diabolo_system/goodmodel_612_0.h5 -a 1
```
```sh
$ roseus juggle.l
    idle t t t :diabolo-system t
```