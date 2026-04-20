decopin_hand
============

# Usage

1. Find dynamixels from USB port.
```
rosrun decopin_hand find_dynamixel /dev/ttyUSB0
```

2. Start dynamixel controllers. You can change config of dynamixels at `config/yamaguchi_dynamixel.yaml`
```
roslaunch decopin_hand dynamixel_workbench_controllers.launch
```

3. Move dynamixels via roseus
```
roseus euslisp/decopin-interface.l
(decopin-init)
(send *ri* :angle-vector (send *robot* :reset-pose))
```
