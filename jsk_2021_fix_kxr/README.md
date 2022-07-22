# jsk_2021_fix_kxr
Fix KXR whose connector was removal by dual_panda2.

## Installation
### Connecting KXR with your PC and starting communication
According to [here](https://www.google.com/url?sa=j&url=https%3A%2F%2Fgitlab.jsk.imi.i.u-tokyo.ac.jp%2Frcb4eus%2Frcb4eus%2F-%2Fwikis%2Fhome&uct=1649641403&usg=XDviwd1cl8Mh3gkoKqJNqcVi3k8.&source=chat), set up KXR and connect KXR to 133.x network through wifi. 


After setting up KXR, connet to your pc through wifi
```
cd ~/prog/rcb4eus/Arduino/M5Stack/M5Stack_rcb4_rosserial
roslaunch rosserial_wifi.launch
```

In the other terminal, start communication with dual_panda2
```
cd ~/prog/rcb4eus
source ~/(ws)/devel/setup.bash
roseus ~/.../jsk_2021_fix_kxr/euslisp/main/kxr_main.l
```
### Launching dual_panda2
In leus@dual_panda2,
```
roslaunch jsk_panda_startup dual_panda2.launch
```

In your pc, launch detection nodes to recognize apriltag and terminal of connector 
```
rossetdualpanda2
source ~/(ws)/devel/setup.bash
roslaunch jsk_2021_fix_kxr fix_kxr_detections.launch
```

## Execution
After setting KXR on workbench as shown in the figure below、start demo

<img src="https://user-images.githubusercontent.com/48650394/180359214-30ca4f64-6571-4abb-a6cf-ea169517f0cb.png" width="550px">

```
rossetdualpanda2
source ~/(ws)/devel/setup.bash
roseus jsk_2021_fix_kxr/euslisp/main/panda_main.l
```

## Video
See [here](https://drive.google.com/file/d/14vz8pw_OfTtmtCxj7IQx12pf1VkxYJls/view?usp=sharing)

##
