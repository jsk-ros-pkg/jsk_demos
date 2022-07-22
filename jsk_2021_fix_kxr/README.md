# jsk_2021_fix_kxr
Fix kxr whose connector was removal by dual_panda2.

## Installation
### Launching dual_panda2
In leus@dual_panda2
'''bash
roslaunch jsk_panda_startup dual_panda2.launch
'''
In your pc, launch detection nodes to recognize apriltag and terminal of connector 
'''bash
rossetdualpanda2
source ~/(ws)/devel/setup.bash
roslaunch jsk_2021_fix_kxr fix_kxr_detections.launch
'''
### Connecting of kxr
According to [here](), connect kxr to 133.x network through wifi. 

In the other turminal, kxr
'''bash

'''

## Execution 
'''bash
rossetdualpanda2
source ~/(ws)/devel/setup.bash
roseus jsk_2021_fix_kxr/euslisp/main/pand_main.l
'''

## Video
See [here](https://drive.google.com/file/d/14vz8pw_OfTtmtCxj7IQx12pf1VkxYJls/view?usp=sharing)

##