elevator_move_base_pr2 ROS Launch Files
=======================================

**Description:** elevator_move_base_pr2

  
  
       elevator_move_base_pr2
  
    

**License:** BSD

elevator_move_base_eng2.launch
------------------------------

.. code-block:: bash

  roslaunch elevator_move_base_pr2 elevator_move_base_eng2.launch

elevator_move_base_eng8.launch
------------------------------

.. code-block:: bash

  roslaunch elevator_move_base_pr2 elevator_move_base_eng8.launch

test-button-light.launch
------------------------

.. code-block:: bash

  roslaunch elevator_move_base_pr2 test-button-light.launch


This scripts is test for elevator call panel light state.

The bagfile contains these topics.
narrow_stereo/left/{camera_info,image_raw}, joint_states, tf, view_point

Then check the button color to detect the button was pushed.
The button area in the camera image is calcurated from Euslisp camera model and panel object model.


.. image:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-electric/lastSuccessfulBuild/artifact/doc/jsk-ros-pkg-electric/html/_images/test-button-light.mp4
  :width: 600

.. video:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-electric/lastSuccessfulBuild/artifact/doc/jsk-ros-pkg-electric/html/_images/call-panel-lighting-not
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
  
    <param name="use_sim_time" value="true" />
    <include file="$(find pr2_machine)/sim.machine" />
    <include file="$(find elevator_move_base_pr2)/launch/elevator_move_base_modules.xml" />
  
    
    <node args="$(find elevator_move_base_pr2)/test/test-button-light.bag -l --clock" name="rosbag_play" pkg="rosbag" type="play" />
  
    
    
    <group ns="/wide_stereo/left">
      <node name="image_proc" pkg="image_proc" type="image_proc" />
    </group>
  
    
    <test args="$(find elevator_move_base_pr2)/test/test-button-light.l" pkg="roseus" test-name="button_light" time-limit="300" type="roseus" />
  
    <node name="mjpeg_server" output="screen" pkg="mjpeg_server" respawn="true" type="mjpeg_server">
      <param name="port" type="int" value="8910" />
    </node>
    <node args="http://localhost:8910/stream?topic=/light_detector/debug_image $(find elevator_move_base_pr2)/build/test-button-light.mp4 -t 20 -p 8910" name="capture_result" pkg="jsk_tools" type="mjpeg_capture.sh" />
  
  
    <anode args="-d $(find elevator_move_base_pr2)/test/test-button-light.vcg" launch-prefix="glc-capture --start --out=$(find elevator_move_base_pr2)/build/test-button-light.glc" name="rviz" pkg="rviz" respawn="true" type="rviz" />
    
    <atest args="$(find elevator_move_base_pr2)/build/test-button-light.glc --ctx 1 -o $(find elevator_move_base_pr2)/build/test-button-light" pkg="jsk_tools" test-name="z_encode_test1" time-limit="300" type="glc_encode.sh" />
  
  </launch>

test-modules-callpanel.launch
-----------------------------

.. code-block:: bash

  roslaunch elevator_move_base_pr2 test-modules-callpanel.launch


This scripts is test for elevator call panel.

.. video:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-electric/lastSuccessfulBuild/artifact/doc/jsk-ros-pkg-electric/html/_images/test-modules-callpanel-1
  :width: 600

.. video:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-electric/lastSuccessfulBuild/artifact/doc/jsk-ros-pkg-electric/html/_images/test-modules-callpanel-2
  :width: 600

Then check the button color to detect the button was pushed.
The button area in the camera image is calcurated from Euslisp camera model and panel object model.

  

Contents
########

.. code-block:: xml

  <launch>
  
    <param name="use_sim_time" value="true" />
    <include file="$(find pr2_machine)/sim.machine" />
    <include file="$(find pr2_description)/robots/upload_pr2.launch" />
    <include file="$(find jsk_maps)/launch/start_map_eng2.launch" />
    <include file="$(find elevator_move_base_pr2)/launch/elevator_move_base_modules.xml" />
  
    
    <node args="$(find elevator_move_base_pr2)/test/test-modules-callpanel.bag -l -r 0.5 --clock" name="rosbag_play" pkg="rosbag" type="play" />
  
    
    <group ns="/narrow_stereo/left">
      <node name="image_proc" pkg="image_proc" type="image_proc">
        <param name="queue_size" value="100" /> 
      </node>
      <node name="sift" pkg="imagesift" type="imagesift">
        <remap from="image" to="image_rect" />
      </node>
    </group>
  
    <group ns="/wide_stereo/left">
      <node name="image_proc" pkg="image_proc" type="image_proc" />
    </group>
  
    
    <test args="$(find elevator_move_base_pr2)/test/test-modules-callpanel.l" pkg="roseus" test-name="modules" time-limit="300" type="roseus" />
  
    <node args="-d $(find elevator_move_base_pr2)/test/test-modules-callpanel.vcg" launch-prefix="glc-capture --start --out=$(find elevator_move_base_pr2)/build/test-modules-callpanel.glc" name="rviz" pkg="rviz" respawn="true" type="rviz" />
  
    
    <test args="$(find elevator_move_base_pr2)/build/test-modules-callpanel.glc" pkg="jsk_tools" test-name="z_encode_test1" time-limit="1000" type="glc_encode.sh" />
  
  </launch>

test-modules-insidepanel.launch
-------------------------------

.. code-block:: bash

  roslaunch elevator_move_base_pr2 test-modules-insidepanel.launch


This scripts is test for elevator inside panel.

.. video:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-electric/lastSuccessfulBuild/artifact/doc/jsk-ros-pkg-electric/html/_images/images/call-panel-pose
  :width: 600

Then apply affine transform to camera image for template match.
Template is number region of the panel.

.. video:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-electric/lastSuccessfulBuild/artifact/doc/jsk-ros-pkg-electric/html/_images/images/inside-panel-number
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
  
    <param name="use_sim_time" value="true" />
    <include file="$(find pr2_machine)/sim.machine" />
    <include file="$(find elevator_move_base_pr2)/launch/elevator_move_base_modules.xml" />
  
    
    <node args="$(find elevator_move_base_pr2)/test/test-eng2-inside-panel.bag -l -r 0.2 --clock" name="rosbag_play" pkg="rosbag" type="play" />
  
    
    
    <group ns="/narrow_stereo/left">
      <node name="image_proc" pkg="image_proc" type="image_proc">
      <param name="queue_size" value="100" /> 
      </node>
      <node name="sift" pkg="imagesift" type="imagesift">
        <remap from="image" to="image_rect" />
      </node>
    </group>
  
    
    <atest args="$(find elevator_move_base_pr2)/test/test-panel-pose-detection.l" pkg="roseus" test-name="panel_pose_detection" type="roseus" />
  
    
    <atest args="$(find elevator_move_base_pr2)/test/test-number-recognition.l" pkg="roseus" test-name="number_recognition" type="roseus" />
  
  
    <node args="-d $(find elevator_move_base_pr2)/test/test-modules-insidepanel.vcg" launch-prefix="glc-capture --start --out=$(find elevator_move_base_pr2)/build/test-modules-insidepanel.glc" name="rviz" pkg="rviz" respawn="true" type="rviz" />
  
    
    <atest args="$(find elevator_move_base_pr2)/build/test-modules-insidepanel.glc --ctx 1 -o $(find elevator_move_base_pr2)/build/call-panel-pose.mp4" pkg="jsk_tools" test-name="z_encode_test1" time-limit="300" type="glc_encode.sh" />
    <atest args="$(find elevator_move_base_pr2)/build/test-modules-insidepanel.glc --ctx 2 -o $(find elevator_move_base_pr2)/build/inside-panel-number.mp4" pkg="jsk_tools" test-name="z_encode_test2" time-limit="300" type="glc_encode.sh" />
  
  </launch>

test-panel-pose-detection.launch
--------------------------------

.. code-block:: bash

  roslaunch elevator_move_base_pr2 test-panel-pose-detection.launch


This scripts is test for elevator call panel pose detection.


.. video:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-electric/lastSuccessfulBuild/artifact/doc/jsk-ros-pkg-electric/html/_images/test-panel-pose-detection-1
  :width: 600

.. video:: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080//job/jsk-ros-pkg-electric/lastSuccessfulBuild/artifact/doc/jsk-ros-pkg-electric/html/_images/test-panel-pose-detection-2
  :width: 600

  

Contents
########

.. code-block:: xml

  <launch>
  
    <param name="use_sim_time" value="true" />
    <include file="$(find pr2_machine)/sim.machine" />
    <include file="$(find pr2_description)/robots/upload_pr2.launch" />
    <include file="$(find jsk_maps)/launch/start_map_eng2.launch" />
    <include file="$(find elevator_move_base_pr2)/launch/elevator_move_base_modules.xml" />
  
    
    <node args="$(find elevator_move_base_pr2)/test/test-panel-pose-detection.bag -l -r 0.2 --clock" name="rosbag_play" pkg="rosbag" type="play" />
  
    
    <group ns="/narrow_stereo/left">
      <node name="image_proc" pkg="image_proc" type="image_proc">
        <param name="queue_size" value="100" /> 
      </node>
      <node name="sift" pkg="imagesift" type="imagesift">
        <remap from="image" to="image_rect" />
      </node>
    </group>
  
    <group ns="/wide_stereo/left">
      <node name="image_proc" pkg="image_proc" type="image_proc" />
    </group>
  
    
    <test args="$(find elevator_move_base_pr2)/test/test-panel-pose-detection.l" pkg="roseus" test-name="panel_pose_detection" time-limit="300" type="roseus" />
  
    <node args="-d $(find elevator_move_base_pr2)/test/test-panel-pose-detection.vcg" launch-prefix="glc-capture --start --out=$(find elevator_move_base_pr2)/build/test-panel-pose-detection.glc" name="rviz" pkg="rviz" respawn="true" type="rviz" />
  
    
    <test args="$(find elevator_move_base_pr2)/build/test-panel-pose-detection.glc" pkg="jsk_tools" test-name="z_encode_test1" time-limit="300" type="glc_encode.sh" />
  
  </launch>

