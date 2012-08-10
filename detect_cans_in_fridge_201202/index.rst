detect_cans_in_fridge_201202 ROS Launch Files
=============================================

**Description:** detect_cans_in_fridge_201202

  
  
       detect_cans_in_fridge_201202
  
    

**License:** BSD

detect_cans.launch
------------------

.. code-block:: bash

  roslaunch detect_cans_in_fridge_201202 detect_cans.launch

grasp_object_extract.launch
---------------------------

.. code-block:: bash

  roslaunch detect_cans_in_fridge_201202 grasp_object_extract.launch

rviz.launch
-----------

.. code-block:: bash

  roslaunch detect_cans_in_fridge_201202 rviz.launch

startup.launch
--------------

.. code-block:: bash

  roslaunch detect_cans_in_fridge_201202 startup.launch



This package containts detect and fetch can in the fridge demo program


.. figure:: http://r7videos-thumbnail.s3.amazonaws.com/ER7_RE_JR_DOMESTICAS_452kbps_2012-02-23_b9dace72-5e73-11e1-b9a6-4ba54d97a5f8.jpg
   :width 400

   http://noticias.r7.com/videos/japoneses-utilizam-robo-para-as-funcoes-de-empregada-domestica/idmedia/4f46c7a2fc9b864945d600a5.html

.. code-block:: bash

  @c1; roslaunch jsk_pr2_startup pr2.launch
  @c1; roslaunch detect_cans_in_fridge_201202 startup.launch
  @local; roslaunch  detect_cans_in_fridge_201202 rviz.launch

set current pr2 position using "2D Pose Estimate" button on rviz

make sure that Tool Properties -> Interact -> 2D Nav Goal -> Topic is move_bas_simple_goal and  
2DPoseEstimate is initialpose.

.. code-block:: bash

  @local; rosrun roseus roseus `rospack find detect_cans_in_fridge_201202`/euslisp/main2.l

type (init)(demo) to start demo



Contents
########

.. code-block:: xml

  <launch>
    
    <anode args="-l -r 0.1 /home/leus/work/rits/refrig-kinect-c.bag" name="play" pkg="rosbag" type="play" />
  
    
    <anode name="select_ref_white" pkg="image_view2" type="image_view2">
      <remap from="image" to="/camera/rgb/image_rect_color" />
      <remap from="camera_info" to="/camera/rgb/camera_info" />
      <remap from="/camera/rgb/image_rect_color/screenpoint" to="/camera/rgb/screenpoint" />
    </anode>
  
    
    <anode args="-d $(find detect_cans_in_fridge_201202)/detect_cans.vcg" name="detect_cans_rviz" pkg="rviz" type="rviz" />
  
    
    <include file="$(find jsk_2011_07_pr2_semantic)/launch/start_perception.launch" />
  
    
    <include file="$(find detect_cans_in_fridge_201202)/launch/white_balance.launch" />
  
    
    <include file="$(find detect_cans_in_fridge_201202)/launch/detect_cans.launch" />
  
    
    <include file="$(find pr2_machine)/$(env ROBOT).machine" />
    <param name="SnapMapICP/age_threshold" value="2.0" />
    <node machine="c2" name="tum_SnapMapICP" output="screen" pkg="SnapMapICP" type="SnapMapICP" />
  
    </launch>

test_perception.launch
----------------------

.. code-block:: bash

  roslaunch detect_cans_in_fridge_201202 test_perception.launch

white_balance.launch
--------------------

.. code-block:: bash

  roslaunch detect_cans_in_fridge_201202 white_balance.launch

