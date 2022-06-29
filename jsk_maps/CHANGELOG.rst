^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_maps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.5 (2021-07-17)
------------------
* add fetch dock2 (`#1329 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1329>`_)
* [jsk_maps] update change-floor.l to set *tfl* when it is not set (`#1301 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1301>`_)
* fix slant and fetch place (`#1300 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1300>`_)
  * update 73b2 kitchen map
  * change 3f-map origin
  * update eng2-3f-0.05_keepout.xcf
  * fix the elevator in eng2 3th floor
  * remove fetch dock in 73b2
  * update 73b2 map
  * update eng2 7f maps to add vending machine
  * make *tfl* when there is no *tfl*
  * remove unused mux key
  * remap static_map to static_map_keepout
  * publish map_keepout topic
  * resize and rotate eng2 8f map
  * set keepout arg default false
  * add eng6 and eng8 keepout arg
  * add eng8 keepout maps
  * add eng6 keepout maps
  * update eng2 1f and 2f keepout
  * remove unnecessary return
  * use group tag
  * add keepout arg in launch/start_map_eng2.launch
  * add eng2 8f keepout
  * add eng2/8f keepout map
  * add keepout arg for eng2 building
  * generate keepout yaml
  * remove eng2-7f-0.05_keepout.yaml
  * update eng2 1f map
  * Merge branch 'master' into update-maps
  * update eng2 3f map
  * update eng2 7f map
  * update eng2 2f maps
  * remove unnecessary space in jsk_maps/CMakeLists.txt
  * Update eng2/7f map
  * Update eng2/2f map

* [jsk_maps] use rospy.logerr in initialpose3d (`#1298 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1298>`_)

  * remove unnecessary ;
  * use rospy.logerr instead of print

* Update map of eng2-8f, add room 83A3 (`#1247 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1247>`_)
* update fetch dock spot (`#1274 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1274>`_)
* remove pr2 from eng2 7f map (`#1265 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1265>`_)
* fix a typo in REAME.md of jsk_maps (`#1269 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1269>`_)
* [jsk_maps] Add change-floor server to change map (`#1251 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1251>`_)
* Refine 73B2 fridge demo motion (`#1264 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1264>`_)

  * remove pr2 from eng2 7f map
  * update 73b2 fridge front spot
  * [jsk_maps/change-floor.l] Modified key value of change-costmap-publish-frequency
  * [jsk_maps/change-floor.l] Fixed typo. inamenitialpose3d  -> initialpose3d
  * [jsk_maps/change-floor.l] Remove shebang
  * [jsk_maps/change-floor-server.l] Refactor unix:usleep -> unix:sleep
  * [jsk_maps/change-floor-server.l] Change publish_frequncy of costmap to reload it
  * [jsk_maps/change-floor.l] Add change-costmap-publish-frequency function
  * [jsk_maps/change-floor.l] Return t in change-floor function
  * [jsk_maps/change-floor.l] Refactor
  * [jsk_maps/change-floor.l] Fixed lookup transform
  * [jsk_maps/change-floor-server.l] Add comments
  * [jsk_maps/change-floor-server.l] Add *tfl* initialization
  * [jsk_maps/change-floor.l] Add *tfl* error exception
  * [jsk_maps] Add change-floor server
  * update map of eng2-8f, add room 83A3

* update position of fetch's charge dock (`#1245 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1245>`_)
* Add fetch's dock spot to /spots_marker_array (`#1244 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1244>`_)
* update map of 73B2, set gray pixels to white (`#1243 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1243>`_)
* change map of eng2-7f (`#1233 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1233>`_)
* elevator_move_base_pr2: Fixes for pr2 indigo demo (`#1238 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1238>`_)

  * jsk_maps: cleanup old codes

* [jsk_maps] refactor publish_spot.l & add test (`#1227 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1227>`_)

  * jsk_maps: disable test on hydro
  * jsk_maps: cleanup old codes
  * jsk_maps: add test
  * jsk_maps: publish_spots.l: refactor / add option to select using pictograms or pins

* jsk_maps/tools/publish_spot.l: add message when map is selected (`#1214 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1214>`_)
* [jsk_maps][building-model.l] add :rooms / :current-room (`#1221 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1221>`_)

  * jsk_maps/tools/publish_spot.l: add emssage when map is selected

* Contributors: Naoaki Kanazawa, Kei Okada, Koki Shinjo, Naoya Yamaguchi, Shingo Kitagawa, Yuki Furuta, Yuto Uchimi, Taichi Higashide, Iory Yanokura, Yoshiki Obinata

0.0.4 (2017-03-15)
------------------
* [jsk_maps] add README.md
* [jsk_maps] cleanup; add option: launch_map_server
* [jsk_maps][dump-map-info.l] fix: indent
* add ARCHDIR for LinuxARM
* use image filename relative to current yaml file
* [jsk_maps/raw_maps/eng2-7f-0.05.pgm] update 73b2 7f map
* Contributors: Kei Okada, Yuki Furuta

0.0.3 (2016-02-11)
------------------
* add keepout maps
* use queue_size=1
* add depends to jsk_rviz_plugins
* [jsk_maps/src/eng2-scene.l] add /eng2/8f/room83b1-front spot
* Contributors: Yuki Furuta, Kei Okada

0.0.2 (2015-11-26)
------------------
* Ignore auto generated files
* [jsk_maps] update jsk eng2 7f 73b2 map data
* Contributors: Yuki Furuta, Kentaro Wada

0.0.1 (2015-06-11)
------------------
* [jsk_maps] add multi_map_server deps for jsk_maps
* [jsk_demos] remove rosmake files
* Merge pull request #154 from garaemon/fc-ocs-ui-integration-go-pos
  [drc_com_common, drc_task_common] Add image_view2 based user interface.
* [jsk_maps] Use find_package to lookup euslisp directory
* [jsk_maps] Use find_package to lookup euslisp directory
* [jsk_maps] Disable X by unsetting DISPLAY variable during compilation to avoid init-x error on catkin build
* [jsk_maps] Disable X by unsetting DISPLAY variable during compilation to avoid init-x error on catkin build
* [jsk_maps] use euslisp to generate map files rather than roseus
* Publish spot as pictogram rather than marker
* hook ros function on catkin_make in roseus
* Revert "add geometry_msgs to dependencies"
  This reverts commit a78e42e9d41a8485cf1dd001bf95b7a2a1734f62.
* add geneus msg to dependencies
* add geometry_msgs to dependencies
* fix typo at CMakeLists.txt
* add launch files to script output dependencies
* fix roseus bin directory
* not use rosrun in CMakeLists.txt
* add empty_map launch, pgm and yaml
* add post proess to generate launchs automatically
* avoid multiple spot decleration
* remove autogenerated yaml under git
* changed eng2-scene.l
* changed 73b2 map
* changed open-fridge-door function in pr2-action.l
* Fix package name, topic names for hydro
* add package.xml to jsk_maps
* modified open-fridge-door function in pr2-action.l
* Rename exeucutable name of the package multi_map_server from map_server to multi_map_server
* update error message
* add error message line and prevent many warning message if the target-tf is empty
* add eng2-1f maps and elevator-outside spot
* add 1f outside
* h-kamada changed eng2-scene.l for eng2/1f
* update eng2-7f-0.05.pgm
* remove 73b2 doot to go to the elevator!!
* support MACHINE argument on start_map_eng2.launch and start_map_eng8.launch
* update spot /eng2/7f/room73B2-counter-side
* updated eng2/7f/73b2
* update spots at room73b2
* update spot at /eng2/7f/73b2
* change some map data
* updated eng2/7f/73b2
* update room73B2 map
* cleanup codes and publish tf at 100 hz, see [#208]
* change name of laundry spot
* fix typo estq -> setq
* add laundry to eng8-scene.l
* add room602, ro10
* support scene parameter in tool/publish_spot
* commit fix dump-map-info, this differs eng2-3f-0.05.yaml from  [-13.0, -48.5, 0] to [-8.0, -53.5, 0]
* remove autogenerated files when make clean
* hvs2rgv is suport in jskeus r845
* fix room610 position
* set /eng8/6f/610 tf frame [#78]
* update eng8-6f-0.05.pgm
* set eng8 default floor
* add auto generation eng #8 map
* add 610 map
* fixed fridge-front spot
* update room73B2 map
* updated 73B2 map
* fix: all nodes should be identical
* update 73B2 scene
* update 73B2 map
* updated pose of refridge
* updated 73a3 floor
* fix radius from 60->61 to avoid face-to-face alignment warning message
* fix publish_spot
* set output screen : publish_spot.l
* add to l aunch publish_spot.l
* fix : update spot publisher, read /map_tf_mux/selected to get current map and publish only that floor
* update spot publisher, read /map_tf_mux/selected to get current map and publish only that floor
* update map of 73B2 on 20120731
* fixed coords of fridge-front
* new spots
* new map for 7f
* changed fridge-front spot
* new map for 73a3
* update eng2-7f-map
* udpate room73b2 map
* uddate fridge position
* jsk_maps requires roseus
* add dependency for multi_map_server
* new maps for 73b1
* moved the spots for picking cups
* moved mini kitchen spot
* remove noize in map eng6-*f
* added seminar B and C
* remove noize in map eng6-3f
* add eng6 scene
* fixed the rotation on the eng6 maps, the center is elevator now
* changed check condition of spot to convert owl
* added eng6 all floor
* added a map of eng6-3f
* fixed eus -> owl script, remove vert type map (it is default now)
* fixed building-model and eng8 model
* incf x on coe-shelf
* comma is needed in the code
* add spot for opening fridge
* added coe-spot and kitchen spot
* move unassocd spots in rooms, same as room
* added ipad demo spot for new map
* add eng2 5f map, not edited version
* changed map manager, map_server2 -> multi_map_manager in jsk_maps
* use rosrun roseus rosues instead of roseus
* last argument of static_transform_publisher is not hz, but msec
* remove internal package name from manifest
* make default-floor in dump script
* changed to use vertical building model in jsk_maps
* update vertical map launch
* changed room wall position
* moved conatenated-map utilities to old directory
* fixed m -> mm to dumped yaml file
* fixed typos in jsk_maps
* removed a file to be generated, add pose initialize node
* jsk_maps become a set of 2D maps
* remove jsk_maps/raw_maps/*.yaml, these files will be generated
* updated map of /eng2/floor7
* added test version of vertical building map, added initialpose3d script (2.5D?)
* removed code for copy spots from sub-scene
* update 73B2 room map in jsk_maps
* add yaml file for each piece of map
* add TF for room73B2 origin
* convert spost from converted scene model (73B2)
* added new spot kitchen-front
* add vertical building model, only for visualizing
* update map information of subway
* update spot for taking elevator
* update spots in eng2 scene
* chmage the make file for copygenerated owl
* copy mini-kitchen of 7th floor to 8th floor
* change spots position in elevator
* update jsk_map,semantic
* fix gimp modification error, remove stairs to avoid falling down
* update : force publish spot 3time in the beginning
* set scale, change rate 0.1->0.01
* add publish_spot.l
* fix mini-kitchen, use ~f instaed of ~a to avoid round-off error
* fix typo LaboratryRoom -> LaboratoryRoom
* update room73b2-front-kitchen-table
* update room73b2
* update eng2-cups with better annotaiton labels
* update convert to eng2-cups
* add and update spots in eng2 map
* added sample picture of cup-map
* add Makeifle for temporary
* code to write cup-annotated map eng2-cup.jpg
* add UtilityRoom to knowrob-type to 7a-mini-kitchen
* fix name of cups in scene1
* add cup2, cup4, cup6f to room
* fixed the place of cupf4
* add name to all cups, add images to cup5,6
* add room73b2-front-kitchen-table and update mini-kitchen-A-inside
* modify cup pose in mini-kitchen, update demo script
* update cup pose in mini-kitchen
* remove 2 cups, and add mini kitchen in 7f-A
* clearfied the map
* update size of mini kitchen
* add mini kitchen room in eng2/7f
* add kitchen type for 83B1 room
* add types for rooms in eng2
* minor
* add multiple types for a object in owl(yaml) convert script
* remove the chen's chair from Rm.73A3
* modify the cup position in 73a3
* renamed files from png to jpg
* updated linktoimagefile tag for new cup images
* add mit-mug to jsk_map:scene1
* added parsing for data properties, fixed rotation matrix
* update converter to add {data,object}-properties
* table in the center of 73b2 is x-leg-desk
* add 5cups in 73b1,73b2,83b1 for scene1
* not to use flatten for avoiding stack overflow
* add room83b1, change to switchable the output of semantic_map converter
* add scene1 for using another environment
* rotate the tables in subway
* fix the translation.z of rooms and floors
* fix :rot -> :worldrot in obj dump method
* adjusted coords for vertical map, removed print
* fix the cashier position in subway shop
* change the objects coordinates in subway model and reduce the z-axis gap for visualization
* add some rooms in 2f and subway simple models
* added simple script that first converts the jsk map from euslisp to yaml, and second, converts the yaml file to owl
* add 73a3 to eng2-scene
* fixed naming of instances
* fix the rotation of spots near subway-shop
* add frame_id also for spots
* fix the problem of wrong translation of rooms in 8th floor
* fix the pose of elevator panels, spot above the ground to not convert
* fix the bug of spot position in global, add type of floors ,elevators and rooms
* added support for places
* add spot relationship
* add spot properties for knowrob
* refactored conversion script
* i forget the update vertices in object
* fixed small errors
* add floors and rooms to conversion
* add room type, and fix type
* add visualization of converting objects
* add rooms in 8f and elevator object
* fixed parent link
* added options for vertical floor stacking and scaling
* added options for vertical floor stacking and scaling
* updated jsk_maps eng2-7f-0.05.pgm
* modify eng2-7f (add new 73a3 map)
* add room73B2 table position
* accounted for objects in the yaml map
* fix the bbox calcuration code, we have to print :bodies to move assoced data
* bbox for semantic map is not needed to move-to
* output bounding box size in object pose coordination
* output global pose to yaml, fix the object pose
* added jsk-to-ias-mappings to conversion rule function
* do not recursive, ???
* fix the bounding box pose
* add unique name to eng2 corridors
* add convert function
* define floor as a body, plane-building-scene have rooms slot
* add 73b2 room to eng2 map
* add room object in eng2
* move eng2/7f/73B1 to correct position
* add Rm.606,610 in eng8 building
* float-vector in eng2-map, #f -> dynamic alloc
* add color to visualize floor region
* fix the transform option, :world
* fix eng8 definition in jsk_maps
* - added frame information to exported map
  - fixed bugs in matrix generation
* fix eng8 map data
* added script for converting a YAML map of a building, floors, and rooms into an OWL representation
* move-to option is parent coords, before assoc to parent
* add room coords to converted eusmodel
* add room definition in eng2-map.yaml
* added a package for storing jsk's semantic maps.
* add scale command from mm to m
* add a line to run by rosrun command
* add convert script from jsk_map to ias_semantic_map
* added spots for 31A again
* added spots for 31A
* change 31A-front spot
* fixed y posiiont ofo room31A
* add position for manipurate printer
* define spots in eng2 as relative to floor origin
* changed spot in eng2.3f
* added spots for eng8/2f
* add spot in eng2.3f
* add eng2-3f map by mikita
* add eng2 spot around subway-shop
* update eng2/2f map, draw wall line
* draw line in order not to fall the robot
* move spot coordinates around the elevator
* add nakanishi and kurotobi desk position
* added eng2-tf-73B2
* add two spots in eng2-scene
* add four spots on /eng2/7f
* correct floor regions of eng2
* add eng2-scene to jsk_maps
* generate pgm from raw_map, add rosdep.yaml to install convert command
* update map of Eng2.7F
* update elevator panel model method
* remove noiz from table in eng8/room602
* set coorect map yaml
* add eng2 map, but yaml is not correct
* add eng2 map, now only 8F is correct
* change dynamic -> static map tf
* add student-afairs position
* add room 606 in eng8.pgm
* change pr2eus camera name, tune B1F elevator position, modify floor check function
* add spot infomationf on eng8.bld B1F
* small fix, launch node name
* add eng8.B1F map to eng8 map
* add map raw data of B1,1,6F of eng8.bld and 8F of eng2.build
* spot should be coded in euslisp
* change make command to irteusgl -> rosrun euslisp irteusgl
* generate euslisp map object from yaml
* add jsk_maps for eng8 building at hongo campus
* Contributors: Yuki Furuta, JSK applications, Kamada Hitoshi, Kei Okada, Ryohei Ueda, Satoshi Iwaishi, Yuto Inagaki, Haseru Chen, Yusuke Furuta, Kazuto Murase, Eisoku Kuroiwa, Kuze Lars, Manabu Saito, Hiroyuki Mikita, Shunichi Nozawa, Youhei Kakiuchi
