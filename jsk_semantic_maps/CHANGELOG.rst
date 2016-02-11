^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_semantic_maps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2016-02-11)
------------------

0.0.2 (2015-11-26)
------------------
* Ignore auto generated files
* Contributors: Kentaro Wada

0.0.1 (2015-06-11)
------------------
* [jsk_demos] remove rosmake files
* catkinize jsk_demos
* moved the spots for picking cups
* moved mini kitchen spot
* added predicate for listing all object/room tuples
* update eng2,scene1 owl model
* fixed eus -> owl script, remove vert type map (it is default now)
* update jsk_map,semantic
* update owl files
* add and update spots in eng2 map
* update add UtilityRoom property to MiniKitchen
* update name of cups from jsk_maps
* update due to r1959
* update owl file due to jsk_maps -r 1956
* modify cup pose in mini-kitchen, update demo script
* update cup pose in mini-kitchen
* remove 2 cups, and add mini kitchen in 7f-A
* add mini kitchen room in eng2/7f
* add kitchen type for 83B1 room
* add types for rooms in eng2
* added knowrob_omics as dependency
* added heatmap visualization, latest maps
* modify the cup position in 73a3
* updated scene1.owl for different image name
* updated new scene1 for images
* add mit-mug to jsk_map:scene1
* updated vertical map, load scene1.owl now by default, ie no second argument is needed
* moved some predicates to init.pl
* latest map with links to images
* Added maps for different scenarios. The map names of the scenarios correspond to the names of the scenarios that can be exported with the convert-iasmap.l tool in the jsk_maps package.
  So far there are these maps available: eng2, scene1.
  In order to load a specific map please use the command like this:
  $ rosrun rosprolog rosprolog jsk_semantic_maps scene1
  Alternatively, you can run:
  $ rosrun rosprolog rosprolog jsk_semantic_maps
  ?- scene1.
  Note: No map is loaded by default, i.e. you have to load a map like described above.
  Currently there are also the options: vert and jsk_map
  jsk_map - correspond basically to the eng2 map, this will be removed later
  vert - basically jsk_map, but the floors appear on top of each other, ONLY for viualization
* added latest map, and vertical map
* latest map, orientation of chairs and places are fixed now
* fixed error in semantic map
* fixed names, and orientations
* latest map
* 8f rooms with right translations
* latest map including spots
* added latest jsk map, removed type info in addons file, added some visualization predicates
* add example-eng2-map.launch
* add json_prolog depend
* latest map
* added room types and local frame information (parent, transformation)
* wip: generated map of eng2 building, 4 floors, and several rooms.
* added a package for storing jsk's semantic maps.
* Contributors: Kei Okada, Yaseru Chen, Kunze Laas, Manabu Saito
