jsk_maps
========

Map tools for JSK Buildings

## Usage

- With `map_server` (e.g. for `PR2`)

  ``` bash
  roslaunch jsk_maps start_map_eng2.launch
```

- Without `map_server` (e.g. for `fetch`)

  ```bash
  roslaunch jsk_maps start_map_eng2.launch launch_map_server:=false
```

## Sync real world update to map

  1. Cleanup your room (In order to create clean map)
  1. Create partial map with your robots (currently PR2)
  
  ```bash
  robot start
  roslaunch jsk_pr2_startup pr2_2dnav.launch
  rosrun map_server map_saver
  # Then move robot with joystick
```

  1. Merge the pertial map into floor map
  
  Edit `pgm` file with Gimp
  
  1. Update models (objects / spots)
  
    1. Update spots (e.g. https://github.com/jsk-ros-pkg/jsk_demos/pull/1176)
    2. Update objects (e.g. https://github.com/euslisp/EusLisp/pull/154)
    3. Create new objects (e.g. https://github.com/jsk-ros-pkg/euslib/pull/188)