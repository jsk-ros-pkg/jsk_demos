welcome_to_jsk_fetch
====================

# Demo flow

1. Undock fetch and move fetch to reception place (e.g. eng2 7f, in front of the elevator)

2. launch the main program

  ```
  ssh fetch@fetch1075
  roslaunch welcome_to_jsk_fetch guide.launch
  ```

3. Fetch waits and searches for the guest by face recognition. Note that wearing a mask reduces recognition accuracy.

4. Fetch asks where to go, so you specify your destination by voice. Available destinations are "じぇーえすけー", "ろうか", and "えれべーたー"

5. Fetch go to the destination.

# Launched programs

The following programs are launched.

- Main roseus program (guide.l)
  - Move robot
- `opencv_apps/face_detection.launch`
  - Face detection to find people
- `julius_ros/julius.launch`
  - Speech recognition
  - Usually we use julius.launch launched by fetch1075, so julius.launch is not launched by this launch)
