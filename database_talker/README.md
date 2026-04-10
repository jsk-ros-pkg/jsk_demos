# database talker

## What is this?

This is sample code to generate a response/diary from a robot's experience stored in MongoDB.

## How to setup

Set up a workspace using the `rosinstall` file and compile it with `catkin build database_talker`.

## How to use

For a minimum setup, run the following command. This will start the mongodb/lifelog nodes and save the usb camera data to the database.
```bash
roslaunch database_talker sample.launch
```

To generate a diary using robot memory, execute the following command and talk to GoogleChat bot.

```bash
rosrun database_talker make_diary.py --prompt-type personality
```

## Tips

### How to test using data from a specific date without using GoogleChat.
```bash
rosrun database_talker make_diary.py --test --prompt-type personality --date 2023-03-20
```

### Stop using external DBs, this is recommended during debug phase.

Remove `mongodb_store_extras` in `jsk_robot_startup/lifelog/mongodb_replication_params.yaml`
```
-mongodb_store_extras: [["robot-database.jsk.imi.i.u-tokyo.ac.jp", 27017],["musca.jsk.imi.i.u-tokyo.ac.jp",27017]]
+mongodb_store_extras: []
```

### Force store image

An image will only be saved if a significant change is found in the image within seconds. To force the image to be saved, use the following command.
```
rostopic pub -1 /publish_trigger_mongodb_event roseus/StringStamped '{header: auto, data: debug}'
```