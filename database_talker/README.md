# hoge.py

What is this?
## Requirements

See `requirements.txt` for python requirements.

For ROS dependency.

- `google_chat_ros` in `jsk_3rdparty` with [this improvement](https://github.com/jsk-ros-pkg/jsk_3rdparty/pull/451)
- `dialogflow_client` in `dialogflow_task_executive` package in `jsk_3rdparty` with [this improvement](https://github.com/jsk-ros-pkg/jsk_3rdparty/pull/451)
- `mongodb_store` with https://github.com/strands-project/mongodb_store/pull/282
- CLIP VQA ros node introduced with https://github.com/jsk-ros-pkg/jsk_recognition/pull/2730.
- `ros_google_cloud_language` package in `jsk_3rdparty`

## How to use

1. Setup google chat ros with Cloud Pub/Sub
   1. prepare `credential_json` and `project_id` and `subscription_id`
2. Setup dialogflow
   1. prepare `credential_json` and `project_id`
3. Setup mongodb_store
   1. Create database with mondodb
4. Setup CLIP VQA node
   1. Make docker model
   2. Run ROS Interface node
5. Setup google cloud natural language
   1. Prepare `credential_json`

And run

```bash
roslaunch database_talker demo.launch
```
