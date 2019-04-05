interactive_behavior_201409
===========================

Interactive behaviors for robots

# Dialog

- Dialogflow
-- Download the service account key as JSON reading [here](https://cloud.google.com/docs/authentication/getting-started)
-- `export GOOGLE_APPLICATION_CREDENTIALS='/path/to/key'`
-- `cloud auth activate-service-account --key-file=$GOOGLE_APPLICATION_CREDENTIALS`

# Tasks

## Type of Tasks

- Onetime
  - immediate task (any on-demand task)
  - triggered task (go to dock if battery level is low)
- Repeated
  - idle task (see humans, tweet)
  - cron task (patrol)
  - deadline task (tidyup room)

## Idle Tasks

- Look Around
- Look People
- Chat

## Triggered Task

- Go to charging dock
- Listen task

## Cron Task

- Patrol building

## Deadline Task

- Tidyup room


# Attentions

## Type of Attentions

- People detection
- Face pose detection
- People identification
- Sound localization
- Touch detection
- (Anormal) object detection

# Author

Yuki Furuta <<furushchev@jsk.imi.i.u-tokyo.ac.jp>>
Ryohei Ueda <<garaemon@gmail.com>>
