cmake_minimum_required(VERSION 3.0.2)
project(techrie_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  actionlib_msgs
  actionlib
)

add_message_files(FILES
  InteractionEvent.msg
  DesireState.msg
  HumanState.msg
)

add_service_files(FILES
  AskForItem.srv
)

add_action_files(DIRECTORY action FILES
  Invite.action
  PaintStroke.action
  FetchTool.action
  ExpressEmotion.action
  ShowArt.action
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs actionlib_msgs sensor_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  nodes/behavior_manager.py
  nodes/desire_manager.py
  nodes/skills/skill_express_emotion.py
  nodes/skills/skill_invite.py
  nodes/skills/skill_paint_stroke.py
  nodes/skills/skill_fetch_tool.py
  nodes/skills/skill_show_art.py
  nodes/logging/diary_logger.py
  nodes/social/ask_and_confirm_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
