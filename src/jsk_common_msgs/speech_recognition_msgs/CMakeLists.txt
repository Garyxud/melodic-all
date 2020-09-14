cmake_minimum_required(VERSION 2.8.3)
project(speech_recognition_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  Grammar.msg
  PhraseRule.msg
  SpeechRecognitionCandidates.msg
  Vocabulary.msg
)

add_service_files(
  FILES
  SpeechRecognition.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs message_runtime
)
