cmake_minimum_required(VERSION 3.0.2)
project(pick_place_msgs)

##############################################################################
# 1) 找到並宣告相依套件 (std_msgs、geometry_msgs、message_generation)
##############################################################################
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

##############################################################################
# 2) 宣告服務檔
##############################################################################
add_service_files(
  FILES
  PickStep.srv
)

##############################################################################
# 3) 產生程式碼 (C++ / Python)；記得把依賴列進來
##############################################################################
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

##############################################################################
# 4) 對外輸出這個 package 的資訊
##############################################################################
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

##############################################################################
# 5) (可選) include 目錄：若無額外標頭可留空
##############################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

##############################################################################
# 6) 若之後要加 cpp library / node，可在這裡 add_library 或 add_executable
##############################################################################
# add_library(${PROJECT_NAME} src/lib.cpp)
# add_executable(example_node src/example.cpp)
# add_dependencies(example_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(example_node ${catkin_LIBRARIES})
