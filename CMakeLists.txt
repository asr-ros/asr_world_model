cmake_minimum_required(VERSION 2.4.6)
project(asr_world_model)
set(CMAKE_BUILD_TYPE Debug)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DOTHER_STUFF=1")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -DOTHER_STUFF=1")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  asr_msgs
  roscpp
  rospy
  message_generation
  std_msgs
  visualization_msgs
  eigen_conversions
  tf
  asr_object_database
  dynamic_reconfigure
  asr_lib_ism
  asr_rapidxml
)
find_package(cmake_modules)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(rsc/DynamicParameters.cfg)

add_message_files(
        DIRECTORY
        msg
        FILES
        CompletePattern.msg
)

add_service_files(
    DIRECTORY srv
    FILES
    EmptyCompletePatterns.srv
    EmptyViewportList.srv
    GetFoundObjectList.srv
    GetViewportList.srv
    FilterViewportDependingOnAlreadyVisitedViewports.srv
    GetCompletePatterns.srv
    PushFoundObject.srv
    PushFoundObjectList.srv
    PushCompletePatterns.srv
    PushViewport.srv
    GetRecognizerName.srv
    GetIntermediateObjectWeight.srv
    GetAllObjectsList.srv
    GetMissingObjectList.srv
    VisualizeSampledPoses.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    asr_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    asr_object_database
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS asr_msgs roscpp rospy std_msgs message_runtime
  #DEPENDS system_lib
)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
FILE(GLOB_RECURSE HEADERS "include/*.hpp"  "include/*.h")
add_executable(${PROJECT_NAME}
    ${HEADERS}
    src/world_model_main.cpp
    src/world_model_visualizer_rviz.cpp
    src/world_model.cpp
    src/found_object_handler.cpp
    src/view_port_handler.cpp
    src/common_information_handler.cpp
    src/helper/pose_helper.cpp
    src/helper/debug_helper.cpp
    src/model/settings.cpp
    src/model/model_type.cpp
    src/model/model_object.cpp
    src/complete_patterns_handler.cpp
)


target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
    asr_msgs_gencpp
    ${PROJECT_NAME}_gencpp
    asr_object_database_gencpp
)
