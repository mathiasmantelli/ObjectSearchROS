cmake_minimum_required(VERSION 2.8.3)
project(phi_exploration)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
  phi_ocr
)

add_compile_options(-std=c++11)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
)

catkin_package(
#    CATKIN_DEPENDS
)

add_executable(explore
    src/explore/BVP.cpp
    src/explore/Configuration.cpp
    src/explore/Doors.cpp
    src/explore/GeoAssistant.cpp
    src/explore/GlutClass.cpp
    src/explore/Grid.cpp
    src/explore/Kernel.cpp
    src/explore/main.cpp
    src/explore/PioneerBase_ROS.cpp
    #src/explore/PioneerBase_ARIA.cpp
    src/explore/PioneerBase.cpp
    src/explore/Planning.cpp
    src/explore/Robot.cpp
    src/explore/SemanticBVP.cpp
    src/explore/SomeKernels.cpp
    src/explore/thinning.cpp
    src/explore/Utils.cpp
)

add_dependencies(explore phi_ocr_generate_messages_cpp)


target_link_libraries(explore
  ${catkin_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  pthread
  freeimage
)
