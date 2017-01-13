cmake_minimum_required(VERSION 2.8.3)

project(rqt_free_gait)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_cpp
  roscpp
  rospy
  std_msgs
  free_gait_msgs
  rqt_gui
  rqt_gui_cpp
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

# Flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include(${QT_USE_FILE})

set(${PROJECT_NAME}_SRCS
  src/${PROJECT_NAME}/FreeGaitPlugin.cpp
)

set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/FreeGaitPlugin.h
)

set(${PROJECT_NAME}_UIS
  resource/FreeGaitPlugin.ui
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp roscpp rospy std_msgs free_gait_msgs rqt_gui rqt_gui_cpp
)
catkin_python_setup()

qt4_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
qt4_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})

include_directories(${${PROJECT_NAME}_INCLUDE_DIRECTORIES} 
  ${catkin_INCLUDE_DIRS} 
  include 
)

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_HDRS}
  ${${PROJECT_NAME}_MOCS}
  ${${PROJECT_NAME}_UIS_H}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY}
)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
