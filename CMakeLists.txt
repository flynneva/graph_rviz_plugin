cmake_minimum_required(VERSION 2.8.3)
project(rviz_graph_panel)
add_compile_options(-std=c++11 -Wall -Wextra)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  class_loader
  roscpp
  rqt_gui
  rviz
)

## Find Qt5 modules
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5PrintSupport REQUIRED)

set(
  PROJECT_QT_PANELS_TARGETS
  ${PROJECT_NAME}
)

set(
  PROJECT_TARGETS
  ${PROJECT_QT_PANELS_TARGETS}
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_TARGETS}
  CATKIN_DEPENDS
  roscpp
  rqt_gui
  rviz
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## QCustomPlot
qt5_wrap_cpp(
  qcustomplot_MOCS
  include/${PROJECT_NAME}/qcustomplot.hpp
)
add_library(
  ${PROJECT_NAME}_qcustomplot
  ${calibrationwindow_MOCS}
  src/qcustomplot.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_qcustomplot
  Qt5::Widgets
  Qt5::Gui
  Qt5::PrintSupport
)
add_dependencies(
  ${PROJECT_NAME}_qcustomplot
  ${catkin_EXPORTED_TARGETS}
)

## SelectionTopics
qt5_wrap_cpp(
  selection_topics_MOCS
  include/${PROJECT_NAME}/selection_topics.hpp
)
add_library(
  ${PROJECT_NAME}_selection_topics
  ${calibrationwindow_MOCS}
  src/selection_topics.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_selection_topics
  Qt5::Widgets
  Qt5::Gui
)
add_dependencies(
  ${PROJECT_NAME}_selection_topics
  ${catkin_EXPORTED_TARGETS}
)


## ConfigureTopics
qt5_wrap_cpp(
  configure_topics_MOCS
  include/${PROJECT_NAME}/configure_topics.hpp
)
add_library(
  ${PROJECT_NAME}_configure_topics
  ${calibrationwindow_MOCS}
  src/configure_topics.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_configure_topics
  Qt5::Widgets
  Qt5::Gui
)
add_dependencies(
  ${PROJECT_NAME}_configure_topics
  ${catkin_EXPORTED_TARGETS}
)


## ConfigureAxes
qt5_wrap_cpp(
  configure_axes_MOCS
  include/${PROJECT_NAME}/configure_axes.hpp
)
add_library(
  ${PROJECT_NAME}_configure_axes
  ${calibrationwindow_MOCS}
  src/configure_axes.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_configure_axes
  Qt5::Widgets
  Qt5::Gui
)
add_dependencies(
  ${PROJECT_NAME}_configure_axes
  ${catkin_EXPORTED_TARGETS}
)


## Qt panel
qt5_wrap_cpp(
  panel_MOCS
  include/${PROJECT_NAME}/panel.hpp
)
add_library(
  ${PROJECT_NAME}
  ${panel_MOCS}
  src/panel.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  Qt5::Widgets
  Qt5::Gui
  ${PROJECT_NAME}_qcustomplot
  ${PROJECT_NAME}_selection_topics
  ${PROJECT_NAME}_configure_topics
  ${PROJECT_NAME}_configure_axes
  ${catkin_LIBRARIES}

)
add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_qcustomplot
  ${PROJECT_NAME}_selection_topics
  ${PROJECT_NAME}_configure_topics
  ${PROJECT_NAME}_configure_axes
  ${catkin_EXPORTED_TARGETS}
)

# Needed by Qt / RViz / ROS
class_loader_hide_library_symbols(
  ${PROJECT_QT_PANELS_TARGETS}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

