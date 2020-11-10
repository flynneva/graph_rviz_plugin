[![Institut Maupertuis logo](http://www.institutmaupertuis.fr/media/gabarit/logo.png)](http://www.institutmaupertuis.fr)

# Overview
This package contains two Qt RViz panels that allows users to draw graphs of topics values dynamically (line graph) and to visualise image histograms.

Line graph supported built-in types are:
- `bool`
- `duration`
- `float32`
- `float32MultiArray`
- `float64`
- `float64MultiArray`
- `int8`
- `int8MultiArray`
- `int16`
- `int16MultiArray`
- `int32`
- `int32MultiArray`
- `int64`
- `int64MultiArray`
- `time`
- `uint8`
- `uint8MultiArray`
- `uint16`
- `uint16MultiArray`
- `uint32`
- `uint32MultiArray`
- `uint64`
- `uint64MultiArray`

:warning: It is not possible to draw graphs from values inside custom messages, see https://gitlab.com/InstitutMaupertuis/graph_rviz_plugin/issues/11

Histogram supported types are:
- sensor_msgs/Image

# Screenshots
- Graph line

![Line graph panel](documentation/line_graph_panel.png)

- Histogram

![Histogram panel](documentation/histogram_panel.png)

:information_source: The panel configuration is saved/loaded in the RViz configuration file, this includes which topics are recorded, the graph settings and settings.

# Dependencies

## rosdep
Install, initialize and update [rosdep](https://wiki.ros.org/rosdep).

# Compiling
Create a catkin workspace and clone the project:

```bash
mkdir -p catkin_workspace/src
cd catkin_workspace/src
git clone https://gitlab.com/InstitutMaupertuis/graph_rviz_plugin.git
cd ..
```

## Resolve ROS dependencies
```bash
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

## Compile
```bash
catkin_make
```

# User manual
Add the panel in Rviz by going to `Panels > Add New Panel > grah_rviz_plugin` and add `Line graph` or `Histogram`.

## Start / pause
Allows to start recording or pause the recording. When paused, it is possible to inspect the graph and the data are still updated in the background (pausing does not make you loose data).

## Stop
Stop a recording, no data will be recorded anymore.

## Topics
Allows to choose which topics to record. Cannot be modified when started or paused.

![Topic selection](documentation/topic_selection.png)

## Graph settings
Allows to change the appearance of each graph.

- Display: Whether to display the graph or not (this does not affect the data update in the background).
- Color: The color of the graph line.
- Thickness: The thickness of the graph line.

## Graph line settings
Allow to change the panel configuration

![Line graph settings](documentation/line_graph_settings.png)

- Refresh frequency: At what rate the graph updates, you can lower the graph update if performance becomes a problem.
- Enabled legend: Show or hide the legend.
- Y axis
  - Y auto: The Y axis will be automatically scaled depending on the topics values (hidden topics are included in the scaling).
  - Y max / Y min: Allows to specify a fixed Y range.
- X axis window time: If enabled, the graph will only show the latest `x` data depending on the time value.

## Histogram settings
Allows to change the panel configuration

![Line graph settings](documentation/histogram_settings.png)
- Topic: Select the image topic.
- Start / Stop: Start or stop the data acquisition.
- Refresh frequency: At what rate the graph updates, you can lower the graph update if performance becomes a problem.
- Bins selection: Define how many bins will be used to compute the histogram.

## Export
Allows to export the graph to a PNG, PDF or JPEG file.

## Reset
Clears the graphs, underlying data and topics to be recorded.

# `subtopic_repub.py`

Republish a subtopic into another topic, useful to republish a field you want to plot, e.g.:

Usage:
```bash
rosrun graph_rviz_plugin subtopic_repub.py /topic/subtopic/field /my_republished_field [queue_size (default 10)]
```

Examples:
```bash
rosrun graph_rviz_plugin subtopic_repub.py /imu/angular_velocity/x /angular_velocity_x
rosrun graph_rviz_plugin subtopic_repub.py /hullbot/sensors/bno055/imu/angular_velocity_covariance[0] /angular_velocity_covariance_0
```

Example output:
```bash
rosrun graph_rviz_plugin subtopic_repub.py /imu/angular_velocity/x /angular_velocity_x
[INFO] [1597403303.283071]: Republishing subtopic: /imu/angular_velocity/x into topic: /angular_velocity_x (with queue_size: 10)
[INFO] [1597403303.362539]: Input topic is of type: sensor_msgs/Imu
[INFO] [1597403303.363576]: Got submsg of type: std_msgs/Float64
```

