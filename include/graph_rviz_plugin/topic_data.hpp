#ifndef GRAPH_RVIZ_PLUGIN_TOPIC_DATA_HPP
#define GRAPH_RVIZ_PLUGIN_TOPIC_DATA_HPP

#include <deque>
#include <exception>
#include <graph_rviz_plugin/qcustomplot.h>
#include <mutex>
#include <QMessageBox>
#include <QObject>
#include <QVector>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>

namespace graph_rviz_plugin
{

class TopicData : public QObject
{
Q_OBJECT
public:
  TopicData(std::string topic_name,
            std::string topic_type,
            std::shared_ptr<ros::NodeHandle>,
            QObject *parent = nullptr);

  ~TopicData();
  std::shared_ptr<ros::NodeHandle> nh_;
  const std::string topic_name_;
  const std::string topic_type_;
  QColor color_ = Qt::GlobalColor::black;
  unsigned thickness_ = 1;
  bool displayed_ = true;
  bool data_update_ = true;
  QCPGraph::LineStyle line_style_ = QCPGraph::lsLine;
  QCPScatterStyle::ScatterShape scatter_shape_ = QCPScatterStyle::ssCross;
  QVector<double> topic_data_;
  QVector<double> topic_time_;
  QVector<double> getTopicData();
  QVector<double> getTopicTime();
  void startRefreshData();
  void stopRefreshData();
  void clearData();

Q_SIGNALS:
  void vectorUpdated(std::string topic_name);
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

protected Q_SLOTS:
  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);

private:
  ros::Subscriber sub_;
  ros::Time begin_;
  std::mutex data_mutex_;
  void boolCallback(const std_msgs::BoolConstPtr &msg);
  void durationCallback(const std_msgs::DurationConstPtr &msg);
  void float32Callback(const std_msgs::Float32ConstPtr &msg);
  void float64Callback(const std_msgs::Float64ConstPtr &msg);
  void int8Callback(const std_msgs::Int8ConstPtr &msg);
  void int16Callback(const std_msgs::Int16ConstPtr &msg);
  void int32Callback(const std_msgs::Int32ConstPtr &msg);
  void int64Callback(const std_msgs::Int64ConstPtr &msg);
  void timeCallback(const std_msgs::TimeConstPtr &msg);
  void uint8Callback(const std_msgs::UInt8ConstPtr &msg);
  void uint16Callback(const std_msgs::UInt16ConstPtr &msg);
  void uint32Callback(const std_msgs::UInt32ConstPtr &msg);
  void uint64Callback(const std_msgs::UInt64ConstPtr &msg);
  void pushData(const double Data, const ros::Time now);
};

}

#endif
