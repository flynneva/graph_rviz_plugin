#include <graph_rviz_plugin/topic_data.hpp>

namespace graph_rviz_plugin
{

TopicData::TopicData(std::string topic_name,
                     std::string topic_type,
                     std::shared_ptr<ros::NodeHandle> nh,
                     QObject *parent) :
    QObject(parent),
    nh_(nh),
    topic_name_(topic_name),
    topic_type_(topic_type)
{
}

TopicData::~TopicData()
{
}

void TopicData::startRefreshData()
{
  begin_ = ros::Time::now();

  if (!nh_)
    throw std::runtime_error("Node handle not initialized!");

  if (topic_type_ == "std_msgs/Bool")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::boolCallback, this);
  else if (topic_type_ == "std_msgs/Duration")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::durationCallback, this);
  else if (topic_type_ == "std_msgs/Float32")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::float32Callback, this);
  else if (topic_type_ == "std_msgs/Float64")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::float64Callback, this);
  else if (topic_type_ == "std_msgs/Int8")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::int8Callback, this);
  else if (topic_type_ == "std_msgs/Int16")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::int16Callback, this);
  else if (topic_type_ == "std_msgs/Int32")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::int32Callback, this);
  else if (topic_type_ == "std_msgs/Int64")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::int64Callback, this);
  else if (topic_type_ == "std_msgs/UInt8")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::uint8Callback, this);
  else if (topic_type_ == "std_msgs/UInt16")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::uint16Callback, this);
  else if (topic_type_ == "std_msgs/UInt32")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::uint32Callback, this);
  else if (topic_type_ == "std_msgs/UInt64")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::uint64Callback, this);
  else
  {
    ROS_ERROR_STREAM("Could not find callback for topic type " << topic_type_);
    return;
  }
}

void TopicData::clearData()
{
  topic_data_.clear();
  topic_time_.clear();
}

void TopicData::stopRefreshData()
{
  sub_.shutdown();
}

void TopicData::pushData(const double data, const ros::Time now)
{
  double time = (now.toSec() - begin_.toSec());

  try
  {
    topic_time_.push_back(time);
    topic_data_.push_back(data);
  }
  catch (const std::exception &e)
  {
    ROS_WARN_STREAM("Memory full, acquisition stop");
    Q_EMIT displayMessageBox("Fatal Error", "Memory is full, acquisition is stopped.", "",
                             QMessageBox::Icon::Critical);
    sub_.shutdown();
  }
}

void TopicData::boolCallback(const std_msgs::BoolConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  const double data(msg->data);
  pushData(data, ros::Time::now());
  data_update_ = true;
}

void TopicData::durationCallback(const std_msgs::DurationConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  const double data = (msg->data).toSec();
  pushData(data, ros::Time::now());
  data_update_ = true;
}

void TopicData::float32Callback(const std_msgs::Float32ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  const double data = (double) (msg->data);
  pushData(data, ros::Time::now());
  data_update_ = true;
}

void TopicData::float64Callback(const std_msgs::Float64ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  const double data = (double) (msg->data);
  pushData(data, ros::Time::now());
  data_update_ = true;
}

void TopicData::int8Callback(const std_msgs::Int8ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  const double data = (double) (msg->data);
  pushData(data, ros::Time::now());
  data_update_ = true;
}

void TopicData::int16Callback(const std_msgs::Int16ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  double data = (double) (msg->data);
  pushData(data, ros::Time::now());
  data_update_ = true;
}

void TopicData::int32Callback(const std_msgs::Int32ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  double data = (double) (msg->data);
  pushData(data, ros::Time::now());
  data_update_ = true;
}

void TopicData::int64Callback(const std_msgs::Int64ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  double data = (double) (msg->data);
  pushData(data, ros::Time::now());
  data_update_ = true;
}

void TopicData::timeCallback(const std_msgs::TimeConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  double data = (msg->data).toSec();
  pushData(data, ros::Time::now());
  data_update_ = true;
}

void TopicData::uint8Callback(const std_msgs::UInt8ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  const double data = (double) (msg->data);
  pushData(data, ros::Time::now());
  data_update_ = true;
}

void TopicData::uint16Callback(const std_msgs::UInt16ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  const double data = (double) (msg->data);
  pushData(data, ros::Time::now());
  data_update_ = true;
}

void TopicData::uint32Callback(const std_msgs::UInt32ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  const double data = (double) (msg->data);
  pushData(data, ros::Time::now());
  data_update_ = true;
}

void TopicData::uint64Callback(const std_msgs::UInt64ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  const double data = (double) (msg->data);
  pushData(data, ros::Time::now());
  data_update_ = true;
}

QVector<double> TopicData::getTopicData()
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  return topic_data_;
}

QVector<double> TopicData::getTopicTime()
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  return topic_time_;
}

void TopicData::displayMessageBoxHandler(const QString title,
                                         const QString message,
                                         const QString info_msg,
                                         const QMessageBox::Icon icon)
{
  QMessageBox msg_box;
  msg_box.setWindowTitle(title);
  msg_box.setText(message);
  msg_box.setInformativeText(info_msg);
  msg_box.setIcon(icon);
  msg_box.setStandardButtons(QMessageBox::Ok);
  msg_box.exec();
}

}
