#include "../include/rviz_graph_panel/topic.hpp"
//FIXME The time's gestion

namespace rviz_graph_plugin
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
  begin_ = ros::Time::now();

  if (nh_)

  sub_ = nh_->subscribe(topic_name_, 1, &TopicData::timeCallback, this);

  if (topic_type == "std_msgs/Bool")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::boolCallback, this);
  else if (topic_type == "std_msgs/Duration")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::durationCallback, this);
  else if (topic_type == "std_msgs/Float32")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::float32Callback, this);
  else if (topic_type == "std_msgs/Float64")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::float64Callback, this);
  else if (topic_type == "std_msgs/Int8")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::int8Callback, this);
  else if (topic_type == "std_msgs/Int16")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::int16Callback, this);
  else if (topic_type == "std_msgs/Int32")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::int32Callback, this);
  else if (topic_type == "std_msgs/Int64")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::int64Callback, this);
  else if (topic_type == "std_msgs/UInt8")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::uint8Callback, this);
  else if (topic_type == "std_msgs/UInt16")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::uint16Callback, this);
  else if (topic_type == "std_msgs/UInt32")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::uint32Callback, this);
  else if (topic_type == "std_msgs/UInt64")
    sub_ = nh_->subscribe(topic_name_, 1, &TopicData::uint64Callback, this);
  else
  {
    ROS_ERROR_STREAM("Could not find callback for topic type " << topic_type_);
    return;
  }
}

TopicData::~TopicData()
{

}

void TopicData::boolCallback(const std_msgs::BoolConstPtr &msg)
{
  if (msg->data == true)
  {
    topic_data.push_back(1);
    double time = (ros::Time::now()).toSec() - begin_.toSec();
    topic_time.push_back(time);
    Q_EMIT vectorUpdated(topic_name_);
    return;
  }
  else
  {
    topic_data.push_back(0);
    double time = (ros::Time::now()).toSec() - begin_.toSec();
    topic_time.push_back(time);
    Q_EMIT vectorUpdated(topic_name_);
    return;
  }

}

void TopicData::durationCallback(const std_msgs::DurationConstPtr &msg)
{
  double tmp;
  tmp = (msg->data).toSec();
  topic_data.push_back(tmp);
  double time = (ros::Time::now()).toSec() - begin_.toSec();
  topic_time.push_back(time);
  Q_EMIT vectorUpdated(topic_name_);
  return;
}

void TopicData::float32Callback(const std_msgs::Float32ConstPtr &msg)
{
  double tmp;
  tmp = (double)(msg->data);
  topic_data.push_back(tmp);
  double time = (ros::Time::now()).toSec() - begin_.toSec();
  topic_time.push_back(time);
  Q_EMIT vectorUpdated(topic_name_);
  return;
}

void TopicData::float64Callback(const std_msgs::Float64ConstPtr &msg)
{
  double tmp;
  tmp = (double)(msg->data);
  topic_data.push_back(tmp);
  double time = (ros::Time::now()).toSec() - begin_.toSec();
  topic_time.push_back(time);
  Q_EMIT vectorUpdated(topic_name_);
  return;
}

void TopicData::int8Callback(const std_msgs::Int8ConstPtr &msg)
{
  double tmp;
  tmp = (double)(msg->data);
  topic_data.push_back(tmp);
  double time = (ros::Time::now()).toSec() - begin_.toSec();
  topic_time.push_back(time);
  Q_EMIT vectorUpdated(topic_name_);
  return;
}

void TopicData::int16Callback(const std_msgs::Int16ConstPtr &msg)
{
  double tmp;
  tmp = (double)(msg->data);
  topic_data.push_back(tmp);
  double time = (ros::Time::now()).toSec() - begin_.toSec();
  topic_time.push_back(time);
  Q_EMIT vectorUpdated(topic_name_);
  return;
}

void TopicData::int32Callback(const std_msgs::Int32ConstPtr &msg)
{
  double tmp;
  tmp = (double)(msg->data);
  topic_data.push_back(tmp);
  double time = (ros::Time::now()).toSec() - begin_.toSec();
  topic_time.push_back(time);
  Q_EMIT vectorUpdated(topic_name_);
  return;
}

void TopicData::int64Callback(const std_msgs::Int64ConstPtr &msg)
{
  double tmp;
  tmp = (double)(msg->data);
  topic_data.push_back(tmp);
  double time = (ros::Time::now()).toSec() - begin_.toSec();
  topic_time.push_back(time);
  Q_EMIT vectorUpdated(topic_name_);
  return;
}

void TopicData::timeCallback(const std_msgs::TimeConstPtr &msg)
{
  //FIXME maybe be usefull
  /*
   QDateTime time;
   time.setMSecsSinceEpoch(msg->data.sec * 1e3 + msg->data.nsec / 1e6);
   */
  double tmp;
  tmp = (msg->data).toSec();
  topic_data.push_back(tmp);
  double time = (ros::Time::now()).toSec() - begin_.toSec();
  topic_time.push_back(time);
  Q_EMIT vectorUpdated(topic_name_);
  return;
}

void TopicData::uint8Callback(const std_msgs::UInt8ConstPtr &msg)
{
  double tmp;
  tmp = (double)(msg->data);
  topic_data.push_back(tmp);
  double time = (ros::Time::now()).toSec() - begin_.toSec();
  topic_time.push_back(time);
  Q_EMIT vectorUpdated(topic_name_);
  return;
}

void TopicData::uint16Callback(const std_msgs::UInt16ConstPtr &msg)
{
  double tmp;
  tmp = (double)(msg->data);
  topic_data.push_back(tmp);
  double time = (ros::Time::now()).toSec() - begin_.toSec();
  topic_time.push_back(time);
  Q_EMIT vectorUpdated(topic_name_);
  return;
}

void TopicData::uint32Callback(const std_msgs::UInt32ConstPtr &msg)
{
  double tmp;
  tmp = (double)(msg->data);
  topic_data.push_back(tmp);
  double time = (ros::Time::now()).toSec() - begin_.toSec();
  topic_time.push_back(time);
  Q_EMIT vectorUpdated(topic_name_);
  return;
}

void TopicData::uint64Callback(const std_msgs::UInt64ConstPtr &msg)
{
  double tmp;
  tmp = (double)(msg->data);
  topic_data.push_back(tmp);
  double time = (ros::Time::now()).toSec() - begin_.toSec();
  topic_time.push_back(time);
  Q_EMIT vectorUpdated(topic_name_);
  return;
}

}
