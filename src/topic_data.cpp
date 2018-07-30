#include <rviz_graph_panel/topic_data.hpp>

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

  if (!nh_)
    throw std::runtime_error("Node handle not initialized!");

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

// FIXME Write a function pushData(const double data, const ros::Time now) and use it everywhere
// There is a lot of duplicated code right now

// FIXME Inform the user when the memory is full!!!! Right now the data is cleared and that's it.
// The main application will not stop the recording, the user won't get notified, this is bad.

void TopicData::boolCallback(const std_msgs::BoolConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  {
    if (msg->data == true)
    {
      try
      {
        topic_data_.push_back(1);
      }
      catch (const std::exception &e)
      {
        ROS_ERROR_STREAM("Memory is full");
        topic_data_.clear();
      }

      double time = (ros::Time::now()).toSec() - begin_.toSec();

      try
      {
        topic_time_.push_back(time);
      }
      catch (const std::exception &e)
      {
        ROS_ERROR_STREAM("Memory is full");
        topic_data_.clear();
      }

      data_update_ = true;
      return;
    }
    else
    {
      try
      {
        topic_data_.push_back(1);
      }
      catch (const std::exception &e)
      {
        ROS_ERROR_STREAM("Memory is full");
        topic_time_.clear();
      }

      double time = (ros::Time::now()).toSec() - begin_.toSec();

      try
      {
        topic_time_.push_back(time);
      }
      catch (const std::exception &e)
      {
        ROS_ERROR_STREAM("Memory is full");
        topic_time_.clear();
      }

      data_update_ = true;
      return;
    }
  }

}

void TopicData::durationCallback(const std_msgs::DurationConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  double tmp;
  tmp = (msg->data).toSec();

  try
  {
    topic_data_.push_back(tmp);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_data_.clear();
  }

  double time = (ros::Time::now()).toSec() - begin_.toSec();

  try
  {
    topic_time_.push_back(time);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_time_.clear();
  }

  data_update_ = true ;
  return;
}

void TopicData::float32Callback(const std_msgs::Float32ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  double tmp;
  tmp = (double)(msg->data);

  try
  {
    topic_data_.push_back(tmp);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_data_.clear();
  }

  double time = (ros::Time::now()).toSec() - begin_.toSec();

  try
  {
    topic_time_.push_back(time);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_time_.clear();
  }

  data_update_ = true;
  return;
}

void TopicData::float64Callback(const std_msgs::Float64ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  double tmp;
  tmp = (double)(msg->data);

  try
  {
    topic_data_.push_back(tmp);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_data_.clear();
  }

  double time = (ros::Time::now()).toSec() - begin_.toSec();

  try
  {
    topic_time_.push_back(time);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_time_.clear();
  }

  data_update_ = true;
  return;
}

void TopicData::int8Callback(const std_msgs::Int8ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  double tmp;
  tmp = (double)(msg->data);

  try
  {
    topic_data_.push_back(tmp);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_data_.clear();
  }

  double time = (ros::Time::now()).toSec() - begin_.toSec();

  try
  {
    topic_time_.push_back(time);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_time_.clear();
  }

  data_update_ = true;
  return;
}

void TopicData::int16Callback(const std_msgs::Int16ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  double tmp;
  tmp = (double)(msg->data);

  try
  {
    topic_data_.push_back(tmp);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_data_.clear();
  }

  double time = (ros::Time::now()).toSec() - begin_.toSec();

  try
  {
    topic_time_.push_back(time);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_time_.clear();
  }

  data_update_ = true;
  return;
}

void TopicData::int32Callback(const std_msgs::Int32ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  double tmp;
  tmp = (double)(msg->data);

  try
  {
    topic_data_.push_back(tmp);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_data_.clear();
  }

  double time = (ros::Time::now()).toSec() - begin_.toSec();

  try
  {
    topic_time_.push_back(time);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_time_.clear();
  }

  data_update_ = true;
  return;
}

void TopicData::int64Callback(const std_msgs::Int64ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  double tmp;
  tmp = (double)(msg->data);

  try
  {
    topic_data_.push_back(tmp);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_data_.clear();
  }

  double time = (ros::Time::now()).toSec() - begin_.toSec();

  try
  {
    topic_time_.push_back(time);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_time_.clear();
  }

  data_update_ = true;
  return;
}

void TopicData::timeCallback(const std_msgs::TimeConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  double tmp;
  tmp = (msg->data).toSec();

  try
  {
    topic_data_.push_back(tmp);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_data_.clear();
  }

  double time = (ros::Time::now()).toSec() - begin_.toSec();

  try
  {
    topic_time_.push_back(time);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_time_.clear();
  }

  data_update_ = true;
  return;
}

void TopicData::uint8Callback(const std_msgs::UInt8ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  double tmp;
  tmp = (double)(msg->data);

  try
  {
    topic_data_.push_back(tmp);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_data_.clear();
  }

  double time = (ros::Time::now()).toSec() - begin_.toSec();

  try
  {
    topic_time_.push_back(time);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_time_.clear();
  }

  data_update_ = true;
  return;
}

void TopicData::uint16Callback(const std_msgs::UInt16ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  double tmp;
  tmp = (double)(msg->data);

  try
  {
    topic_data_.push_back(tmp);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_data_.clear();
  }

  double time = (ros::Time::now()).toSec() - begin_.toSec();

  try
  {
    topic_time_.push_back(time);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_time_.clear();
  }

  data_update_ = true;
  return;
}

void TopicData::uint32Callback(const std_msgs::UInt32ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  double tmp;
  tmp = (double)(msg->data);

  try
  {
    topic_data_.push_back(tmp);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_data_.clear();
  }

  double time = (ros::Time::now()).toSec() - begin_.toSec();

  try
  {
    topic_time_.push_back(time);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_time_.clear();
  }

  data_update_ = true;
  return;
}

void TopicData::uint64Callback(const std_msgs::UInt64ConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  double tmp;
  tmp = (double)(msg->data);

  try
  {
    topic_data_.push_back(tmp);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_data_.clear();
  }

  double time = (ros::Time::now()).toSec() - begin_.toSec();

  try
  {
    topic_time_.push_back(time);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Memory is full");
    topic_time_.clear();
  }

  data_update_ = true;
  return;
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

}
