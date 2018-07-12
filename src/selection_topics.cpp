#include "../include/rviz_graph_panel/selection_topics.hpp"

namespace rviz_graph_plugin
{

TopicWindow::TopicWindow(QDialog *parent)
{
  setWindowTitle("Topics Selection");

}

TopicWindow::~TopicWindow()
{
}

void TopicWindow::detectTopics()
{
  Q_EMIT setEnabled(false);

  ros::master::V_TopicInfo topics;
  if (!ros::master::getTopics(topics))
  {
    Q_EMIT TopicWindow::displayMessageBox("Error getting topics", "Could not retrieve the topics names.", "",
                             QMessageBox::Icon::Critical);
    Q_EMIT setEnabled(true);
    return;
  }

  ros::master::V_TopicInfo supported_topics;
  for (auto topic : topics)
  {
    if (topic.datatype == "std_msgs/Bool" ||
        topic.datatype == "std_msgs/Int8" ||
        topic.datatype == "std_msgs/UInt8" ||
        topic.datatype == "std_msgs/Int16" ||
        topic.datatype == "std_msgs/UInt16" ||
        topic.datatype == "std_msgs/Int32" ||
        topic.datatype == "std_msgs/UInt32" ||
        topic.datatype == "std_msgs/Int64" ||
        topic.datatype == "std_msgs/UInt64" ||
        topic.datatype == "std_msgs/Float32" ||
        topic.datatype == "std_msgs/Float64" ||
        topic.datatype == "std_msgs/String" ||
        topic.datatype == "std_msgs/Time" ||
        topic.datatype == "std_msgs/Duration")
      supported_topics.push_back(topic);
  }

  if (supported_topics.empty())
  {
    Q_EMIT setEnabled(false);

    QDialog *no_topics_dialog = new QDialog(this);
    no_topics_dialog->setWindowTitle("No supported topic");
    QVBoxLayout *layout = new QVBoxLayout;
    no_topics_dialog->setLayout(layout);
    layout->addWidget(new QLabel("Error with topics, no supported topics found.\n"));
    QPushButton *ok_button = new QPushButton("Ok");
    connect(ok_button,SIGNAL(clicked()),SLOT(this->accepted()));
  }



}



}
