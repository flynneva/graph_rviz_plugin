#ifndef GRAPH_RVIZ_PLUGIN_SELECTION_TOPICS_HPP
#define GRAPH_RVIZ_PLUGIN_SELECTION_TOPICS_HPP

#include <deque>
#include <mutex>
#include <QCheckBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QLabel>
#include <QMessageBox>
#include <QScrollArea>
#include <QVBoxLayout>
#include <ros/ros.h>
#include <graph_rviz_plugin/topic_data.hpp>

namespace graph_rviz_plugin
{

class SelectionTopics : public QDialog
{
  Q_OBJECT

public:
  SelectionTopics(std::shared_ptr<ros::NodeHandle> nh,
                  std::deque<std::shared_ptr<TopicData>> already_displayed_topics,
                  QDialog *parent = 0);
  ~SelectionTopics();
  void detectTopics();
  std::deque<std::shared_ptr<TopicData>> displayed_topics_;
  std::deque<std::shared_ptr<TopicData>> already_displayed_topics_;
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::master::V_TopicInfo supported_topics_;

Q_SIGNALS:
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

protected Q_SLOTS:
  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);
  void okClicked();

private:
  std::vector<QCheckBox *> topic_buttons_;
};

}

#endif
