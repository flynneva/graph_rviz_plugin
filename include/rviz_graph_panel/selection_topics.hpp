#ifndef RVIZ_GRAPH_PANEL_SELECTION_TOPICS_HPP
#define RVIZ_GRAPH_PANEL_SELECTION_TOPICS_HPP

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/service.h>
#include <rviz/panel.h>
#endif

#include <QCheckBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFuture>
#include <QLabel>
#include <QMessageBox>
#include <QPen>
#include <QPushButton>
#include <QRadioButton>
#include <QSpinBox>
#include <QVBoxLayout>
#include <deque>
#include <mutex>
#include <rviz_graph_panel/qcustomplot.h>
#include <rviz_graph_panel/topic.hpp>

namespace rviz_graph_plugin
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

Q_SIGNALS:
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

protected Q_SLOTS:
  void displayMessageBoxHandler(const QString title,
                                const QString message,
                                const QString info_msg = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);

  void okClicked();

private:
  ros::master::V_TopicInfo supported_topics_;
  std::vector<QCheckBox *> topic_buttons_;

};

}

#endif
