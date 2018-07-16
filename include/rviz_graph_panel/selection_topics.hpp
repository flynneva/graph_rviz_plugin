#ifndef TOPIC_WINDOW_HPP
#define TOPIC_WINDOW_HPP

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>
#endif

#include <QCheckBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFuture>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QRadioButton>
#include <QSpinBox>
#include <QVBoxLayout>
namespace rviz_graph_plugin
{

class SelectionTopics : public QDialog
{
Q_OBJECT

public:
  SelectionTopics(QDialog *parent = 0);
  ~SelectionTopics();
  void detectTopics();
  std::map<std::string, std::string> displayed_topics_;

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
