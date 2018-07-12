#ifndef TOPIC_WINDOW_HPP
#define TOPIC_WINDOW_HPP

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>
#endif

#include <QFuture>
#include <QLabel>
#include <QDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>
namespace rviz_graph_plugin
{

 class TopicWindow : public QDialog
 {
 public:
   TopicWindow(QDialog *parent = 0);
   ~TopicWindow();
   void detectTopics();

 };



}





#endif
