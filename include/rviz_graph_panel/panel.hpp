#ifndef RVIZ_GRAPH_PANEL_PANEL_HPP
#define RVIZ_GRAPH_PANEL_PANEL_HPP

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>
#include <ros/time.h>
#endif

#include <QFuture>
#include <QLabel>
#include <QMessageBox>
#include <QPen>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QTimer>
#include <deque>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <thread>
#include <algorithm>
#include <QtConcurrent/QtConcurrentRun>
#include <rviz_graph_panel/qcustomplot.h>
#include <rviz_graph_panel/configure_axes.hpp>
#include <rviz_graph_panel/configure_topics.hpp>
#include <rviz_graph_panel/selection_topics.hpp>
#include <rviz_graph_panel/topic.hpp>

namespace rviz_graph_plugin
{

class GraphPanel : public rviz::Panel
{
  Q_OBJECT

public:
  GraphPanel(QWidget *parent = 0);
  virtual ~GraphPanel();
  std::shared_ptr<ros::NodeHandle> nh_;

Q_SIGNALS:
  void enable(const bool);
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

protected Q_SLOTS:
  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);
  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;
  void startStopClicked();
  void topicsSelectionClicked();
  void configClicked();
  void axesClicked();
  void clearClicked();
  void graphUpdate();

private:
  QPushButton *start_stop_button_;
  QTimer *graph_refresh_timer_;
  QCustomPlot *plot_;
  std::deque<std::shared_ptr<TopicData>> displayed_topics_;
  bool graph_running_ = true ;
  bool yaxis_rescale_auto_ = true;
  bool window_time_enable_ = false; 
  double y_min_ = 0;
  double y_max_ = 1;
  double w_time_ = 1;


};

}

#endif
