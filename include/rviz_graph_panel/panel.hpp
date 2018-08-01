#ifndef RVIZ_GRAPH_PANEL_PANEL_HPP
#define RVIZ_GRAPH_PANEL_PANEL_HPP

#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <ros/ros.h>
#include <ros/service.h>
#include <ros/time.h>
#include <rviz_graph_panel/configure_graph.hpp>
#include <rviz_graph_panel/configure.hpp>
#include <rviz_graph_panel/qcustomplot.h>
#include <rviz_graph_panel/selection_topics.hpp>
#include <rviz_graph_panel/topic_data.hpp>
#include <rviz/panel.h>
#include <thread>

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
  void graphClicked();
  void clearClicked();
  void graphUpdate();

private:
  QPushButton *start_stop_button_;
  QPushButton *topic_button_;
  QTimer *graph_refresh_timer_;
  QCustomPlot *plot_;
  std::deque<std::shared_ptr<TopicData>> displayed_topics_;
  std::atomic<bool> legend_enable_;
  std::atomic<bool> yaxis_rescale_auto_;
  std::atomic<bool> window_time_enable_;
  double y_min_ = 0;
  double y_max_ = 1;
  double w_time_ = 1;
  double refresh_period_ms_ = 16; // in milliseconds
};

}

#endif
