#ifndef RVIZ_GRAPH_PLUGIN_HPP
#define RVIZ_GRAPH_PLUGIN_HPP

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
#include <deque>
#include <memory>
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
  GraphPanel(QWidget* parent = 0);
  virtual ~GraphPanel();
  std::shared_ptr<ros::NodeHandle> nh_;
  void GraphUpdate();

Q_SIGNALS:
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

protected Q_SLOTS:
  void displayMessageBoxHandler(const QString title,
                                const QString message,
                                const QString info_msg = "",
                                const QMessageBox::Icon icon = QMessageBox::Information);
  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;
  void startStopClicked();
  void topicsSelectionClicked();
  void configClicked();
  void axesClicked();
  void clearClicked();

private:
  QPushButton *start_stop_button_;
  QCustomPlot *plot_;
  std::deque<std::shared_ptr<TopicData>> displayed_topics_;

};

}

#endif
