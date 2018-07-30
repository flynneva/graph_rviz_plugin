#ifndef RVIZ_GRAPH_PANEL_CONFIGURE_HPP
#define RVIZ_GRAPH_PANEL_CONFIGURE_HPP

#include <rviz_graph_panel/topic_data.hpp>
#include <QCheckBox>
#include <QComboBox>
#include <QColor>
#include <QDialog>
#include <QFormLayout>
#include <QSpinBox>
#include <QStringList>
#include <deque>

namespace rviz_graph_plugin
{

class Configure : public QDialog
{
  Q_OBJECT

public:
  Configure(QDialog *parent = 0);
  ~Configure();
  // FIXME Make displayed_topics_ private
  std::deque<std::shared_ptr<TopicData>> displayed_topics_;
  void WindowConstruction();
  std::vector<QCheckBox *> topic_buttons_;
  std::vector<QComboBox *> topic_combobox_;
  std::vector<QSpinBox *> topic_spinbox_;

protected Q_SLOTS:
  void okClicked();
};

}

#endif