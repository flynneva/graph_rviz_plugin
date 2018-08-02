#ifndef RVIZ_GRAPH_PANEL_CONFIGURE_HPP
#define RVIZ_GRAPH_PANEL_CONFIGURE_HPP

#include <QCheckBox>
#include <QComboBox>
#include <QColor>
#include <QDialog>
#include <QFormLayout>
#include <QSpinBox>
#include <QStringList>
#include <QTableWidget>
#include <deque>
#include <rviz_graph_panel/topic_data.hpp>

namespace rviz_graph_plugin
{

class Configure : public QDialog
{
  Q_OBJECT

public:
  Configure(std::deque<std::shared_ptr<TopicData>> displayed_topics, QDialog *parent = 0);
  ~Configure();
  std::vector<QCheckBox *> topic_buttons_;
  std::vector<QComboBox *> topic_combobox_;
  std::vector<QSpinBox *> topic_spinbox_;

protected Q_SLOTS:
  void okClicked();

private:
  std::deque<std::shared_ptr<TopicData>> displayed_topics_;
};

}

#endif
