#ifndef GRAPH_RVIZ_PLUGIN_GRAPH_SETTINGS_HPP
#define GRAPH_RVIZ_PLUGIN_GRAPH_SETTINGS_HPP

#include <QCheckBox>
#include <QComboBox>
#include <QColor>
#include <QDialog>
#include <QFormLayout>
#include <QSpinBox>
#include <QStringList>
#include <QTableWidget>
#include <deque>
#include <graph_rviz_plugin/topic_data.hpp>
#include <graph_rviz_plugin/topic_color.hpp>

namespace graph_rviz_plugin
{

class GraphSettings : public QDialog
{
Q_OBJECT

public:
  GraphSettings(std::deque<std::shared_ptr<TopicData>> displayed_topics, QDialog *parent = 0);
  ~GraphSettings();

protected Q_SLOTS:
  void okClicked();

private:
  std::deque<std::shared_ptr<TopicData>> displayed_topics_;
  TopicColor topic_color_;
  std::vector<QCheckBox *> topic_buttons_;
  std::vector<QComboBox *> topic_combobox_;
  std::vector<QSpinBox *> topic_spinbox_;
};

}

#endif
