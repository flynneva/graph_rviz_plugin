#ifndef CONFIG_WINDOW_HPP
#define CONFIG_WINDOW_HPP

#include "../include/rviz_graph_panel/topic.hpp"

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

class ConfigWindow : public QDialog
{
  Q_OBJECT

public:
  ConfigWindow(QDialog *parent = 0);
  ~ConfigWindow();
  std::deque<std::shared_ptr<TopicData>> displayed_topics_;
  void WindowConstruction();
  std::vector<QCheckBox *> topic_buttons_;
  std::vector<QComboBox *> topic_combobox_;
  std::vector<QSpinBox *> topic_spinbox_;

protected Q_SLOTS:
  void okClicked();

private :


};

}





#endif
