#ifndef CONFIG_WINDOW_HPP
#define CONFIG_WINDOW_HPP

#include "../include/rviz_graph_panel/topic.hpp"

#include <QDialog>
#include <deque>
namespace rviz_graph_plugin
{

class ConfigWindow : public QDialog
{
public:
  ConfigWindow(QDialog *parent = 0);
  ~ConfigWindow();
  std::deque<std::pair<std::string,TopicData>> displayed_topics_ ;
};

}





#endif
