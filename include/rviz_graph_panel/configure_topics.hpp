#ifndef CONFIG_WINDOW_HPP
#define CONFIG_WINDOW_HPP

#include <QDialog>
namespace rviz_graph_plugin
{

class ConfigWindow : public QDialog
{
public:
  ConfigWindow(QDialog *parent = 0);
  ~ConfigWindow();
};

}





#endif
