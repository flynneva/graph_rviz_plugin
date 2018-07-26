#ifndef RVIZ_GRAPH_PLUGIN_CONFIGURE_AXES_HPP
#define RVIZ_GRAPH_PLUGIN_CONFIGURE_AXES_HPP

#include <QDialog>

namespace rviz_graph_plugin
{

class AxesWindow : public QDialog
{
public:
  AxesWindow(QDialog *parent = 0);
  ~AxesWindow();
};

}

#endif
