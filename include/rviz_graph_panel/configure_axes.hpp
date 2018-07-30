#ifndef RVIZ_GRAPH_PANEL_CONFIGURE_AXES_HPP
#define RVIZ_GRAPH_PANEL_CONFIGURE_AXES_HPP

#include <QDialog>
#include <QDialogButtonBox>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QRadioButton>
#include <QFormLayout>
#include <QDoubleSpinBox>

namespace rviz_graph_plugin
{

class ConfigureAxes : public QDialog
{
  Q_OBJECT

public:
  ConfigureAxes(bool scale_auto, bool window_time_enable, double y_min, double y_max , double w_time , QDialog *parent = 0);
  ~ConfigureAxes();
  double y_min_;
  double y_max_;
  double w_time_;
  bool scale_auto_;
  bool window_time_enable_;

protected Q_SLOTS:
  void yAxisAutoscale(bool checked);
  void xAxisWindowTime(bool checked);
  void okClicked();

private:
  QDoubleSpinBox *y_min_double_spin_box_;
  QDoubleSpinBox *y_max_double_spin_box_;
  QDoubleSpinBox *w_time_double_spin_box_;
};

}

#endif
