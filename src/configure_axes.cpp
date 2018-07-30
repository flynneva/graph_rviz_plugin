#include <rviz_graph_panel/configure_axes.hpp>

namespace rviz_graph_plugin
{

AxesWindow::AxesWindow(bool rescale_auto, bool window_time_enable, double y_min, double y_max, double w_time , QDialog *):
  y_min_(y_min),
  y_max_(y_max),
  w_time_(w_time),
  rescale_auto_(rescale_auto),
  window_time_enable_(window_time_enable),
  y_min_double_spin_box_(new QDoubleSpinBox),
  y_max_double_spin_box_(new QDoubleSpinBox),
  w_time_double_spin_box_(new QDoubleSpinBox)

{
  setWindowTitle("Axes Configuration");
  QVBoxLayout *main_layout = new QVBoxLayout;
  setLayout(main_layout);

  QGroupBox *y_axis_groupbox = new QGroupBox("Y Axis");
  QFormLayout *y_axis_layout = new QFormLayout;
  QRadioButton *y_axis_autoscalle = new QRadioButton("Y Auto");
  y_axis_layout->addWidget(y_axis_autoscalle);

  if (rescale_auto_ == true)
  {
    y_axis_autoscalle->setChecked(true);
    y_min_double_spin_box_->setEnabled(false);
    y_max_double_spin_box_->setEnabled(false);
  }
  else
  {
    y_axis_autoscalle->setChecked(false);
    y_min_double_spin_box_->setEnabled(true);
    y_max_double_spin_box_->setEnabled(true);
    y_min_double_spin_box_->setValue(y_min_);
    y_max_double_spin_box_->setValue(y_max_);
  }
  y_axis_layout->addRow("Y Max : ", y_max_double_spin_box_);
  y_axis_layout->addRow("Y Min : ", y_min_double_spin_box_);
  y_axis_groupbox->setLayout(y_axis_layout);
  main_layout->addWidget(y_axis_groupbox);

  QGroupBox *x_axis_groupbox = new QGroupBox("X Axis");
  QFormLayout *x_axis_layout = new QFormLayout;
  QRadioButton *x_axis_window_time_button = new QRadioButton("Window Time: ");
  x_axis_layout->addWidget(x_axis_window_time_button);

  if (window_time_enable_ == false)
  {
    x_axis_window_time_button->setChecked(false);
    w_time_double_spin_box_->setEnabled(false);
  }
  else
  {
    x_axis_window_time_button->setChecked(true);
    w_time_double_spin_box_->setEnabled(true);
    w_time_double_spin_box_->setValue(w_time_);
  }

  x_axis_layout->addRow("Time: ", w_time_double_spin_box_);
  x_axis_groupbox->setLayout(x_axis_layout);
  main_layout->addWidget(x_axis_groupbox);

  connect(y_axis_autoscalle, SIGNAL(toggled(bool)), SLOT(yAxisAutoscale(bool)));
  connect(x_axis_window_time_button, SIGNAL(toggled(bool)), SLOT(xAxisWindowTime(bool)));




  QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok
      | QDialogButtonBox::Cancel);
  main_layout->addWidget(button_box);

  connect(button_box, &QDialogButtonBox::accepted, this, &AxesWindow::okClicked);
  connect(button_box, &QDialogButtonBox::rejected, this, &QDialog::reject);

}

AxesWindow::~AxesWindow()
{
}

void AxesWindow::yAxisAutoscale(bool checked)
{
  if (checked == false)
  {
    y_min_double_spin_box_->setEnabled(true);
    y_max_double_spin_box_->setEnabled(true);
    rescale_auto_ = false ;

  }
  else
  {
    y_min_double_spin_box_->setEnabled(false);
    y_max_double_spin_box_->setEnabled(false);
    rescale_auto_ = true;
  }
}


void AxesWindow::xAxisWindowTime(bool checked)
{
  if (checked == false)
  {
    w_time_double_spin_box_->setEnabled(false);
    window_time_enable_ = false;
  }
  else
  {
    w_time_double_spin_box_->setEnabled(true);
    window_time_enable_ = true;
  }
}

void AxesWindow::okClicked()
{
  if (rescale_auto_ == true)
  {
    y_min_ = y_min_double_spin_box_->value();
    y_max_ = y_max_double_spin_box_->value();
  }

  if (window_time_enable_ == true)
  {
    w_time_ = w_time_double_spin_box_->value();
  }

  accept();
}



}
