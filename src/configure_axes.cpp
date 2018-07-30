#include <rviz_graph_panel/configure_axes.hpp>

namespace rviz_graph_plugin
{

ConfigureAxes::ConfigureAxes(bool scale_auto, bool window_time_enable, double y_min, double y_max, double w_time , QDialog *):
  y_min_(y_min),
  y_max_(y_max),
  w_time_(w_time),
  scale_auto_(scale_auto),
  window_time_enable_(window_time_enable),
  y_min_double_spin_box_(new QDoubleSpinBox),
  y_max_double_spin_box_(new QDoubleSpinBox),
  w_time_double_spin_box_(new QDoubleSpinBox)
{
  setWindowTitle("Axes configuration");
  QVBoxLayout *main_layout = new QVBoxLayout;
  setLayout(main_layout);

  // FIXME Tweak y_min_double_spin_box_, y_max_double_spin_box_ ranges!

  QGroupBox *y_axis_groupbox = new QGroupBox("Y axis");
  QFormLayout *y_axis_layout = new QFormLayout;
  QRadioButton *y_axis_autoscale = new QRadioButton("Y auto");
  y_axis_layout->addWidget(y_axis_autoscale);

  if (scale_auto_ == true)
  {
    y_axis_autoscale->setChecked(true);
    y_min_double_spin_box_->setEnabled(false);
    y_max_double_spin_box_->setEnabled(false);
  }
  else
  {
    y_axis_autoscale->setChecked(false);
    y_min_double_spin_box_->setEnabled(true);
    y_max_double_spin_box_->setEnabled(true);
    y_min_double_spin_box_->setValue(y_min_);
    y_max_double_spin_box_->setValue(y_max_);
  }

  y_axis_layout->addRow("Y max: ", y_max_double_spin_box_);
  y_axis_layout->addRow("Y min: ", y_min_double_spin_box_);
  y_axis_groupbox->setLayout(y_axis_layout);
  main_layout->addWidget(y_axis_groupbox);

  QGroupBox *x_axis_groupbox = new QGroupBox("X axis");
  QFormLayout *x_axis_layout = new QFormLayout;
  QRadioButton *x_axis_window_time_button = new QRadioButton("Window time: ");
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

  connect(y_axis_autoscale, SIGNAL(toggled(bool)), SLOT(yAxisAutoscale(bool)));
  connect(x_axis_window_time_button, SIGNAL(toggled(bool)), SLOT(xAxisWindowTime(bool)));

  QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok
      | QDialogButtonBox::Cancel);
  main_layout->addWidget(button_box);

  connect(button_box, &QDialogButtonBox::accepted, this, &ConfigureAxes::okClicked);
  connect(button_box, &QDialogButtonBox::rejected, this, &QDialog::reject);
}

ConfigureAxes::~ConfigureAxes()
{
}

void ConfigureAxes::yAxisAutoscale(bool checked)
{
  if (checked == false)
  {
    y_min_double_spin_box_->setEnabled(true);
    y_max_double_spin_box_->setEnabled(true);
    scale_auto_ = false ;
  }
  else
  {
    y_min_double_spin_box_->setEnabled(false);
    y_max_double_spin_box_->setEnabled(false);
    scale_auto_ = true;
  }
}


void ConfigureAxes::xAxisWindowTime(bool checked)
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

void ConfigureAxes::okClicked()
{
  if (scale_auto_ == false)
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
