#include <graph_rviz_plugin/configure_graph.hpp>

namespace graph_rviz_plugin
{

ConfigureGraph::ConfigureGraph(bool scale_auto, bool window_time_enable, bool legend_enable, double y_min, double y_max, double w_time , double refresh_period, QDialog *):
  y_min_(y_min),
  y_max_(y_max),
  w_time_(w_time),
  refresh_period_(refresh_period),
  scale_auto_(scale_auto),
  window_time_enable_(window_time_enable),
  legend_enable_(legend_enable),
  y_min_double_spin_box_(new QDoubleSpinBox),
  y_max_double_spin_box_(new QDoubleSpinBox),
  w_time_double_spin_box_(new QDoubleSpinBox),
  refresh_frequency_spin_box_(new QComboBox),
  legend_enable_button_(new QCheckBox("Enable legend"))
{
  setWindowTitle("Graph configuration");
  QVBoxLayout *main_layout = new QVBoxLayout;
  setLayout(main_layout);

  QGroupBox *general_groupbox = new QGroupBox("General");
  QVBoxLayout *general_layout = new QVBoxLayout;
  QLabel *title_combo_box = new QLabel("Refresh frequency");
  general_layout->addWidget(title_combo_box);
  QStringList frequency_list = {"20Hz", "40Hz", "60Hz", "80Hz", "100Hz"};
  refresh_frequency_spin_box_->addItems(frequency_list);

  if (refresh_period_ == 50)
    refresh_frequency_spin_box_->setCurrentIndex(0);
  else if (refresh_period_ == 25)
    refresh_frequency_spin_box_->setCurrentIndex(1);
  else if (refresh_period_ == 16)
    refresh_frequency_spin_box_->setCurrentIndex(2);
  else if (refresh_period_ == 12.5)
    refresh_frequency_spin_box_->setCurrentIndex(3);
  else
    refresh_frequency_spin_box_->setCurrentIndex(4);

  general_layout->addWidget(refresh_frequency_spin_box_);
  general_layout->addWidget(legend_enable_button_);

  if (legend_enable_ == true)
    legend_enable_button_->setChecked(true);
  else
    legend_enable_button_->setChecked(false);

  general_groupbox->setLayout(general_layout);
  main_layout->addWidget(general_groupbox);

  QGroupBox *y_axis_groupbox = new QGroupBox("Y axis");
  QFormLayout *y_axis_layout = new QFormLayout;
  QCheckBox *y_axis_autoscale = new QCheckBox("Y auto");
  y_axis_layout->addWidget(y_axis_autoscale);
  y_min_double_spin_box_->setRange(-1e9, 1e9);
  y_max_double_spin_box_->setRange(-1e9, 1e9);

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
  QCheckBox *x_axis_window_time_button = new QCheckBox("Window time");
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

  connect(button_box, &QDialogButtonBox::accepted, this, &ConfigureGraph::okClicked);
  connect(button_box, &QDialogButtonBox::rejected, this, &QDialog::reject);
}

ConfigureGraph::~ConfigureGraph()
{
}

void ConfigureGraph::yAxisAutoscale(bool checked)
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


void ConfigureGraph::xAxisWindowTime(bool checked)
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

void ConfigureGraph::okClicked()
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

  legend_enable_ = legend_enable_button_->isChecked();

  if (refresh_frequency_spin_box_->currentIndex() == 0)
    refresh_period_ = 0.05;
  else if (refresh_frequency_spin_box_->currentIndex() == 1)
    refresh_period_ = 0.025;
  else if (refresh_frequency_spin_box_->currentIndex() == 2)
    refresh_period_ = 0.016;
  else if (refresh_frequency_spin_box_->currentIndex() == 3)
    refresh_period_ = 0.0125;
  else
    refresh_period_ = 0.01;
  accept();
}

}
