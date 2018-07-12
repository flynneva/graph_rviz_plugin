#include <rviz_graph_panel/panel.hpp>

namespace rviz_graph_plugin
{


GraphPanel::GraphPanel(QWidget* parent) :
        rviz::Panel(parent),
        start_stop_button_ (new QPushButton)
{
  qRegisterMetaType<QMessageBox::Icon>();
  setName("Graph");
  setObjectName(getName());
  QVBoxLayout* layout = new QVBoxLayout();
  setLayout(layout);
  QHBoxLayout *button_layout = new QHBoxLayout();
  QPushButton *topic_button = new QPushButton("Topics");
  QPushButton *config_button = new QPushButton("Config");
  QPushButton *axes_button = new QPushButton("Axes");
  QPushButton *clear_button = new QPushButton("Clear");
  start_stop_button_->setText("Start");
  button_layout->addWidget(start_stop_button_);
  button_layout->addWidget(topic_button);
  button_layout->addWidget(config_button);
  button_layout->addWidget(axes_button);
  button_layout->addWidget(clear_button);
  layout->addLayout(button_layout);
  connect(start_stop_button_,SIGNAL(clicked()),SLOT(startStopClicked()));
  connect(topic_button,SIGNAL(clicked()),SLOT(topicsSelectionClicked()));
  connect(config_button,SIGNAL(clicked()),SLOT(configClicked()));
  connect(axes_button,SIGNAL(clicked()),SLOT(axesClicked()));

}

GraphPanel::~GraphPanel()
{
}

void GraphPanel::displayMessageBoxHandler(const QString title,
                                     const QString message,
                                     const QString info_msg,
                                     const QMessageBox::Icon icon)
{
  const bool old(isEnabled());
  Q_EMIT setEnabled(false);
  QMessageBox msg_box;
  msg_box.setWindowTitle(title);
  msg_box.setText(message);
  msg_box.setInformativeText(info_msg);
  msg_box.setIcon(icon);
  msg_box.setStandardButtons(QMessageBox::Ok);
  msg_box.exec();
  Q_EMIT setEnabled(old);
}

void GraphPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

void GraphPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void GraphPanel::startStopClicked()
{
  start_stop_button_->setText("Stop");
}

void GraphPanel::topicsSelectionClicked()
{
  TopicWindow *topic_window = new TopicWindow();
  topic_window->exec();

}

void GraphPanel::configClicked()
{
  ConfigWindow *configure_topics = new ConfigWindow;
  configure_topics->exec();
}

void GraphPanel::axesClicked()
{
  AxesWindow *configure_axes = new AxesWindow;
  configure_axes->exec();
}

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_graph_plugin::GraphPanel, rviz::Panel)
