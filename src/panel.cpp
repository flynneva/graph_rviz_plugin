#include <rviz_graph_panel/panel.hpp>

namespace rviz_graph_plugin
{

GraphPanel::GraphPanel(QWidget *parent) :
  rviz::Panel(parent),
  nh_(std::make_shared<ros::NodeHandle>()),
  start_stop_button_(new QPushButton),
  graph_refresh_timer_(new QTimer(this)),
  plot_(new QCustomPlot)
{
  connect(this, SIGNAL(enable(const bool)), this, SLOT(setEnabled(const bool)));
  setName("Graph");
  setObjectName(getName());

  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, SIGNAL(displayMessageBox(const QString, const QString, const QString, const QMessageBox::Icon)),
          this, SLOT(displayMessageBoxHandler(const QString, const QString, const QString, const QMessageBox::Icon)));

  QVBoxLayout *layout = new QVBoxLayout();
  setLayout(layout);
  QHBoxLayout *button_layout = new QHBoxLayout();
  QPushButton *topic_button = new QPushButton("Topics");
  QPushButton *config_button = new QPushButton("Config");
  QPushButton *axes_button = new QPushButton("Axes");
  QPushButton *clear_button = new QPushButton("Clear");
  start_stop_button_->setText("Stop");
  button_layout->addWidget(start_stop_button_);
  button_layout->addWidget(topic_button);
  button_layout->addWidget(config_button);
  button_layout->addWidget(axes_button);
  button_layout->addWidget(clear_button);

  layout->addLayout(button_layout);
  layout->addWidget(plot_);

  graph_refresh_timer_->start(16);

  connect(start_stop_button_, SIGNAL(clicked()), SLOT(startStopClicked()));
  connect(topic_button, SIGNAL(clicked()), SLOT(topicsSelectionClicked()));
  connect(config_button, SIGNAL(clicked()), SLOT(configClicked()));
  connect(axes_button, SIGNAL(clicked()), SLOT(axesClicked()));
  connect(clear_button,SIGNAL(clicked()),SLOT(clearClicked()));
  connect(graph_refresh_timer_, SIGNAL(timeout()), this, SLOT(graphUpdate()));
}

GraphPanel::~GraphPanel()
{
  nh_->shutdown();
  graph_refresh_timer_->stop();
}

void GraphPanel::graphUpdate()
{

  plot_->legend->setVisible(true);
  QFont legendFont = font();
  legendFont.setPointSize(9);
  plot_->legend->setFont(legendFont);
  plot_->legend->setBrush(QBrush(QColor(255, 255, 255, 230)));
  //ROS_WARN_STREAM("nh_" << nh_->ok());
  ROS_INFO_STREAM("Number of Topic " << displayed_topics_.size()); //FIXME

  if (graph_running_ == false)
    return;

  for (unsigned i = 0; i < displayed_topics_.size(); i++)
  {
    if ((*displayed_topics_[i]).data_update_ == true && (*displayed_topics_[i]).graph_enable_ == false)
    {
      QVector<double> topic_data = (*displayed_topics_[i]).getTopicData();
      QVector<double> topic_time = (*displayed_topics_[i]).getTopicTime();
      plot_->addGraph();
      plot_->graph(i)->removeFromLegend();
      plot_->graph(i)->setName(QString::fromStdString((*displayed_topics_[i]).topic_name_));
      plot_->graph(i)->setPen(QPen((*displayed_topics_[i]).color_));
      plot_->graph(i)->setLineStyle((*displayed_topics_[i]).line_style_);
      plot_->graph(i)->setData(topic_time, topic_data);
      plot_->graph(i)->setVisible((*displayed_topics_[i]).displayed_);
      (*displayed_topics_[i]).data_update_ = false;
      (*displayed_topics_[i]).graph_enable_ = true;
      plot_->graph(i)->addToLegend();

    }

    if ((*displayed_topics_[i]).data_update_ == true)
    {
      QVector<double> topic_data = (*displayed_topics_[i]).getTopicData();
      QVector<double> topic_time = (*displayed_topics_[i]).getTopicTime();
      plot_->graph(i)->setPen(QPen((*displayed_topics_[i]).color_));
      plot_->graph(i)->setLineStyle((*displayed_topics_[i]).line_style_);
      plot_->xAxis->rescale(true);
      plot_->yAxis->rescale(true);
//       plot_->xAxis->setRange(*std::min_element(topic_time.constBegin(), topic_time.constEnd()), *std::max_element(topic_time.constBegin(), topic_time.constEnd()));
//       plot_->yAxis->setRange((*std::min_element(topic_data.constBegin(), topic_data.constEnd())-1), (*std::max_element(topic_data.constBegin(), topic_data.constEnd())+1)); //FIXME
      plot_->graph(i)->setVisible((*displayed_topics_[i]).displayed_);
      plot_->graph(i)->setData(topic_time, topic_data);
      (*displayed_topics_[i]).data_update_ = false;
      plot_->replot();
    }
  }
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

void GraphPanel::load(const rviz::Config &config)
{
  rviz::Panel::load(config);
}

void GraphPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void GraphPanel::startStopClicked()
{
  if (graph_running_ == false)
  {
    graph_refresh_timer_->start(16);
    start_stop_button_->setText("Stop");
    graph_running_ = true ;
    plot_->setInteraction(QCP::iRangeZoom, false);
    plot_->setInteraction(QCP::iRangeDrag, false);
    return;
  }

  if (graph_running_ == true)
  {
    graph_refresh_timer_->stop();
    start_stop_button_->setText("Start");
    graph_running_ = false;
    plot_->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);
    return;
  }

}

void GraphPanel::topicsSelectionClicked()
{

  SelectionTopics *topic_window = new SelectionTopics(nh_, displayed_topics_);

  if (!(topic_window->exec()))
    return;
  if (graph_running_ == true)
  {
    Q_EMIT startStopClicked();
  }
  Q_EMIT clearClicked();
  
  ROS_WARN_STREAM("Topic size " << displayed_topics_.size()); //FIXME
  displayed_topics_ = topic_window->displayed_topics_;
  ROS_WARN_STREAM("Nombre de personnes utilisant le shared ptr " << displayed_topics_[0].use_count()); //FIXME 
}

void GraphPanel::configClicked()
{

  ConfigWindow *configure_topics = new ConfigWindow;
  configure_topics->displayed_topics_ = displayed_topics_;
  configure_topics->WindowConstruction();

  if (!(configure_topics->exec()))
    return;

  for (auto button : configure_topics->topic_buttons_)
  {
    for (unsigned i = 0; i < displayed_topics_.size(); i++)
    {
      if (((*displayed_topics_[i]).topic_name_) == button->objectName().toStdString()
          && button->isChecked())
        (*displayed_topics_[i]).displayed_ = true;

      if (((*displayed_topics_[i]).topic_name_) == button->objectName().toStdString()
          && !button->isChecked())
        (*displayed_topics_[i]).displayed_ = false;
    }
  }

  for (auto spinbox : configure_topics->topic_spinbox_)
  {
    for (unsigned i = 0; i < displayed_topics_.size(); i++)
    {
      if (((*displayed_topics_[i]).topic_name_) == spinbox->objectName().toStdString())
        (*displayed_topics_[i]).thickness_ = spinbox->value();
    }
  }

  for (auto combobox : configure_topics->topic_combobox_)
  {
    for (unsigned i = 0; i < displayed_topics_.size(); i++)
    {
      if (((*displayed_topics_[i]).topic_name_) == combobox->objectName().toStdString())
      {
        int index = combobox->currentIndex();

        if (index == 0) //blue
          (*displayed_topics_[i]).color_ = QColor(0, 0, 255);

        if (index == 1) //red
          (*displayed_topics_[i]).color_ = QColor(255, 0, 0);

        if (index == 2) //black
          (*displayed_topics_[i]).color_ = QColor(0, 0, 0);

        if (index == 3) //cyan
          (*displayed_topics_[i]).color_ = QColor(0, 255, 255);

        if (index == 4) //yellow
          (*displayed_topics_[i]).color_ = QColor(255, 255, 0);

        if (index == 5) //gray
          (*displayed_topics_[i]).color_ = QColor(192, 192, 192);
      }
    }
  }

  return;
}

void GraphPanel::axesClicked()
{
  AxesWindow *configure_axes = new AxesWindow;
  configure_axes->exec();
}

void GraphPanel::clearClicked()
{
  if (graph_running_ == true)
    Q_EMIT startStopClicked();
  else
    graph_refresh_timer_->stop();
  std::this_thread::sleep_for (std::chrono::milliseconds(16));
  displayed_topics_.clear();
  plot_->legend->setVisible(false);
  plot_->clearPlottables();
  plot_->replot();
}


}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_graph_plugin::GraphPanel, rviz::Panel)
