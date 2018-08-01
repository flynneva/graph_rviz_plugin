#include <rviz_graph_panel/panel.hpp>

namespace rviz_graph_plugin
{

GraphPanel::GraphPanel(QWidget *parent) :
  rviz::Panel(parent),
  nh_(std::make_shared<ros::NodeHandle>()),
  start_stop_button_(new QPushButton),
  topic_button_(new QPushButton("Topics")),
  graph_refresh_timer_(new QTimer(this)),
  plot_(new QCustomPlot)
{
  connect(this, SIGNAL(enable(const bool)), this, SLOT(setEnabled(const bool)));
  setName("Graph");
  setObjectName(getName());

  yaxis_rescale_auto_ = true;
  window_time_enable_ = false;
  legend_enable_ = true;

  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, SIGNAL(displayMessageBox(const QString, const QString, const QString, const QMessageBox::Icon)),
          this, SLOT(displayMessageBoxHandler(const QString, const QString, const QString, const QMessageBox::Icon)));

  connect(topic_button_, SIGNAL(clicked()), SLOT(topicsSelectionClicked()));
  QPushButton *config_button = new QPushButton("Configure");
  connect(config_button, SIGNAL(clicked()), SLOT(configClicked()));
  QPushButton *graph_button = new QPushButton("Graph settings");
  connect(graph_button, SIGNAL(clicked()), SLOT(graphClicked()));
  QPushButton *clear_button = new QPushButton("Clear");
  connect(clear_button, SIGNAL(clicked()), SLOT(clearClicked()));

  start_stop_button_->setText("Start");
  connect(start_stop_button_, SIGNAL(clicked()), SLOT(startStopClicked()));

  QHBoxLayout *button_layout = new QHBoxLayout();
  button_layout->addWidget(start_stop_button_);
  button_layout->addWidget(topic_button_);
  button_layout->addWidget(config_button);
  button_layout->addWidget(graph_button);
  button_layout->addWidget(clear_button);

  QVBoxLayout *layout = new QVBoxLayout();
  setLayout(layout);
  layout->addLayout(button_layout);
  layout->addWidget(plot_);

  connect(graph_refresh_timer_, SIGNAL(timeout()), this, SLOT(graphUpdate()));
}

GraphPanel::~GraphPanel()
{
  nh_->shutdown();
  graph_refresh_timer_->stop();
}

void GraphPanel::graphUpdate()
{
  plot_->legend->setVisible(legend_enable_);
  QFont legendFont = font();
  legendFont.setPointSize(9);
  plot_->legend->setFont(legendFont);
  plot_->legend->setBrush(QBrush(QColor(255, 255, 255, 230)));

  if ((graph_refresh_timer_->isActive()) == false)
    return;

  for (unsigned i = 0; i < displayed_topics_.size(); i++)
  {
    if (displayed_topics_.at(i)->data_update_ == true)
    {
      QVector<double> topic_data = displayed_topics_.at(i)->getTopicData();
      QVector<double> topic_time = displayed_topics_.at(i)->getTopicTime();

      if (displayed_topics_.at(i)->graph_enable_ == false)
      {
        plot_->addGraph();
        plot_->graph(i)->removeFromLegend();
        plot_->graph(i)->setName(QString::fromStdString(displayed_topics_.at(i)->topic_name_));
        plot_->graph(i)->setPen(QPen(displayed_topics_.at(i)->color_,displayed_topics_.at(i)->thickness_));
        plot_->graph(i)->setLineStyle(displayed_topics_.at(i)->line_style_);
        plot_->graph(i)->setData(topic_time, topic_data);
        plot_->graph(i)->setVisible(displayed_topics_.at(i)->displayed_);
        displayed_topics_.at(i)->data_update_ = false;
        displayed_topics_.at(i)->graph_enable_ = true;
        plot_->graph(i)->addToLegend();
      }
      
      if (yaxis_rescale_auto_ == true)
        plot_->yAxis->rescale(true);
      else
        plot_->yAxis->setRange(y_min_, y_max_);

      if (window_time_enable_ == false)
        plot_->xAxis->rescale(true);
      else
        plot_->xAxis->setRange(topic_time.last(), w_time_, Qt::AlignCenter);

      plot_->graph(i)->setPen(QPen(displayed_topics_.at(i)->color_,displayed_topics_.at(i)->thickness_));
      plot_->graph(i)->setLineStyle(displayed_topics_.at(i)->line_style_);
      plot_->graph(i)->setVisible(displayed_topics_.at(i)->displayed_);
      plot_->graph(i)->setData(topic_time, topic_data);
      displayed_topics_.at(i)->data_update_ = false;
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
  {
    bool tmp;
    if (config.mapGetBool("window_time_enable", &tmp))
      window_time_enable_ = tmp;

    if (config.mapGetBool("yaxis_rescale_auto", &tmp))
      yaxis_rescale_auto_ = tmp;

    if (config.mapGetBool("legend_enable", &tmp))
      legend_enable_ = tmp;
  }

  {
    float tmp;
    if (config.mapGetFloat("w_time", &tmp))
      w_time_ = (double)tmp;

    if (config.mapGetFloat("y_min", &tmp))
      y_min_ = (double)tmp;

    if (config.mapGetFloat("y_max", &tmp))
      y_max_ = (double)tmp;

    if (config.mapGetFloat("refresh_period_", &tmp))
      refresh_period_ms_ = (double)tmp;
  }
}

void GraphPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("window_time_enable", window_time_enable_.load());
  config.mapSetValue("yaxis_rescale_auto", yaxis_rescale_auto_.load());
  config.mapSetValue("legend_enable", legend_enable_.load());
  config.mapSetValue("w_time", w_time_);
  config.mapSetValue("y_min", y_min_);
  config.mapSetValue("y_max", y_max_);
  config.mapSetValue("refresh_period", refresh_period_ms_);
}

void GraphPanel::startStopClicked()
{
  if ((graph_refresh_timer_->isActive()) == false)
  {
    graph_refresh_timer_->start(refresh_period_ms_);
    start_stop_button_->setText("Stop");
    plot_->setInteraction(QCP::iRangeZoom, false);
    plot_->setInteraction(QCP::iRangeDrag, false);
    topic_button_->setEnabled(false);
    return;
  }
  else
  {
    graph_refresh_timer_->stop();
    start_stop_button_->setText("Start");
    plot_->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);
    topic_button_->setEnabled(true);
    return;
  }
}

void GraphPanel::topicsSelectionClicked()
{
  SelectionTopics *topic_window = new SelectionTopics(nh_, displayed_topics_);
  
  if (topic_window->supported_topics_.empty())
  {
    Q_EMIT displayMessageBox("No supported topic", "Error with topics, no supported topics found.", "",
                             QMessageBox::Icon::Warning);
    return;
  }

  if (!(topic_window->exec()))
    return;

  if ((graph_refresh_timer_->isActive()) == true)
    Q_EMIT startStopClicked();

  Q_EMIT clearClicked();
  displayed_topics_ = topic_window->displayed_topics_;
  for (unsigned i = 0; i < displayed_topics_.size(); i++)
  {
    if(i%6 == 0)
      displayed_topics_.at(i)->color_ = QColor(255, 0, 0); //red by default
    else if(i%6 == 1)
      displayed_topics_.at(i)->color_ = QColor(0, 0, 255); //blue
    else if(i%6 == 2)
      displayed_topics_.at(i)->color_ = QColor(0, 0, 0); //black
    else if(i%6 == 3)
      displayed_topics_.at(i)->color_ = QColor(0, 255, 255); //cyan
    else if(i%6 == 4)
      displayed_topics_.at(i)->color_ = QColor(255, 255, 0); //yellow
    else
      displayed_topics_.at(i)->color_ = QColor(192, 192, 192); //gray
  }
}

void GraphPanel::configClicked()
{
  Configure *configure_topics = new Configure(displayed_topics_);

  if (!(configure_topics->exec()))
    return;

  for (auto button : configure_topics->topic_buttons_)
  {
    for (unsigned i = 0; i < displayed_topics_.size(); i++)
    {
      if ((displayed_topics_.at(i)->topic_name_) == button->objectName().toStdString()
          && button->isChecked())
        displayed_topics_.at(i)->displayed_ = true;

      if ((displayed_topics_.at(i)->topic_name_) == button->objectName().toStdString()
          && !button->isChecked())
        displayed_topics_.at(i)->displayed_ = false;
    }
  }

  for (auto spinbox : configure_topics->topic_spinbox_)
  {
    for (unsigned i = 0; i < displayed_topics_.size(); i++)
    {
      if ((displayed_topics_.at(i)->topic_name_) == spinbox->objectName().toStdString())
        displayed_topics_.at(i)->thickness_ = spinbox->value();
    }
  }

  for (auto combobox : configure_topics->topic_combobox_)
  {
    for (unsigned i = 0; i < displayed_topics_.size(); i++)
    {
      if ((displayed_topics_.at(i)->topic_name_) == combobox->objectName().toStdString())
      {
        int index = combobox->currentIndex();
        if (index == 0) //blue
          displayed_topics_.at(i)->color_ = QColor(0, 0, 255);
        else if (index == 1) //red
          displayed_topics_.at(i)->color_ = QColor(255, 0, 0);
        else if (index == 2) //black
          displayed_topics_.at(i)->color_ = QColor(0, 0, 0);
        else if (index == 3) //cyan
          displayed_topics_.at(i)->color_ = QColor(0, 255, 255);
        else if (index == 4) //yellow
          displayed_topics_.at(i)->color_ = QColor(255, 255, 0);
        else //gray
          displayed_topics_.at(i)->color_ = QColor(192, 192, 192);
      }
    }
  }

  return;
}

void GraphPanel::graphClicked()
{
  ConfigureGraph *configure_graph = new ConfigureGraph(yaxis_rescale_auto_, window_time_enable_, legend_enable_,  y_min_, y_max_, w_time_, refresh_period_ms_);
  
  if (!(configure_graph->exec()))
    return;

  window_time_enable_ = configure_graph->window_time_enable_;
  yaxis_rescale_auto_ = configure_graph->scale_auto_;
  w_time_ = configure_graph->w_time_;
  y_min_ = configure_graph->y_min_;
  y_max_ = configure_graph->y_max_;
  legend_enable_ = configure_graph->legend_enable_;
  refresh_period_ms_ = (configure_graph->refresh_period_) * 1000; // Convert seconds in milliseconds
  Q_EMIT configChanged();
}

void GraphPanel::clearClicked()
{
  if ((graph_refresh_timer_->isActive()) == true)
    Q_EMIT startStopClicked();
  else
    graph_refresh_timer_->stop();

  displayed_topics_.clear();
  plot_->legend->setVisible(false);
  plot_->clearPlottables();
  plot_->replot();
}

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_graph_plugin::GraphPanel, rviz::Panel)

