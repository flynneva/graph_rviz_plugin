#include <graph_rviz_plugin/panel.hpp>

namespace graph_rviz_plugin
{

GraphPanel::GraphPanel(QWidget *parent) :
  rviz::Panel(parent),
  nh_(std::make_shared<ros::NodeHandle>()),
  start_pause_button_(new QPushButton),
  topic_button_(new QPushButton("Topics")),
  stop_button_(new QPushButton("Stop")),
  graph_settings_button_(new QPushButton("Graph settings")),
  graph_refresh_timer_(new QTimer(this)),
  plot_(new QCustomPlot)
{
  connect(this, SIGNAL(enable(const bool)), this, SLOT(setEnabled(const bool)));
  setName("Graph");
  setObjectName(getName());

  yaxis_rescale_auto_ = true;
  window_time_enable_ = false;
  legend_enable_ = true;
  graph_stopped_ = true;
  y_max_ = 10;

  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, SIGNAL(displayMessageBox(const QString, const QString, const QString, const QMessageBox::Icon)),
          this, SLOT(displayMessageBoxHandler(const QString, const QString, const QString, const QMessageBox::Icon)));

  connect(start_pause_button_, SIGNAL(clicked()), SLOT(startPauseClicked()));
  connect(stop_button_, SIGNAL(clicked()), SLOT(stopClicked()));
  connect(topic_button_, SIGNAL(clicked()), SLOT(topicsSelectionClicked()));
  connect(graph_settings_button_, SIGNAL(clicked()), SLOT(graphSettingsClicked()));
  QPushButton *settings_button = new QPushButton("Settings");
  connect(settings_button, SIGNAL(clicked()), SLOT(settingsClicked()));
  QPushButton *reset_button = new QPushButton("Reset");
  connect(reset_button, SIGNAL(clicked()), SLOT(resetClicked()));

  start_pause_button_->setText("Start");

  QHBoxLayout *button_layout = new QHBoxLayout();
  button_layout->addWidget(start_pause_button_);
  button_layout->addWidget(stop_button_);
  button_layout->addWidget(topic_button_);
  button_layout->addWidget(graph_settings_button_);
  button_layout->addWidget(settings_button);
  button_layout->addWidget(reset_button);

  QVBoxLayout *layout = new QVBoxLayout();
  setLayout(layout);
  layout->addLayout(button_layout);
  layout->addWidget(plot_);

  connect(graph_refresh_timer_, SIGNAL(timeout()), this, SLOT(graphUpdate()));

  QFont legendFont = font();
  legendFont.setPointSize(9);
  plot_->legend->setFont(legendFont);
  plot_->legend->setBrush(QBrush(Qt::GlobalColor::white));

  start_pause_button_->setEnabled(false);
  stop_button_->setEnabled(false);
  graph_settings_button_->setEnabled(false);
}

GraphPanel::~GraphPanel()
{
  nh_->shutdown();
  graph_refresh_timer_->stop();
}

void GraphPanel::graphInit()
{
  for (unsigned i = 0; i < displayed_topics_.size(); i++)
  {
    plot_->addGraph();
    plot_->graph(i)->removeFromLegend();
    plot_->graph(i)->setName(QString::fromStdString(displayed_topics_.at(i)->topic_name_));
    plot_->graph(i)->addToLegend();
  }
}

void GraphPanel::graphSettingsUpdate()
{
  for (unsigned i = 0; i < displayed_topics_.size(); i++)
  {
    plot_->graph(i)->setPen(QPen(displayed_topics_.at(i)->color_, displayed_topics_.at(i)->thickness_));
    plot_->graph(i)->setLineStyle(displayed_topics_.at(i)->line_style_);
    plot_->graph(i)->setVisible(displayed_topics_.at(i)->displayed_);
  }

  plot_->replot();
}

void GraphPanel::graphUpdate()
{
  for (unsigned i = 0; i < displayed_topics_.size(); i++)
  {
    if (displayed_topics_.at(i)->data_update_ == true)
    {
      QVector<double> topic_data = displayed_topics_.at(i)->getTopicData();
      QVector<double> topic_time = displayed_topics_.at(i)->getTopicTime();

      if (yaxis_rescale_auto_ == true)
        plot_->yAxis->rescale(true);
      else
        plot_->yAxis->setRange(y_min_, y_max_);

      if (window_time_enable_ == false)
        plot_->xAxis->rescale(true);
      else
        plot_->xAxis->setRange(topic_time.last(), w_time_, Qt::AlignRight);

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

  unsigned i(0);
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

  while (1)
  {
    QString topic_name;
    QString topic_type;
    int topic_thickness;
    int color_index;

    if (!config.mapGetString("topic_" + QString::number(i) + "_name", &topic_name))
      break;

    if (!config.mapGetString("topic_" + QString::number(i) + "_type", &topic_type))
      break;

    if (!config.mapGetInt("topic_" + QString::number(i) + "_thickness", &topic_thickness))
      break;

    if (!config.mapGetInt("topic_" + QString::number(i) + "_color", &color_index))
      break;

    std::shared_ptr<TopicData> topic_data = std::make_shared<TopicData>(topic_name.toStdString(), topic_type.toStdString(), nh_);
    topic_data->thickness_ = topic_thickness;
    topic_data->color_ = topic_color_class_.getColorFromIndex(color_index);
    displayed_topics_.push_back(topic_data);
    ++i;
  }

  if (displayed_topics_.empty())
    return;

  start_pause_button_->setEnabled(true);
  stop_button_->setEnabled(true);
  graph_settings_button_->setEnabled(true);
  graphInit();
  graphSettingsUpdate();
  Q_EMIT enableLegend(legend_enable_);
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

  for (unsigned i = 0; i < displayed_topics_.size(); i++)
  {
    config.mapSetValue("topic_" + QString::number(i) + "_name", QString::fromStdString(displayed_topics_.at(i)->topic_name_));
    config.mapSetValue("topic_" + QString::number(i) + "_type", QString::fromStdString(displayed_topics_.at(i)->topic_type_));
    config.mapSetValue("topic_" + QString::number(i) + "_thickness", displayed_topics_.at(i)->thickness_);
    config.mapSetValue("topic_" + QString::number(i) + "_color", topic_color_class_.getIndexFromColor(displayed_topics_.at(i)->color_));
  }
}

void GraphPanel::startPauseClicked()
{
  if ((graph_refresh_timer_->isActive()) == false)
  {
    graph_refresh_timer_->start(refresh_period_ms_);
    start_pause_button_->setText("Pause");
    plot_->setInteraction(QCP::iRangeZoom, false);
    plot_->setInteraction(QCP::iRangeDrag, false);
    topic_button_->setEnabled(false);
    stop_button_->setEnabled(true);

    if (graph_stopped_ == true)
    {
      graph_stopped_ = false;
      plot_->clearGraphs();
      plot_->clearPlottables();
      plot_->replot();
      graphInit();

      for (unsigned i = 0; i < displayed_topics_.size(); i++)
      {
        displayed_topics_.at(i)->clearData();
        displayed_topics_.at(i)->startRefreshData();
      }

      graphSettingsUpdate();
      graphUpdate();
    }

    return;
  }

  else
  {
    graph_refresh_timer_->stop();
    start_pause_button_->setText("Start");
    plot_->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);
    return;
  }
}

void GraphPanel::stopClicked()
{
  topic_button_->setEnabled(true);

  if ((graph_refresh_timer_->isActive()) == true)
    startPauseClicked();

  for (unsigned i = 0; i < displayed_topics_.size(); i++)
    displayed_topics_.at(i)->stopRefreshData();

  graph_stopped_ = true;
}

void GraphPanel::resetClicked()
{
  if ((graph_refresh_timer_->isActive()) == true)
    startPauseClicked();

  for (unsigned i = 0; i < displayed_topics_.size(); i++)
  {
    displayed_topics_.at(i)->stopRefreshData();
    displayed_topics_.at(i)->clearData();
  }

  displayed_topics_.clear();
  plot_->legend->setVisible(false);
  plot_->clearGraphs();
  plot_->clearPlottables();
  plot_->replot();

  start_pause_button_->setEnabled(false);
  stop_button_->setEnabled(false);
  graph_settings_button_->setEnabled(false);
  topic_button_->setEnabled(true);

  Q_EMIT configChanged();
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

  if (topic_window->exec())
  {
    if ((graph_refresh_timer_->isActive()) == true)
      startPauseClicked();

    resetClicked();
    displayed_topics_ = topic_window->displayed_topics_;

    for (unsigned i = 0; i < displayed_topics_.size(); i++)
      displayed_topics_.at(i)->color_ = topic_color_class_.getColorFromIndex(i % ((topic_color_class_.colors_list_).size()));

    graphInit();
    graphSettingsUpdate();
    Q_EMIT configChanged();
    Q_EMIT enableLegend(legend_enable_);

    if (displayed_topics_.empty() == false)
    {
      start_pause_button_->setEnabled(true);
      graph_settings_button_->setEnabled(true);
    }
  }
}

void GraphPanel::graphSettingsClicked()
{
  Configure *configure_topics = new Configure(displayed_topics_);

  if (configure_topics->exec())
    graphSettingsUpdate();
}

void GraphPanel::settingsClicked()
{
  ConfigureGraph *configure_graph = new ConfigureGraph(yaxis_rescale_auto_, window_time_enable_,
      legend_enable_,  y_min_, y_max_, w_time_, refresh_period_ms_);

  if (!configure_graph->exec())
    return;

  window_time_enable_ = configure_graph->window_time_enable_;
  yaxis_rescale_auto_ = configure_graph->scale_auto_;
  w_time_ = configure_graph->w_time_;
  y_min_ = configure_graph->y_min_;
  y_max_ = configure_graph->y_max_;
  legend_enable_ = configure_graph->legend_enable_;
  Q_EMIT enableLegend(legend_enable_);
  refresh_period_ms_ = (configure_graph->refresh_period_) * 1000; // Convert seconds in milliseconds
  Q_EMIT configChanged();
}

void GraphPanel::enableLegend(bool legend_enable)
{
  plot_->legend->setVisible(legend_enable);
}

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(graph_rviz_plugin::GraphPanel, rviz::Panel)

