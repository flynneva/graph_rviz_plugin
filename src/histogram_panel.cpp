#include <graph_rviz_plugin/histogram_panel.hpp>

namespace graph_rviz_plugin
{

HistogramPanel::HistogramPanel(QWidget *parent) :
  rviz::Panel(parent),
  nh_(std::make_shared<ros::NodeHandle>()),
  updating_(false)
{
  connect(this, &HistogramPanel::enable, this, &HistogramPanel::setEnabled);
  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, &HistogramPanel::displayMessageBox, this, &HistogramPanel::displayMessageBoxHandler);
  setName("Histogram");
  setObjectName(getName());

  QVBoxLayout *layout = new QVBoxLayout();
  setLayout(layout);

  connect(this, &HistogramPanel::subscribeToTopic, this, &HistogramPanel::subscribeToTopicSlot);
  graph_refresh_timer_ = new QTimer(this);
  graph_refresh_timer_->setInterval(100);    // milliseconds // 10 Hz
  connect(graph_refresh_timer_, &QTimer::timeout, this, &HistogramPanel::updateChartSlot);

  QHBoxLayout *buttons(new QHBoxLayout);
  start_stop_ = new QPushButton("Start");
  start_stop_->setToolTip("Allow to start the histogram");
  start_stop_->setEnabled(false);
  connect(start_stop_, &QPushButton::clicked, this, &Panel::configChanged);
  connect(start_stop_, &QPushButton::clicked, this, [ = ]()
  {
    if (!updating_)
    {
      graph_refresh_timer_->start();
      start_stop_->setText("Stop");
      start_stop_->setToolTip("Allow to stop the histogram");
      custom_plot_->setInteraction(QCP::iRangeZoom, false);
      custom_plot_->setInteraction(QCP::iRangeDrag, false);
    }
    else
    {
      graph_refresh_timer_->stop();
      custom_plot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
      start_stop_->setText("Start");
      start_stop_->setToolTip("Allow to start the histogram");
    }
    updating_ = !updating_;
  });

  buttons->addWidget(start_stop_);
  buttons->addStretch(1);

  QComboBox *graph_refresh_frequency(new QComboBox);
  QLabel *refresh_label(new QLabel("Refresh frequency:"));
  graph_refresh_frequency->setToolTip("Allow to change the histogram display frequency");
  refresh_label->setToolTip(graph_refresh_frequency->toolTip());
  buttons->addWidget(refresh_label);
  buttons->addWidget(graph_refresh_frequency);
  buttons->addStretch(1);
  graph_refresh_frequency->addItem("1 Hz");
  graph_refresh_frequency->addItem("2 Hz");
  graph_refresh_frequency->addItem("5 Hz");
  graph_refresh_frequency->addItem("10 Hz");
  graph_refresh_frequency->addItem("20 Hz");
  graph_refresh_frequency->addItem("50 Hz");
  graph_refresh_frequency->setCurrentIndex(3); // Display by default "5 Hz"
  connect(graph_refresh_frequency, qOverload<int>(&QComboBox::currentIndexChanged), this, &Panel::configChanged);
  connect(graph_refresh_frequency, qOverload<int>(&QComboBox::currentIndexChanged), this, [ = ](const int index)
  {
    switch (index)
    {
      case 0:
        {
          graph_refresh_timer_->setInterval(1000 / 1);
          break;
        }
      case 1:
        {
          graph_refresh_timer_->setInterval(1000 / 2);
          break;
        }
      case 2:
        {
          graph_refresh_timer_->setInterval(1000 / 5);
          break;
        }
      case 3:
        {
          graph_refresh_timer_->setInterval(1000 / 10);
          break;
        }
      case 4:
        {
          graph_refresh_timer_->setInterval(1000 / 20);
          break;
        }
      case 5:
        {
          graph_refresh_timer_->setInterval(1000 / 50);
          break;
        }
      default:
        {
          graph_refresh_timer_->setInterval(1000 / 5); // Default to 5 Hz
          break;
        }
    }
  });

  custom_plot_ = new QCustomPlot;

  bars_ = new QCPBars(custom_plot_->xAxis, custom_plot_->yAxis);
  bars_->setAntialiased(false);
  bars_->setPen(QPen(QColor(255, 255, 255)));
  bars_->setBrush(QColor(0, 0, 0));
  bars_->setWidthType(QCPBars::WidthType::wtPlotCoords);
  bars_->setWidth(2);

  for (int i(0); i < 255; ++i)
    ticks_.push_back(i);

  custom_plot_->yAxis->rescale();
  custom_plot_->xAxis->setRange(0, ticks_.back());
  custom_plot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

  QComboBox *bins_selection = new QComboBox;
  QLabel *bins_selection_label = new QLabel("Histogram bins selection:");
  bins_selection->setToolTip("Allow to change the bins number used to compute the histogram");
  bins_selection_label->setToolTip(bins_selection->toolTip());
  buttons->addWidget(bins_selection_label);
  buttons->addWidget(bins_selection);
  buttons->addStretch(1);
  bins_selection->addItem("16");
  bins_selection->addItem("32");
  bins_selection->addItem("64");
  bins_selection->addItem("128");
  bins_selection->addItem("256");
  bins_selection->addItem("512");
  bins_selection->addItem("1024");
  bins_selection->addItem("2048");
  bins_selection->addItem("4096");
  bins_selection->addItem("8192");
  bins_selection->addItem("16384");
  connect(bins_selection, qOverload<int>(&QComboBox::currentIndexChanged), this, &Panel::configChanged);
  connect(bins_selection, qOverload<int>(&QComboBox::currentIndexChanged), this, [ = ](const int index)
  {
    switch (index)
    {
      case 0:
        {
          bins_value_ = 16;
          bars_->setWidth(1);
          break;
        }
      case 1:
        {
          bins_value_ = 32;
          bars_->setWidth(1);
          break;
        }
      case 2:
        {
          bins_value_ = 64;
          bars_->setWidth(1);
          break;
        }
      case 3:
        {
          bins_value_ = 128;
          bars_->setWidth(1);
          break;
        }
      case 4:
        {
          bins_value_ = 256;
          bars_->setWidth(1);
          break;
        }
      case 5:
        {
          bins_value_ = 512;
          bars_->setWidth(2);
          break;
        }
      case 6:
        {
          bins_value_ = 1024;
          bars_->setWidth(4);
          break;
        }
      case 7:
        {
          bins_value_ = 2048;
          bars_->setWidth(8);
          break;
        }
      case 8:
        {
          bins_value_ = 4096;
          bars_->setWidth(16);
          break;
        }
      case 9:
        {
          bins_value_ = 8192;
          bars_->setWidth(32);
          break;
        }
      case 10:
        {
          bins_value_ = 16394;
          bars_->setWidth(64);
          break;
        }
      default:
        {
          bins_value_ = 512; // Default to 512
          bars_->setWidth(2);
          break;
        }
    }
  });
  bins_selection->setCurrentIndex(4);

  QPushButton *topic(new QPushButton("Topic"));
  topic->setToolTip("Allow to choose and receive any compatible data");
  connect(topic, &QPushButton::clicked, this, &HistogramPanel::topicSelectionSlot);
  buttons->addWidget(topic);
  layout->addLayout(buttons);
  layout->addWidget(custom_plot_);
}

HistogramPanel::~HistogramPanel()
{
  graph_refresh_timer_->stop();
}

void HistogramPanel::updateChartSlot()
{
  std::lock_guard<std::mutex> lock(data_ticks_mutex_);

  if (data_.size() != ticks_.size() || data_.empty())
    return;

  bars_->setData(ticks_, data_, true);

  custom_plot_->yAxis->rescale();
  custom_plot_->xAxis->rescale();
  custom_plot_->replot();
}

void HistogramPanel::topicSelectionSlot()
{
  std::vector<std::string> allowed_topics;
  allowed_topics.emplace_back("sensor_msgs/Image");

  std::deque<std::shared_ptr<TopicData>> displayed_topics;
  SelectionTopics *topic_window = new SelectionTopics(nh_, displayed_topics, allowed_topics, true);

  if (topic_window->supported_topics_.empty())
  {
    Q_EMIT displayMessageBox("No supported topic", "Error with topics, no supported topics found.", "",
                             QMessageBox::Icon::Warning);
    return;
  }

  if (!topic_window->exec() || topic_window->displayed_topics_.empty())
    return;

  Q_EMIT subscribeToTopic(QString::fromStdString(topic_window->displayed_topics_.at(0)->topic_name_));
  Q_EMIT configChanged();
}

void HistogramPanel::subscribeToTopicSlot(const QString topic)
{
  custom_plot_->legend->setVisible(true);
  bars_->setName(topic);
  start_stop_->setEnabled(true);
  sub_ = nh_->subscribe(topic.toStdString(), 1, &HistogramPanel::imageCallback, this);

  if (!updating_)
    Q_EMIT

    start_stop_->click();
}

void HistogramPanel::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(data_ticks_mutex_);

  cv_bridge::CvImagePtr cv_image;
  try
  {
    cv_image = cv_bridge::toCvCopy(msg, msg->encoding);
  }
  catch (const cv::Exception &e)
  {
    std::string error("Error converting the image: ");
    error += e.what();
    ROS_ERROR_STREAM_NAMED(getName().toStdString(), error);
    return;
  }

  if (cv_image->image.channels() != 1)
  {
    ROS_ERROR_STREAM_NAMED(getName().toStdString(),
                           "Image format is not supported: Only images with one channel are supported");
    return;
  }

  cv::Mat_<float> output;

  const int bins_number(bins_value_);
  if (ticks_.size() != bins_number)
  {
    ticks_.resize(bins_number);
    data_.resize(ticks_.size());
    for (int i(0); i < ticks_.size(); ++i)
    {
      ticks_[i] = i;
    }
  }

  data_.clear();

  if (cv_image->image.depth() == CV_8U)
  {
    float range[] = {0, 255};
    const float *hist_range = {range};
    cv::calcHist(&cv_image->image, 1, nullptr, cv::Mat(), output, 1, &bins_number, &hist_range);

    for (int i(0); i < ticks_.size(); ++i)
      data_.push_back(output.at<float>(i));
  }
  else if (cv_image->image.depth() == CV_16U)
  {
    float range[] = {0, 65535};
    const float *hist_range = {range};
    cv::calcHist(&cv_image->image, 1, nullptr, cv::Mat(), output, 1, &bins_number, &hist_range);

    for (int i(0); i < ticks_.size(); ++i)
      data_.push_back(output.at<float>(i));
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(getName().toStdString(),
                           "Image format is not supported: Only CV_8U and CV_16U images are supported");
    return;
  }
}

void HistogramPanel::load(const rviz::Config &config)
{
  rviz::Panel::load(config);

  int tmp_int;
  bool tmp_bool;
  QString tmp_str;

  if (config.mapGetString(objectName() + "_subscriber", &tmp_str))
    subscribeToTopicSlot(tmp_str);

  if (config.mapGetBool(objectName() + "_running", &tmp_bool))
    graph_refresh_timer_->start();
  else
    graph_refresh_timer_->stop();

  if (config.mapGetInt(objectName() + "_bins_value", &tmp_int))
    bins_value_ = (int16_t) tmp_int;

  if (config.mapGetInt(objectName() + "_graph_refresh_timer", &tmp_int))
    graph_refresh_timer_->setInterval(tmp_int);
  else
    graph_refresh_timer_->setInterval(1000 / 5);
}

void HistogramPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);

  config.mapSetValue(objectName() + "_subscriber", QString::fromStdString(sub_.getTopic()));
  config.mapSetValue(objectName() + "_running", graph_refresh_timer_->isActive());
  config.mapSetValue(objectName() + "_bins_value", bins_value_);
  config.mapSetValue(objectName() + "_graph_refresh_timer", graph_refresh_timer_->interval());
}

void HistogramPanel::displayMessageBoxHandler(const QString title,
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

}
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(graph_rviz_plugin::HistogramPanel, rviz::Panel)

