#include <rviz_graph_panel/selection_topics.hpp>

namespace rviz_graph_plugin
{

SelectionTopics::SelectionTopics(std::shared_ptr<ros::NodeHandle> nh,
                                 std::deque<std::shared_ptr<TopicData>> already_displayed_topics,
                                 QDialog *) :
  already_displayed_topics_(already_displayed_topics),
  nh_(nh)
{
  setWindowTitle("Topics selection");
  QVBoxLayout *pick_topics_dialog = new QVBoxLayout;
  setLayout(pick_topics_dialog);
  detectTopics();

  for (auto topic : supported_topics_)
  {
    QCheckBox *radio_button = new QCheckBox;
    topic_buttons_.push_back(radio_button);
    radio_button->setText(QString::fromStdString(topic.name));
    radio_button->setObjectName(QString::fromStdString(topic.name));
    radio_button->setToolTip(QString::fromStdString(topic.datatype));

    for (unsigned i = 0; i < already_displayed_topics_.size(); i++)
    {
      if ((*already_displayed_topics_[i]).topic_name_ == topic.name)
        radio_button->setChecked(true);
    }

    pick_topics_dialog->addWidget(radio_button);
  }

  QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok
      | QDialogButtonBox::Cancel);
  pick_topics_dialog->addWidget(button_box);

  connect(button_box, &QDialogButtonBox::accepted, this, &SelectionTopics::okClicked);
  connect(button_box, &QDialogButtonBox::rejected, this, &QDialog::reject);
  connect(this, SIGNAL(displayMessageBox(const QString, const QString, const QString, const QMessageBox::Icon)),
          this, SLOT(displayMessageBoxHandler(const QString, const QString, const QString, const QMessageBox::Icon)));

}

SelectionTopics::~SelectionTopics()
{
}

void SelectionTopics::detectTopics()
{
  ros::master::V_TopicInfo topics;

  if (!ros::master::getTopics(topics))
  {
    Q_EMIT displayMessageBox("Error getting topics", "Could not retrieve the topics names.", "",
                             QMessageBox::Icon::Critical);
    return;
  }

  for (auto topic : topics)
  {
    if (topic.datatype == "std_msgs/Bool" ||
        topic.datatype == "std_msgs/Int8" ||
        topic.datatype == "std_msgs/UInt8" ||
        topic.datatype == "std_msgs/Int16" ||
        topic.datatype == "std_msgs/UInt16" ||
        topic.datatype == "std_msgs/Int32" ||
        topic.datatype == "std_msgs/UInt32" ||
        topic.datatype == "std_msgs/Int64" ||
        topic.datatype == "std_msgs/UInt64" ||
        topic.datatype == "std_msgs/Float32" ||
        topic.datatype == "std_msgs/Float64")
      supported_topics_.push_back(topic);
  }
}

void SelectionTopics::displayMessageBoxHandler(const QString title,
    const QString text,
    const QString info,
    const QMessageBox::Icon icon)
{
  const bool old_state(isEnabled());
  setEnabled(false);
  QMessageBox msg_box;
  msg_box.setWindowTitle(title);
  msg_box.setText(text);
  msg_box.setInformativeText(info);
  msg_box.setIcon(icon);
  msg_box.setStandardButtons(QMessageBox::Ok);
  msg_box.exec();
  setEnabled(old_state);
}

void SelectionTopics::okClicked()
{
  for (auto button : topic_buttons_)
  {
    if (!button->isChecked())
      continue;

    std::shared_ptr<TopicData> topic_data =
      std::make_shared<TopicData>(button->objectName().toStdString(), button->toolTip().toStdString(), nh_);
    displayed_topics_.push_back(topic_data);
  }

  accept();
}

}
