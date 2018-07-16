#include "../include/rviz_graph_panel/selection_topics.hpp"

namespace rviz_graph_plugin
{

SelectionTopics::SelectionTopics(QDialog *)

{
  setWindowTitle("Topics Selection");
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
    pick_topics_dialog->addWidget(radio_button);
  }

  QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok
      | QDialogButtonBox::Cancel);
  pick_topics_dialog->addWidget(button_box);

  connect(button_box, &QDialogButtonBox::accepted, this, &SelectionTopics::okClicked);
  connect(button_box, &QDialogButtonBox::rejected, this, &QDialog::reject);
}

SelectionTopics::~SelectionTopics()
{
}

void SelectionTopics::detectTopics()
{

  ros::master::V_TopicInfo topics;
  if (!ros::master::getTopics(topics))
  {
    Q_EMIT SelectionTopics::displayMessageBox("Error getting topics", "Could not retrieve the topics names.", "",
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
        topic.datatype == "std_msgs/Float64" ||
        topic.datatype == "std_msgs/String")
      supported_topics_.push_back(topic);
  }

  if (supported_topics_.empty())
  {

    QDialog *no_topics_dialog = new QDialog(this);
    no_topics_dialog->setWindowTitle("No supported topic");
    QVBoxLayout *layout = new QVBoxLayout;
    no_topics_dialog->setLayout(layout);
    layout->addWidget(
        new QLabel("Error with topics, no supported topics found.\n"
                   "- Ok will clear the topics displayed\n- Cancel will not change the displayed topics"));

    QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok
        | QDialogButtonBox::Cancel);
    no_topics_dialog->layout()->addWidget(button_box);

    connect(button_box, &QDialogButtonBox::accepted, no_topics_dialog, &QDialog::accept);
    connect(button_box, &QDialogButtonBox::rejected, no_topics_dialog, &QDialog::reject);

    if (!no_topics_dialog->exec())
    {
      close();
      return;
    }
    else
    {
      displayed_topics_.clear();
      close();
      return;
    }
  }
}

void SelectionTopics::displayMessageBoxHandler(const QString title,
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

void SelectionTopics::okClicked()
{
  for (auto button : topic_buttons_)
  {
    if (!button->isChecked())
      continue;

    displayed_topics_.insert(std::pair<std::string, std::string>(button->objectName().toStdString(),
                                                                 button->toolTip().toStdString()));
  }
  accept();
}

}
