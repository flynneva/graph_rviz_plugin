#include <rviz_graph_panel/configure.hpp>

namespace rviz_graph_plugin
{

Configure::Configure(std::deque<std::shared_ptr<TopicData>> displayed_topics, QDialog *):
  displayed_topics_(displayed_topics)
{
  setWindowTitle("Configure topics");
  QVBoxLayout *configure_layout = new QVBoxLayout;
  QHBoxLayout *form_layout = new QHBoxLayout;
  setLayout(configure_layout);
  QStringList color_list = {"Blue", "Red", "Black", "Cyan", "Yellow", "Gray"};

  for (unsigned i = 0; i < displayed_topics_.size(); i++)
  {
    QFormLayout *topic_form = new QFormLayout;
    QLabel *topic_name_label = new QLabel;
    topic_name_label->setText(QString::fromStdString(displayed_topics_.at(i)->topic_name_));
    topic_form->addRow("Name", topic_name_label);

    QComboBox *color_selection_combobox = new QComboBox;
    topic_combobox_.push_back(color_selection_combobox);
    color_selection_combobox->setObjectName(QString::fromStdString(displayed_topics_.at(i)->topic_name_));
    color_selection_combobox->addItems(color_list);

    if (displayed_topics_.at(i)->color_ == QColor(0, 0, 255)) //blue
      color_selection_combobox->setCurrentIndex(0);
    else if (displayed_topics_.at(i)->color_ == QColor(255, 0, 0)) //red
      color_selection_combobox->setCurrentIndex(1);
    else if (displayed_topics_.at(i)->color_ == QColor(0, 0, 0)) //black
      color_selection_combobox->setCurrentIndex(2);
    else if (displayed_topics_.at(i)->color_ == QColor(0, 255, 255)) //cyan
      color_selection_combobox->setCurrentIndex(3);
    else if (displayed_topics_.at(i)->color_ == QColor(255, 255, 0)) //yellow
      color_selection_combobox->setCurrentIndex(4);
    else//gray
      color_selection_combobox->setCurrentIndex(5);

    topic_form->addRow("Color", color_selection_combobox);

    QSpinBox *thickness_spin_box = new QSpinBox;
    topic_spinbox_.push_back(thickness_spin_box);
    thickness_spin_box->setObjectName(QString::fromStdString(displayed_topics_.at(i)->topic_name_));
    thickness_spin_box->setRange(1, 10);
    thickness_spin_box->setValue(displayed_topics_.at(i)->thickness_);
    topic_form->addRow("Thickness", thickness_spin_box);

    QCheckBox *topic_checkbox = new QCheckBox;
    topic_buttons_.push_back(topic_checkbox);
    topic_checkbox->setObjectName(QString::fromStdString(displayed_topics_.at(i)->topic_name_));
    topic_checkbox->setChecked(true);

    if (displayed_topics_.at(i)->displayed_ == false)
      topic_checkbox->setChecked(false);

    topic_form->addRow("Display", topic_checkbox);
    form_layout->addLayout(topic_form);
  }

  configure_layout->addLayout(form_layout);
  QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok
      | QDialogButtonBox::Cancel);
  configure_layout->addWidget(button_box);

  connect(button_box, &QDialogButtonBox::accepted, this, &Configure::okClicked);
  connect(button_box, &QDialogButtonBox::rejected, this, &QDialog::reject);
}

Configure::~Configure()
{
}

void Configure::okClicked()
{
  for (auto button : topic_buttons_)
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

  for (auto spinbox : topic_spinbox_)
  {
    for (unsigned i = 0; i < displayed_topics_.size(); i++)
    {
      if ((displayed_topics_.at(i)->topic_name_) == spinbox->objectName().toStdString())
        displayed_topics_.at(i)->thickness_ = spinbox->value();
    }
  }

  for (auto combobox : topic_combobox_)
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
  close();
  return;
}

}
