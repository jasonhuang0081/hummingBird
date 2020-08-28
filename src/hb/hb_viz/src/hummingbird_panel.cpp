/**
 * \file hummingbird_panel.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 * \modified Mark Petersen <pet09034@byu.edu>
 */



#include "hummingbird_panel.h"

namespace hb_viz
{




HummingbirdPanel::HummingbirdPanel(QWidget* parent = 0) : rviz::Panel(parent),
  pitch_setpoint_deg_(0.0f),
  yaw_setpoint_deg_(0.0f),
  roll_setpoint_deg_(0.0f),
  pitch_setpoint_deg_r_(0.0f),
  yaw_setpoint_deg_r_(0.0f),
  pwm_(0.0f),
  pitch_max_deg_(45),
  yaw_max_deg_(45),
  roll_max_deg_(45),
  ratio_(1000),
  force_max_(20),
  torque_max_(20),
  pwm_max_(1)
{


  // Get parameters from the ros network if they exist
  if ( nh_.hasParam("/hummingbird/d"))
    nh_.getParam("/hummingbird/d", d_);
  else
    d_ = 1;

  if ( nh_.hasParam("/hummingbird/km"))
    nh_.getParam("/hummingbird/km", km_);
  else
    km_ = 1;

}

//--------------------------------------------------------------------------------------------------

void HummingbirdPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  QString topic;

  // Grab the topic parameter. This parameter determines the layout
  // of the panel.
  if (config.mapGetString("Topic", &topic))
  {
    // Topic isn't valid so default to /hummingbird
    if(topic.toStdString() != kStateTopic && topic.toStdString() != kReferenceTopic && topic.toStdString() != kCommandTopic)
    {
      topic_ =  kStateTopic.c_str();
    }
    else
    {
      topic_ = topic;
    }
  }


  if (config.mapGetBool("Enabled", &enabled_))
  {
    enabled_check_box_ = new QCheckBox;
    enabled_check_box_->setChecked(enabled_);
  }
  else
  {
    enabled_ = false;
    enabled_check_box_ = new QCheckBox;
    enabled_check_box_->setChecked(enabled_);
  }


  if (config.mapGetBool("PWM Cntrl Enabled", &enabled_pwm_))
  {
    enabled_check_box_pwm_ = new QCheckBox;
    enabled_check_box_pwm_->setChecked(enabled_pwm_);
  }
  else
  {
    enabled_pwm_ = false;
    enabled_check_box_pwm_ = new QCheckBox;
    enabled_check_box_pwm_->setChecked(enabled_pwm_);
  }


  init();
  updateWidgets();
}

//--------------------------------------------------------------------------------------------------
void HummingbirdPanel::updateWidgets()
{

  // Turn off signals during update so that callback functions arn't called
  // when the values change.
  slider_1_->blockSignals(true);
  slider_2_->blockSignals(true);
  slider_3_->blockSignals(true);
  spinbox_1_->blockSignals(true);
  spinbox_2_->blockSignals(true);
  spinbox_3_->blockSignals(true);

  if (topic_.toStdString() == kStateTopic)
  {
    slider_1_->setRange(-pitch_max_deg_, pitch_max_deg_);
    slider_1_->setTickInterval(15);
    slider_1_->setValue(pitch_setpoint_deg_);
    spinbox_1_->setRange(-pitch_max_deg_, pitch_max_deg_);
    spinbox_1_->setValue(static_cast<double>(pitch_setpoint_deg_));

    slider_2_->setRange(-yaw_max_deg_, yaw_max_deg_);
    slider_2_->setTickInterval(15);
    slider_2_->setValue(yaw_setpoint_deg_);
    spinbox_2_->setRange(-yaw_max_deg_, yaw_max_deg_);
    spinbox_2_->setValue(static_cast<double>(yaw_setpoint_deg_));

    slider_3_->setRange(-roll_max_deg_, roll_max_deg_);
    slider_3_->setTickInterval(15);
    spinbox_3_->setRange(-roll_max_deg_, roll_max_deg_);
    spinbox_3_->setSingleStep(1);
    spinbox_3_->setDecimals(1);        

    slider_3_->setVisible(true);
    spinbox_3_->setVisible(true);
    label_3_->setVisible(true);

    label_1_->setText("Pitch");
    label_2_->setText("Yaw");
    label_3_->setText("Roll");

    setpoints_group_box_->setTitle("Hummingbird States");


    ros::Duration(0.5).sleep(); // sleep for half a second

  }
  else if (topic_.toStdString() == kReferenceTopic)
  {
    slider_1_->setRange(-pitch_max_deg_, pitch_max_deg_);
    slider_1_->setTickInterval(15);
    slider_1_->setValue(pitch_setpoint_deg_r_);
    spinbox_1_->setRange(-pitch_max_deg_, pitch_max_deg_);
    spinbox_1_->setValue(static_cast<double>(pitch_setpoint_deg_r_));

    slider_2_->setRange(-yaw_max_deg_, yaw_max_deg_);
    slider_2_->setTickInterval(15);
    slider_2_->setValue(yaw_setpoint_deg_r_);
    spinbox_2_->setRange(-yaw_max_deg_, yaw_max_deg_);
    spinbox_2_->setValue(static_cast<double>(yaw_setpoint_deg_r_));

    slider_3_->setVisible(false);
    spinbox_3_->setVisible(false);
    label_3_->setVisible(false);

    label_1_->setText("Pitch_r");
    label_2_->setText("Yaw_r");

    setpoints_group_box_->setTitle("Hummingbird Reference States");

  }
  else
  {
    slider_1_->setRange(-force_max_*ratio_, force_max_*ratio_);
    slider_1_->setTickInterval(15*ratio_);
    slider_1_->setValue(static_cast<int>(force_*ratio_));
    spinbox_1_->setRange(-force_max_, force_max_);
    spinbox_1_->setValue(force_);

    slider_2_->setRange(-torque_max_*ratio_, torque_max_*ratio_);
    slider_2_->setTickInterval(15*ratio_);
    slider_2_->setValue(static_cast<int>(torque_*ratio_));
    spinbox_2_->setRange(-torque_max_, torque_max_);
    spinbox_2_->setValue(torque_);

    slider_3_->setRange(0, pwm_max_*ratio_);
    slider_3_->setTickInterval(15*ratio_);
    slider_3_->setValue(static_cast<int>(pwm_*ratio_));
    spinbox_3_->setSingleStep(0.001);
    spinbox_3_->setDecimals(3);    
    spinbox_3_->setRange(0, pwm_max_);
    spinbox_3_->setValue(pwm_);

    slider_3_->setVisible(true);
    spinbox_3_->setVisible(true);
    label_3_->setVisible(true);
    
    label_1_->setText("Force");
    label_2_->setText("Torque");
    label_3_->setText("PWM");
    
    setpoints_group_box_->setTitle("Hummingbird Commands");

  }

  // Turn on signals after update.
  slider_1_->blockSignals(false);
  slider_2_->blockSignals(false);
  slider_3_->blockSignals(false);
  spinbox_1_->blockSignals(false);
  spinbox_2_->blockSignals(false);
  spinbox_3_->blockSignals(false);


}


//--------------------------------------------------------------------------------------------------

void HummingbirdPanel::init()
{
  //===========================================================================
  // controls
  //===========================================================================

  // setpoints
  slider_1_ = new QSlider(Qt::Horizontal);
  slider_1_->setTickPosition(QSlider::TicksBelow);

  spinbox_1_ = new QDoubleSpinBox;


  slider_2_ = new QSlider(Qt::Horizontal);
  slider_2_->setTickPosition(QSlider::TicksBelow);

  spinbox_2_ = new QDoubleSpinBox;

  slider_3_ = new QSlider(Qt::Horizontal);
  slider_3_->setTickPosition(QSlider::TicksBelow);

  spinbox_3_ = new QDoubleSpinBox;

  layout_type_ = new QComboBox;
  layout_type_->addItem(kStateTopic.c_str());
  layout_type_->addItem(kReferenceTopic.c_str());
  layout_type_->addItem(kCommandTopic.c_str());
  layout_type_->setInsertPolicy(QComboBox::NoInsert);
  layout_type_->setCurrentText(topic_);

   // timer
  QTimer* setpoint_timer = new QTimer(this);

  //===========================================================================
  // labels
  //===========================================================================
  label_1_ = new QLabel();
  label_2_ = new QLabel();
  label_3_= new QLabel();




  //===========================================================================
  // layout states
  //===========================================================================

  // Commands
  setpoints_layout_ = new QGridLayout;
  setpoints_layout_->addWidget(label_1_, 0, 0);
  setpoints_layout_->addWidget(slider_1_, 0, 1);
  setpoints_layout_->addWidget(spinbox_1_, 0, 2);
  setpoints_layout_->addWidget(label_2_, 1, 0);
  setpoints_layout_->addWidget(slider_2_, 1, 1);
  setpoints_layout_->addWidget(spinbox_2_, 1, 2);
  setpoints_layout_->addWidget(label_3_, 2, 0);
  setpoints_layout_->addWidget(slider_3_, 2, 1);
  setpoints_layout_->addWidget(spinbox_3_, 2, 2);




  setpoints_group_box_ = new QGroupBox();
  setpoints_group_box_->setLayout(setpoints_layout_);

  // Topic
  QGridLayout* topics_layout = new QGridLayout;
  topics_layout->addWidget(new QLabel("Layout Type"), 0, 0);
  topics_layout->addWidget(layout_type_, 0, 1);

  QGroupBox* topics_group_box = new QGroupBox(tr("Topics"));
  topics_group_box->setLayout(topics_layout);


  QHBoxLayout* enabled_layout = new QHBoxLayout;
  QSizePolicy enabled_check_box_policy;
  enabled_check_box_policy.setHorizontalStretch(0);
  enabled_check_box_->setSizePolicy(enabled_check_box_policy);
  enabled_layout->addWidget(enabled_check_box_);
  enabled_layout->addWidget(new QLabel("Enabled"));

  QHBoxLayout* enabled_layout_pwm = new QHBoxLayout;
  QSizePolicy enabled_check_box_pwm_policy;
  enabled_check_box_pwm_policy.setHorizontalStretch(0);
  enabled_check_box_pwm_->setSizePolicy(enabled_check_box_pwm_policy);
  enabled_layout_pwm->addWidget(enabled_check_box_pwm_);
  enabled_layout_pwm->addWidget(new QLabel("PWM Cntrl Enabled"));
  

  layout_ = new QVBoxLayout;
  layout_->addWidget(setpoints_group_box_);
  layout_->addWidget(topics_group_box);
  layout_->addLayout(enabled_layout);
  layout_->addLayout(enabled_layout_pwm);




  //===========================================================================
  // connections
  //===========================================================================

  // link setpoint sliders and spinboxes
  connect(slider_1_, SIGNAL(valueChanged(int)), this, SLOT(sliderOneCallback(int)));
  connect(spinbox_1_, SIGNAL(valueChanged(double)), this, SLOT(spinboxOneCallback(double)));
  connect(slider_2_, SIGNAL(valueChanged(int)), this, SLOT(sliderTwoCallback(int)));
  connect(spinbox_2_, SIGNAL(valueChanged(double)), this, SLOT(spinboxTwoCallback(double)));
  connect(slider_3_, SIGNAL(valueChanged(int)), this, SLOT(sliderThreeCallback(int)));
  connect(spinbox_3_, SIGNAL(valueChanged(double)), this, SLOT(spinboxThreeCallback(double)));

  connect(layout_type_, SIGNAL(activated(QString)), this, SLOT(updateLayout(QString)));

  connect(enabled_check_box_, SIGNAL(toggled(bool)), this, SLOT(setEnabled(bool)));
  //connect(enabled_check_box_pwm_, SIGNAL(toggled(bool)), this, SLOT(setEnabled(bool)));

  connect(setpoint_timer, SIGNAL(timeout()), this, SLOT(sendSetpoints()));

  //===========================================================================
  // final setup
  //===========================================================================

  setTopic();
  // initialize publishers
  setEnabled(enabled_check_box_->isChecked());

  // start the timer
  setpoint_timer->start(50);

  setLayout(layout_);


}

//--------------------------------------------------------------------------------------------------

void HummingbirdPanel::sliderOneCallback(int value)
{

  double v;
  if (topic_.toStdString() == kStateTopic) {
    pitch_setpoint_deg_ = value;
    v = static_cast<double>(value);
  }
  else if (topic_.toStdString() == kReferenceTopic) {
    pitch_setpoint_deg_r_ = value;
    v = static_cast<double>(value);
  }
  else {
    v = static_cast<double>(value/ratio_);
    force_ = v;
  }

  spinbox_1_->setValue(v);
}

//--------------------------------------------------------------------------------------------------

void HummingbirdPanel::sliderTwoCallback(int value)
{

  double v;
  if (topic_.toStdString() == kStateTopic) {
    yaw_setpoint_deg_ = value;
    v = static_cast<double>(value);
  }
  else if (topic_.toStdString() == kReferenceTopic) {
    yaw_setpoint_deg_r_ = value;
    v = static_cast<double>(value);
  }
  else {
    v = static_cast<double>(value/ratio_);
    torque_ = v;
  }

  spinbox_2_->setValue(v);
}

//--------------------------------------------------------------------------------------------------

void HummingbirdPanel::sliderThreeCallback(int value)
{
  double v;
  if (topic_.toStdString() == kStateTopic)
  {
    v = static_cast<double>(value);
    roll_setpoint_deg_ = value;
  }
  else if (topic_.toStdString() == kCommandTopic) {
    v = static_cast<double>(value/ratio_);
    pwm_ = v;
  }

  spinbox_3_->setValue(v);
  
}

//--------------------------------------------------------------------------------------------------

void HummingbirdPanel::spinboxOneCallback(double value)
{

  int v;
  if (topic_.toStdString() == kStateTopic) {
    v = static_cast<int>(value);
    pitch_setpoint_deg_ = v;
  }
  else if (topic_.toStdString() == kReferenceTopic) {
    v = static_cast<int>(value);
    pitch_setpoint_deg_r_ = v;
  }
  else {
    v = static_cast<int>(value*ratio_);
    force_ = value;
  }
  slider_1_->setValue(v);
}

//--------------------------------------------------------------------------------------------------

void HummingbirdPanel::spinboxTwoCallback(double value)
{

  int v;
  if (topic_.toStdString() == kStateTopic) {
    v = static_cast<int>(value);
    yaw_setpoint_deg_ = v;
  }
  else if (topic_.toStdString() == kReferenceTopic) {
    v = static_cast<int>(value);
    yaw_setpoint_deg_r_ = v;
  }
  else {
    v = static_cast<int>(value*ratio_);
    torque_ = value;
  }
  slider_2_->setValue(v);
}

//--------------------------------------------------------------------------------------------------

void HummingbirdPanel::spinboxThreeCallback(double value)
{

  int v;
  if (topic_.toStdString() == kStateTopic)
  {
    v = static_cast<int>(value);
    roll_setpoint_deg_ = v;
  }
  else if (topic_.toStdString() == kCommandTopic)
  {
    v = static_cast<double>(value*ratio_);
    pwm_ = value;
  }

  slider_3_->setValue(v);
  
}

//--------------------------------------------------------------------------------------------------

void HummingbirdPanel::save(rviz::Config config) const
{

  config.mapSetValue("Enabled", enabled_);
  config.mapSetValue("Topic", topic_);
  rviz::Panel::save(config);

}

//--------------------------------------------------------------------------------------------------

void HummingbirdPanel::updateLayout(const QString& layout_type)
{

  topic_ = layout_type;
  updateWidgets();
  setTopic();

}

//--------------------------------------------------------------------------------------------------

void HummingbirdPanel::setTopic()
{

  // Switch the topic to the correct one
  // publisher_.getTopic() != topic_.toStdString() &&
  if ( enabled_)
  {

    if (topic_.toStdString() == kStateTopic)
      publisher_ = nh_.advertise<hb_msgs::State>(topic_.toStdString(), 1);
    else if (topic_.toStdString() == kReferenceTopic)
      publisher_ = nh_.advertise<hb_msgs::ReferenceState>(topic_.toStdString(), 1);
    else
      publisher_ = nh_.advertise<hb_msgs::Command>(topic_.toStdString(), 1);

  }

  // Shut down the publisher
  if (!enabled_)
  {
    publisher_.shutdown();
  }

}

//--------------------------------------------------------------------------------------------------

void HummingbirdPanel::setEnabled(bool enabled)
{
  if (enabled != enabled_)
  {
    Q_EMIT configChanged();

    enabled_ = enabled;
    setTopic();
  }

  slider_1_->setEnabled(enabled_);
  spinbox_1_->setEnabled(slider_1_->isEnabled());
  slider_2_->setEnabled(enabled_);
  spinbox_2_->setEnabled(slider_2_->isEnabled());
  slider_3_->setEnabled(enabled_);
  spinbox_3_->setEnabled(slider_3_->isEnabled());

}

//--------------------------------------------------------------------------------------------------

void HummingbirdPanel::sendSetpoints()
{
  if (ros::ok() && enabled_)
  {

    if (topic_.toStdString() == kStateTopic)
    {
      hb_msg_.roll = roll_setpoint_deg_*(M_PI / 180);
      hb_msg_.pitch = pitch_setpoint_deg_*(M_PI / 180);
      hb_msg_.yaw = yaw_setpoint_deg_*(M_PI / 180);
      publisher_.publish(hb_msg_);
    }
    else if (topic_.toStdString() == kReferenceTopic)
    {
      hb_r_msg_.pitch = pitch_setpoint_deg_r_*(M_PI / 180);
      hb_r_msg_.yaw = yaw_setpoint_deg_r_*(M_PI / 180);
      publisher_.publish(hb_r_msg_);
    }
    else
    {
      if (enabled_check_box_pwm_->isChecked())
      {
	hb_cmd_msg_.left_motor = pwm_;
	hb_cmd_msg_.right_motor = pwm_;
	publisher_.publish(hb_cmd_msg_);	
      }
      else
      {
	hb_cmd_msg_.left_motor = (force_ + torque_/d_)/km_/2;
	hb_cmd_msg_.right_motor = (force_ -torque_/d_)/km_/2;
	publisher_.publish(hb_cmd_msg_);
      }

    }
  }
}

} // namespace hb_viz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hb_viz::HummingbirdPanel, rviz::Panel)
