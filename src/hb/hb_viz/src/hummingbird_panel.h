/**
 * \file hummingbird_panel.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 *
 * Rviz panel plugin for setting hummingbird setpoints
 */

#ifndef HUMMINGBIRD_PANEL_H
#define HUMMINGBIRD_PANEL_H

#include <QLineEdit>
#include <QSlider>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QTimer>
#include <QComboBox>

#include <std_msgs/Float32.h>
#include <hb_msgs/State.h>
#include <hb_msgs/ReferenceState.h>
#include <hb_msgs/Command.h>

#include <vector>
#include <string>

#include <iostream>

#ifndef Q_MOC_RUN
  #include <ros/ros.h>
  #include <rviz/panel.h>
#endif

class QLineEdit;
class QSlider;
class QSpinBox;
class QCheckBox;
class QString;

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

namespace hb_viz
{

const std::string kStateTopic ="/hb_state";
const std::string kReferenceTopic ="/hb_reference_state";
const std::string kCommandTopic ="/hb_command";

enum class LayoutType { states, reference_states, command };

class HummingbirdPanel : public rviz::Panel
{
Q_OBJECT
public:
  HummingbirdPanel(QWidget* parent);

  // Load parameters from the .rviz config file.
  // These parameters are used to initialize the panel
  // and ROS topic
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:
  /* These functions are called when the corresponding slider or
     spin box values change.
  */

  // Controls which topic is being published.
  void setTopic();


protected Q_SLOTS:

  void sliderOneCallback(int value);
  void sliderTwoCallback(int value);
  void sliderThreeCallback(int value);

  void spinboxOneCallback(double value);
  void spinboxTwoCallback(double value);
  void spinboxThreeCallback(double value);


  void sendSetpoints();

  // Used to enable or disable user  input and
  // the ROS topic
  void setEnabled(bool enabled);

  // Used to update the layout to reflect the change in selected topic
  //
  void updateLayout(const QString& layout_type);



protected:

  void init();

  void updateWidgets();

  QSlider* slider_1_;  // Used for pitch, pitch_r, and command F
  QSlider* slider_2_;  // Used for yaw, yaw_r, and command tau
  QSlider* slider_3_;  // Used for roll

  QDoubleSpinBox* spinbox_1_;  // Used for pitch, pitch_r, and command F
  QDoubleSpinBox* spinbox_2_; // Used for yaw, yaw_r, and command tau
  QDoubleSpinBox* spinbox_3_; // Used for roll

  QLabel* label_1_; // Used for pitch, pitch_r, and command F
  QLabel* label_2_; // Used for yaw, yaw_r, and command tau
  QLabel* label_3_; // Used for roll

  // labels that change when the topic is changed
  QLabel* pitch_label_;
  QLabel* yaw_label_;
  QLabel* roll_label_;

  QString topic_;

  QCheckBox* enabled_check_box_;
  QCheckBox* enabled_check_box_pwm_;
  QComboBox* layout_type_;


  // layouts that change
  QVBoxLayout* layout_;
  QGridLayout* setpoints_layout_;
  QGroupBox* setpoints_group_box_;

  ros::Publisher publisher_;

  ros::NodeHandle nh_;

  hb_msgs::State hb_msg_;
  hb_msgs::ReferenceState hb_r_msg_;
  hb_msgs::Command hb_cmd_msg_;

  // Hummingbird Parameters
  float d_;       // Length of the hb rod, m
  float km_;      // Force to PWM conversion at equilibrium


  float ratio_;

  bool enabled_;
  bool enabled_pwm_;
  bool layout_set_ = false;

  // Hummingbird states in degrees
  int pitch_setpoint_deg_ = 0;
  int yaw_setpoint_deg_ = 0;
  int roll_setpoint_deg_ = 0;

  // Hummingbird reference states in degrees
  int pitch_setpoint_deg_r_ = 0;
  int yaw_setpoint_deg_r_ = 0;

  // Commands
  double force_ = 0;
  double torque_ = 0;


  // PWM Commands
  double pwm_ = 0;
  
  // Max and min range for pitch, yaw, roll
  int pitch_max_deg_;
  int yaw_max_deg_;
  int roll_max_deg_;

  int force_max_;
  int torque_max_;

  int pwm_max_;

};

} // namespace hb_viz

#endif // HUMMINGBIRD_PANEL_H
