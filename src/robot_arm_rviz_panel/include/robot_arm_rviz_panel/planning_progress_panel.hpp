#ifndef ROBOT_ARM_RVIZ_PANEL__PLANNING_PROGRESS_PANEL_HPP_
#define ROBOT_ARM_RVIZ_PANEL__PLANNING_PROGRESS_PANEL_HPP_

#include <memory>

#include <QLabel>
#include <QProgressBar>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

namespace robot_arm_rviz_panel
{

class PlanningProgressPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit PlanningProgressPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  void update_progress(const std_msgs::msg::String::ConstSharedPtr message);

  QLabel * status_label_;
  QProgressBar * progress_bar_;
  rclcpp::Node::SharedPtr rviz_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

}  // namespace robot_arm_rviz_panel

#endif  // ROBOT_ARM_RVIZ_PANEL__PLANNING_PROGRESS_PANEL_HPP_
