#include "robot_arm_rviz_panel/planning_progress_panel.hpp"

#include <algorithm>
#include <string>

#include <QMetaObject>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace robot_arm_rviz_panel
{

PlanningProgressPanel::PlanningProgressPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  status_label_ = new QLabel("Waiting for a pick and place plan", this);
  progress_bar_ = new QProgressBar(this);
  progress_bar_->setRange(0, 100);
  progress_bar_->setValue(0);
  progress_bar_->setFormat("%p%");
  layout->addWidget(status_label_);
  layout->addWidget(progress_bar_);
  setLayout(layout);
}

void PlanningProgressPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    status_label_->setText("RViz ROS node is unavailable");
    return;
  }
  rviz_node_ = abstraction->get_raw_node();
  subscription_ = rviz_node_->create_subscription<std_msgs::msg::String>(
    "/gp8_pick_place/planning_progress", 10,
    [this](const std_msgs::msg::String::ConstSharedPtr message) {
      const auto payload = message->data;
      QMetaObject::invokeMethod(
        this, [this, payload]() {
          const auto separator = payload.find('|');
          if (separator == std::string::npos) {
            status_label_->setText(QString::fromStdString(payload));
            return;
          }
          try {
            const auto percent = std::stoi(payload.substr(0, separator));
            progress_bar_->setValue(std::clamp(percent, 0, 100));
            status_label_->setText(
              QString::fromStdString(payload.substr(separator + 1)));
          } catch (const std::exception &) {
            status_label_->setText(QString::fromStdString(payload));
          }
        }, Qt::QueuedConnection);
    });
}

}  // namespace robot_arm_rviz_panel

PLUGINLIB_EXPORT_CLASS(
  robot_arm_rviz_panel::PlanningProgressPanel, rviz_common::Panel)
