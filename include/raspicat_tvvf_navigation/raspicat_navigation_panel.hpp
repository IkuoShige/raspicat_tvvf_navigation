#ifndef RASPICAT_TVVF_NAVIGATION__RASPICAT_NAVIGATION_PANEL_HPP_
#define RASPICAT_TVVF_NAVIGATION__RASPICAT_NAVIGATION_PANEL_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QProgressBar>
#include <QTableWidget>
#include <QTextEdit>
#include <QFileDialog>
#include <QGroupBox>
#include <QGridLayout>

#include <std_srvs/srv/trigger.hpp>
#include "raspicat_tvvf_navigation/msg/waypoint_status.hpp"

namespace raspicat_tvvf_navigation
{

class RaspicatNavigationPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RaspicatNavigationPanel(QWidget * parent = nullptr);
  ~RaspicatNavigationPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

protected Q_SLOTS:
  void onStartButtonClick();
  void onPauseButtonClick();
  void onResumeButtonClick();
  void onSkipButtonClick();
  void onLoadWaypointsButtonClick();
  void onJumpToWaypointButtonClick();
  void onWaypointTableCellClicked(int row, int column);
  void updateLog(const QString & message, const QString & level = "INFO");

private:
  // GUI Layout setup
  void setupMainLayout();
  QGroupBox * createStatusGroup();
  QGroupBox * createControlGroup();
  QGroupBox * createWaypointGroup();
  QGroupBox * createLogGroup();

  // Helper methods
  void callService(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
    const std::string & service_name);
  void updateProgressBar(int current, int total);
  void updateWaypointTable();
  void updateButtonStates(const std::string & state);

  // Status labels
  QLabel * status_label_;
  QLabel * current_wp_label_;
  QLabel * total_wp_label_;
  QLabel * completed_label_;
  QLabel * skipped_label_;
  QLabel * distance_label_;
  QLabel * orientation_label_;
  QLabel * command_label_;

  // Progress bar
  QProgressBar * progress_bar_;

  // Control buttons
  QPushButton * start_button_;
  QPushButton * pause_button_;
  QPushButton * resume_button_;
  QPushButton * skip_button_;
  QPushButton * load_waypoints_button_;
  QPushButton * jump_to_waypoint_button_;

  // Waypoint table
  QTableWidget * waypoint_table_;
  int selected_waypoint_id_;

  // Log display
  QTextEdit * log_display_;

  // ROS2 Node
  rclcpp::Node::SharedPtr nh_;

  // Service Clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resume_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr skip_client_;

  // Subscriber
  rclcpp::Subscription<msg::WaypointStatus>::SharedPtr status_sub_;

  // Callback
  void statusCallback(const msg::WaypointStatus::SharedPtr msg);

  // Latest status message
  msg::WaypointStatus::SharedPtr latest_status_;

  // Waypoint CSV path
  std::string waypoint_csv_path_;
};

}  // namespace raspicat_tvvf_navigation

#endif  // RASPICAT_TVVF_NAVIGATION__RASPICAT_NAVIGATION_PANEL_HPP_
