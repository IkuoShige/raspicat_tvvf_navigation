#include "raspicat_tvvf_navigation/raspicat_navigation_panel.hpp"

#include <rviz_common/display_context.hpp>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QHeaderView>
#include <QScrollBar>
#include <QDateTime>

namespace raspicat_tvvf_navigation
{

RaspicatNavigationPanel::RaspicatNavigationPanel(QWidget * parent)
: rviz_common::Panel(parent),
  selected_waypoint_id_(-1),
  waypoint_csv_path_("")
{
  setupMainLayout();
}

RaspicatNavigationPanel::~RaspicatNavigationPanel()
{
}

void RaspicatNavigationPanel::onInitialize()
{
  // Get the RViz node
  auto rviz_node = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!rviz_node) {
    updateLog("Failed to get RViz node", "ERROR");
    return;
  }

  // Create our own node for service clients and subscribers
  nh_ = rviz_node->get_raw_node();

  // Initialize service clients
  start_client_ = nh_->create_client<std_srvs::srv::Trigger>("/start_waypoint_navigation");
  pause_client_ = nh_->create_client<std_srvs::srv::Trigger>("/pause_waypoint_navigation");
  resume_client_ = nh_->create_client<std_srvs::srv::Trigger>("/resume_waypoint_navigation");
  skip_client_ = nh_->create_client<std_srvs::srv::Trigger>("/skip_current_waypoint");

  // Initialize subscriber
  status_sub_ = nh_->create_subscription<msg::WaypointStatus>(
    "/waypoint_status",
    10,
    std::bind(&RaspicatNavigationPanel::statusCallback, this, std::placeholders::_1));

  updateLog("RaspicatNavigationPanel initialized", "INFO");
}

void RaspicatNavigationPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString waypoint_path;
  if (config.mapGetString("waypoint_csv_path", &waypoint_path)) {
    waypoint_csv_path_ = waypoint_path.toStdString();
  }
}

void RaspicatNavigationPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("waypoint_csv_path", QString::fromStdString(waypoint_csv_path_));
}

void RaspicatNavigationPanel::setupMainLayout()
{
  QVBoxLayout * main_layout = new QVBoxLayout();
  main_layout->setSpacing(10);
  main_layout->setContentsMargins(10, 10, 10, 10);

  // Create and add all groups
  QGroupBox * status_group = createStatusGroup();
  QGroupBox * control_group = createControlGroup();
  QGroupBox * waypoint_group = createWaypointGroup();
  QGroupBox * log_group = createLogGroup();

  main_layout->addWidget(status_group);
  main_layout->addWidget(control_group);
  main_layout->addWidget(waypoint_group);
  main_layout->addWidget(log_group);
  main_layout->addStretch();

  setLayout(main_layout);
}

QGroupBox * RaspicatNavigationPanel::createStatusGroup()
{
  QGroupBox * group = new QGroupBox("Navigation Status");
  QGridLayout * layout = new QGridLayout();

  // Status label
  layout->addWidget(new QLabel("State:"), 0, 0);
  status_label_ = new QLabel("IDLE");
  status_label_->setStyleSheet("QLabel { font-weight: bold; color: blue; }");
  layout->addWidget(status_label_, 0, 1);

  // Current waypoint
  layout->addWidget(new QLabel("Current WP:"), 1, 0);
  current_wp_label_ = new QLabel("0");
  layout->addWidget(current_wp_label_, 1, 1);

  // Total waypoints
  layout->addWidget(new QLabel("Total WPs:"), 1, 2);
  total_wp_label_ = new QLabel("0");
  layout->addWidget(total_wp_label_, 1, 3);

  // Completed
  layout->addWidget(new QLabel("Completed:"), 2, 0);
  completed_label_ = new QLabel("0");
  layout->addWidget(completed_label_, 2, 1);

  // Skipped
  layout->addWidget(new QLabel("Skipped:"), 2, 2);
  skipped_label_ = new QLabel("0");
  layout->addWidget(skipped_label_, 2, 3);

  // Distance
  layout->addWidget(new QLabel("Distance:"), 3, 0);
  distance_label_ = new QLabel("0.00 m");
  layout->addWidget(distance_label_, 3, 1);

  // Orientation
  layout->addWidget(new QLabel("Orient Diff:"), 3, 2);
  orientation_label_ = new QLabel("0.00 rad");
  layout->addWidget(orientation_label_, 3, 3);

  // Command
  layout->addWidget(new QLabel("Command:"), 4, 0);
  command_label_ = new QLabel("-");
  command_label_->setWordWrap(true);
  layout->addWidget(command_label_, 4, 1, 1, 3);

  // Progress bar
  progress_bar_ = new QProgressBar();
  progress_bar_->setRange(0, 100);
  progress_bar_->setValue(0);
  progress_bar_->setTextVisible(true);
  layout->addWidget(progress_bar_, 5, 0, 1, 4);

  group->setLayout(layout);
  return group;
}

QGroupBox * RaspicatNavigationPanel::createControlGroup()
{
  QGroupBox * group = new QGroupBox("Navigation Control");
  QGridLayout * layout = new QGridLayout();

  // First row: Start, Pause, Resume
  start_button_ = new QPushButton("Start");
  start_button_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; min-height: 30px; }");
  connect(start_button_, SIGNAL(clicked()), this, SLOT(onStartButtonClick()));
  layout->addWidget(start_button_, 0, 0);

  pause_button_ = new QPushButton("Pause");
  pause_button_->setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; min-height: 30px; }");
  connect(pause_button_, SIGNAL(clicked()), this, SLOT(onPauseButtonClick()));
  layout->addWidget(pause_button_, 0, 1);

  resume_button_ = new QPushButton("Resume");
  resume_button_->setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; min-height: 30px; }");
  connect(resume_button_, SIGNAL(clicked()), this, SLOT(onResumeButtonClick()));
  layout->addWidget(resume_button_, 0, 2);

  // Second row: Skip, Load Waypoints
  skip_button_ = new QPushButton("Skip Current");
  skip_button_->setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; min-height: 30px; }");
  connect(skip_button_, SIGNAL(clicked()), this, SLOT(onSkipButtonClick()));
  layout->addWidget(skip_button_, 1, 0);

  load_waypoints_button_ = new QPushButton("Load Waypoints CSV");
  load_waypoints_button_->setStyleSheet("QPushButton { min-height: 30px; }");
  connect(load_waypoints_button_, SIGNAL(clicked()), this, SLOT(onLoadWaypointsButtonClick()));
  layout->addWidget(load_waypoints_button_, 1, 1, 1, 2);

  group->setLayout(layout);
  return group;
}

QGroupBox * RaspicatNavigationPanel::createWaypointGroup()
{
  QGroupBox * group = new QGroupBox("Waypoint List");
  QVBoxLayout * layout = new QVBoxLayout();

  // Waypoint table
  waypoint_table_ = new QTableWidget();
  waypoint_table_->setColumnCount(5);
  waypoint_table_->setHorizontalHeaderLabels(
    QStringList() << "ID" << "X" << "Y" << "Yaw" << "Command");
  waypoint_table_->horizontalHeader()->setStretchLastSection(true);
  waypoint_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  waypoint_table_->setSelectionMode(QAbstractItemView::SingleSelection);
  waypoint_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  waypoint_table_->setMaximumHeight(150);
  connect(waypoint_table_, SIGNAL(cellClicked(int, int)),
          this, SLOT(onWaypointTableCellClicked(int, int)));

  layout->addWidget(waypoint_table_);

  // Jump button
  jump_to_waypoint_button_ = new QPushButton("Jump to Selected Waypoint");
  jump_to_waypoint_button_->setEnabled(false);
  connect(jump_to_waypoint_button_, SIGNAL(clicked()), this, SLOT(onJumpToWaypointButtonClick()));
  layout->addWidget(jump_to_waypoint_button_);

  group->setLayout(layout);
  return group;
}

QGroupBox * RaspicatNavigationPanel::createLogGroup()
{
  QGroupBox * group = new QGroupBox("Log");
  QVBoxLayout * layout = new QVBoxLayout();

  log_display_ = new QTextEdit();
  log_display_->setReadOnly(true);
  log_display_->setMaximumHeight(100);
  log_display_->setStyleSheet("QTextEdit { font-family: monospace; font-size: 9pt; }");

  layout->addWidget(log_display_);

  group->setLayout(layout);
  return group;
}

void RaspicatNavigationPanel::onStartButtonClick()
{
  callService(start_client_, "start_waypoint_navigation");
}

void RaspicatNavigationPanel::onPauseButtonClick()
{
  callService(pause_client_, "pause_waypoint_navigation");
}

void RaspicatNavigationPanel::onResumeButtonClick()
{
  callService(resume_client_, "resume_waypoint_navigation");
}

void RaspicatNavigationPanel::onSkipButtonClick()
{
  callService(skip_client_, "skip_current_waypoint");
}

void RaspicatNavigationPanel::onLoadWaypointsButtonClick()
{
  QString file_path = QFileDialog::getOpenFileName(
    this,
    tr("Open Waypoints CSV"),
    QString::fromStdString(waypoint_csv_path_),
    tr("CSV Files (*.csv);;All Files (*)"));

  if (!file_path.isEmpty()) {
    waypoint_csv_path_ = file_path.toStdString();

    // Note: The waypoint_follower_node reads the waypoint_csv_path parameter on startup.
    // For dynamic parameter update, you would need to implement parameter callback in the node.
    // For now, we just save the path and inform the user.

    updateLog(QString("Waypoint CSV path selected: %1").arg(file_path), "INFO");
    updateLog("Note: Restart the waypoint_follower_node with this CSV path,", "INFO");
    updateLog("or implement dynamic parameter update in the node.", "INFO");
  }
}

void RaspicatNavigationPanel::onJumpToWaypointButtonClick()
{
  if (selected_waypoint_id_ < 0) {
    updateLog("No waypoint selected", "WARN");
    return;
  }

  // This feature requires implementing a new service in waypoint_follower_node
  // For now, just log the intention
  updateLog(QString("Jump to waypoint %1 requested (feature not yet implemented)").arg(selected_waypoint_id_), "WARN");
}

void RaspicatNavigationPanel::onWaypointTableCellClicked(int row, int column)
{
  (void)column;  // Unused

  if (row >= 0 && row < waypoint_table_->rowCount()) {
    selected_waypoint_id_ = waypoint_table_->item(row, 0)->text().toInt();
    jump_to_waypoint_button_->setEnabled(true);
    updateLog(QString("Selected waypoint %1").arg(selected_waypoint_id_), "INFO");
  }
}

void RaspicatNavigationPanel::callService(
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
  const std::string & service_name)
{
  if (!client) {
    updateLog(QString("Service client for %1 is not initialized").arg(QString::fromStdString(service_name)), "ERROR");
    return;
  }

  if (!client->wait_for_service(std::chrono::seconds(1))) {
    updateLog(QString("Service %1 not available").arg(QString::fromStdString(service_name)), "ERROR");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  updateLog(QString("Calling service: %1").arg(QString::fromStdString(service_name)), "INFO");

  // Use a lambda to handle the response asynchronously
  auto response_callback =
    [this, service_name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      try {
        auto response = future.get();
        QString log_level = response->success ? "INFO" : "ERROR";
        QString message = QString("Service %1: %2")
          .arg(QString::fromStdString(service_name))
          .arg(QString::fromStdString(response->message));

        QMetaObject::invokeMethod(
          this, "updateLog",
          Qt::QueuedConnection,
          Q_ARG(QString, message),
          Q_ARG(QString, log_level));
      } catch (const std::exception & e) {
        QString error_msg = QString("Exception calling %1: %2")
          .arg(QString::fromStdString(service_name))
          .arg(e.what());

        QMetaObject::invokeMethod(
          this, "updateLog",
          Qt::QueuedConnection,
          Q_ARG(QString, error_msg),
          Q_ARG(QString, "ERROR"));
      }
    };

  client->async_send_request(request, response_callback);
}

void RaspicatNavigationPanel::statusCallback(const msg::WaypointStatus::SharedPtr msg)
{
  latest_status_ = msg;

  // Update status label with color coding
  QString state = QString::fromStdString(msg->state);
  QString state_color;
  if (msg->state == "IDLE") {
    state_color = "blue";
  } else if (msg->state == "NAVIGATING") {
    state_color = "green";
  } else if (msg->state == "WAITING") {
    state_color = "orange";
  } else if (msg->state == "COMPLETED") {
    state_color = "darkgreen";
  } else if (msg->state == "ERROR") {
    state_color = "red";
  } else {
    state_color = "black";
  }

  QMetaObject::invokeMethod(
    status_label_, "setText",
    Qt::QueuedConnection,
    Q_ARG(QString, state));

  QMetaObject::invokeMethod(
    status_label_, "setStyleSheet",
    Qt::QueuedConnection,
    Q_ARG(QString, QString("QLabel { font-weight: bold; color: %1; }").arg(state_color)));

  // Update waypoint info
  QMetaObject::invokeMethod(
    current_wp_label_, "setText",
    Qt::QueuedConnection,
    Q_ARG(QString, QString::number(msg->current_waypoint_id)));

  QMetaObject::invokeMethod(
    total_wp_label_, "setText",
    Qt::QueuedConnection,
    Q_ARG(QString, QString::number(msg->total_waypoints)));

  QMetaObject::invokeMethod(
    completed_label_, "setText",
    Qt::QueuedConnection,
    Q_ARG(QString, QString::number(msg->completed_waypoints)));

  QMetaObject::invokeMethod(
    skipped_label_, "setText",
    Qt::QueuedConnection,
    Q_ARG(QString, QString::number(msg->skipped_waypoints)));

  // Update distance and orientation
  QMetaObject::invokeMethod(
    distance_label_, "setText",
    Qt::QueuedConnection,
    Q_ARG(QString, QString("%1 m").arg(msg->distance_to_goal, 0, 'f', 2)));

  QMetaObject::invokeMethod(
    orientation_label_, "setText",
    Qt::QueuedConnection,
    Q_ARG(QString, QString("%1 rad").arg(msg->orientation_diff, 0, 'f', 3)));

  // Update command
  QMetaObject::invokeMethod(
    command_label_, "setText",
    Qt::QueuedConnection,
    Q_ARG(QString, QString::fromStdString(msg->current_command)));

  // Update progress bar
  updateProgressBar(msg->completed_waypoints, msg->total_waypoints);

  // Update button states based on navigation state
  updateButtonStates(msg->state);
}

void RaspicatNavigationPanel::updateLog(const QString & message, const QString & level)
{
  QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
  QString color;

  if (level == "ERROR") {
    color = "red";
  } else if (level == "WARN") {
    color = "orange";
  } else if (level == "INFO") {
    color = "black";
  } else {
    color = "blue";
  }

  QString formatted_message = QString("<span style='color: gray;'>[%1]</span> "
                                     "<span style='color: %2; font-weight: bold;'>[%3]</span> %4")
    .arg(timestamp)
    .arg(color)
    .arg(level)
    .arg(message);

  QMetaObject::invokeMethod(
    log_display_, "append",
    Qt::QueuedConnection,
    Q_ARG(QString, formatted_message));

  // Auto-scroll to bottom
  QMetaObject::invokeMethod(
    log_display_->verticalScrollBar(), "setValue",
    Qt::QueuedConnection,
    Q_ARG(int, log_display_->verticalScrollBar()->maximum()));
}

void RaspicatNavigationPanel::updateProgressBar(int current, int total)
{
  if (total > 0) {
    int percentage = (current * 100) / total;
    QMetaObject::invokeMethod(
      progress_bar_, "setValue",
      Qt::QueuedConnection,
      Q_ARG(int, percentage));

    QMetaObject::invokeMethod(
      progress_bar_, "setFormat",
      Qt::QueuedConnection,
      Q_ARG(QString, QString("%1/%2 (%p%)").arg(current).arg(total)));
  } else {
    QMetaObject::invokeMethod(
      progress_bar_, "setValue",
      Qt::QueuedConnection,
      Q_ARG(int, 0));

    QMetaObject::invokeMethod(
      progress_bar_, "setFormat",
      Qt::QueuedConnection,
      Q_ARG(QString, QString("0/0 (0%)")));
  }
}

void RaspicatNavigationPanel::updateWaypointTable()
{
  // This would require loading waypoint data from the CSV or from a service
  // For now, this is a placeholder
  // In a full implementation, you would parse the CSV or request waypoint data
}

void RaspicatNavigationPanel::updateButtonStates(const std::string & state)
{
  bool is_idle = (state == "IDLE");
  bool is_navigating = (state == "NAVIGATING");
  bool is_waiting = (state == "WAITING");
  bool is_completed = (state == "COMPLETED");
  bool is_error = (state == "ERROR");

  // Start button: enabled when IDLE, COMPLETED, or ERROR
  QMetaObject::invokeMethod(
    start_button_, "setEnabled",
    Qt::QueuedConnection,
    Q_ARG(bool, is_idle || is_completed || is_error));

  // Pause button: enabled when NAVIGATING or WAITING
  QMetaObject::invokeMethod(
    pause_button_, "setEnabled",
    Qt::QueuedConnection,
    Q_ARG(bool, is_navigating || is_waiting));

  // Resume button: enabled when WAITING
  QMetaObject::invokeMethod(
    resume_button_, "setEnabled",
    Qt::QueuedConnection,
    Q_ARG(bool, is_waiting));

  // Skip button: enabled when NAVIGATING or WAITING
  QMetaObject::invokeMethod(
    skip_button_, "setEnabled",
    Qt::QueuedConnection,
    Q_ARG(bool, is_navigating || is_waiting));
}

}  // namespace raspicat_tvvf_navigation

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(raspicat_tvvf_navigation::RaspicatNavigationPanel, rviz_common::Panel)
