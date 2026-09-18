#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/tool.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <chrono>
#include <map>
#include <memory>
#include <string>

namespace gl5_rviz_plugins {
class RegionPanel : public rviz_common::Panel {
 public:
  explicit RegionPanel(QWidget* parent = nullptr) : Panel(parent) {
    createWidgets();
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this]() { tick(); });
  }

  void onInitialize() override {
    node_ = std::make_shared<rclcpp::Node>("gl5_region_panel");
    executor_.add_node(node_);
    for (const auto& entry : buttons_) {
      clients_[entry.first] =
          node_->create_client<std_srvs::srv::Trigger>("/gl5/region/" + entry.first);
    }
    subscription_ = node_->create_subscription<std_msgs::msg::String>(
        "/gl5/obstacle_state", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr msg) { updateStatus(msg->data); });
    timer_->start(50);
  }

  ~RegionPanel() override {
    timer_->stop();
    if (node_) {
      executor_.remove_node(node_);
    }
  }

 private:
  using Client = rclcpp::Client<std_srvs::srv::Trigger>;

  void createWidgets() {
    auto* layout = new QVBoxLayout(this);
    status_ = new QLabel("Status: Waiting for detector", this);
    status_->setObjectName("roi_status");
    status_->setWordWrap(true);
    status_->setStyleSheet("font-weight: bold; padding: 5px;");
    layout->addWidget(status_);
    auto* help = new QLabel(
        "Draw Region: click vertices, then Finish Region.\nZoom: select Move Camera, then scroll.",
        this);
    help->setWordWrap(true);
    layout->addWidget(help);
    auto* grid = new QGridLayout();
    addButton(grid, "Draw Region", "edit", 0, 0);
    addButton(grid, "Finish Region", "finish", 0, 1);
    addButton(grid, "Undo Last Point", "undo", 1, 0);
    addButton(grid, "Cancel Edit", "cancel", 1, 1);
    addButton(grid, "Clear Region", "clear", 2, 0);
    addButton(grid, "Load Saved Region", "load", 2, 1);
    layout->addLayout(grid);
    auto* camera = new QPushButton("Move Camera (Zoom)", this);
    connect(camera, &QPushButton::clicked, this,
            [this]() { chooseTool("rviz_default_plugins/MoveCamera"); });
    layout->addWidget(camera);
    message_ = new QLabel("Draw a region to show obstacles inside it.", this);
    message_->setObjectName("roi_message");
    message_->setWordWrap(true);
    message_->setMinimumHeight(42);
    layout->addWidget(message_);
    layout->addStretch();
  }

  void updateStatus(const std::string& state) {
    last_state_ = std::chrono::steady_clock::now();
    const std::map<std::string, QString> labels{{"NO_REGION", "No region"},
                                                {"EDITING", "Editing region - click Finish Region"},
                                                {"NO_DATA", "No LiDAR data"},
                                                {"CLEAR", "No obstacles in region"},
                                                {"OCCUPIED", "Obstacle detected in region"}};
    auto it = labels.find(state);
    status_->setText("Status: " +
                     (it == labels.end() ? QString::fromStdString(state) : it->second));
    status_->setStyleSheet(state == "OCCUPIED" ? "font-weight: bold; color: #bb2222; padding: 5px;"
                                               : "font-weight: bold; padding: 5px;");
  }

  void handleResponse(const std::string& action,
                      const std_srvs::srv::Trigger::Response::SharedPtr& response) {
    pending_ = false;
    if (response->success) {
      message_->setText(successMessage(action));
      if (action == "edit") {
        chooseTool("rviz_default_plugins/PublishPoint");
      } else if (action != "undo") {
        chooseTool("rviz_default_plugins/Interact");
      }
    } else {
      message_->setText("Failed: " + QString::fromStdString(response->message));
    }
    setButtonsEnabled(true);
  }

  static QString successMessage(const std::string& action) {
    if (action == "clear") {
      return "Current and saved regions cleared.";
    }
    if (action == "edit") {
      return "Click vertices in order, then click Finish Region.";
    }
    if (action == "finish") {
      return "Region confirmed and saved.";
    }
    return "Done.";
  }

  void setButtonsEnabled(bool enabled) {
    for (auto& entry : buttons_) {
      entry.second->setEnabled(enabled);
    }
  }

  void addButton(QGridLayout* grid, const QString& text, const std::string& action, int row,
                 int column) {
    auto* button = new QPushButton(text, this);
    button->setObjectName(QString::fromStdString("roi_" + action));
    buttons_[action] = button;
    connect(button, &QPushButton::clicked, this, [this, action]() { request(action); });
    grid->addWidget(button, row, column);
  }
  void chooseTool(const QString& id) {
    auto* context = getDisplayContext();
    if (!context) {
      return;
    }
    auto* manager = context->getToolManager();
    for (int i = 0; i < manager->numTools(); ++i) {
      if (manager->getTool(i)->getClassId() == id) {
        manager->setCurrentTool(manager->getTool(i));
        return;
      }
    }
    manager->setCurrentTool(manager->addTool(id));
  }
  void request(const std::string& action) {
    auto found = clients_.find(action);
    if (found == clients_.end() || !found->second->service_is_ready()) {
      message_->setText("Detector not connected. Check that it is running.");
      return;
    }
    pending_ = true;
    active_client_ = found->second;
    deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    setButtonsEnabled(false);
    message_->setText("Processing...");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = active_client_->async_send_request(
        request,
        [this, action](Client::SharedFuture future) { handleResponse(action, future.get()); });
    request_id_ = result.request_id;
  }
  void tick() {
    if (!node_ || !rclcpp::ok()) {
      return;
    }
    executor_.spin_some(std::chrono::milliseconds(5));
    const auto now = std::chrono::steady_clock::now();
    if (last_state_.time_since_epoch().count() == 0 ||
        now - last_state_ > std::chrono::seconds(2)) {
      status_->setText("Status: Waiting for detector");
    }
    if (pending_ && now > deadline_) {
      active_client_->remove_pending_request(request_id_);
      pending_ = false;
      message_->setText("Request timed out. Check the current region status.");
      setButtonsEnabled(true);
    }
  }
  QLabel *status_, *message_;
  QTimer* timer_;
  std::map<std::string, QPushButton*> buttons_;
  std::map<std::string, Client::SharedPtr> clients_;
  Client::SharedPtr active_client_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  bool pending_ = false;
  int64_t request_id_ = 0;
  std::chrono::steady_clock::time_point deadline_, last_state_;
};
}  // namespace gl5_rviz_plugins

PLUGINLIB_EXPORT_CLASS(gl5_rviz_plugins::RegionPanel, rviz_common::Panel)
