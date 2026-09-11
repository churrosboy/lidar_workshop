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
#include <vector>

namespace gl5_driver {
class RegionPanel : public rviz_common::Panel {
 public:
  explicit RegionPanel(QWidget* parent = nullptr) : Panel(parent) {
    auto* layout = new QVBoxLayout(this);
    status_ = new QLabel("상태: 감지 노드 연결 대기", this);
    status_->setObjectName("roi_status");
    status_->setWordWrap(true);
    status_->setStyleSheet("font-weight: bold; padding: 5px;");
    layout->addWidget(status_);
    auto* help = new QLabel("영역 지정 → 화면에 꼭짓점 클릭 → 영역 확정\n확대·축소: 카메라 조작 버튼을 누른 뒤 휠", this);
    help->setWordWrap(true);
    layout->addWidget(help);
    auto* grid = new QGridLayout();
    addButton(grid, "영역 지정", "edit", 0, 0);
    addButton(grid, "영역 확정", "finish", 0, 1);
    addButton(grid, "마지막 점 취소", "undo", 1, 0);
    addButton(grid, "편집 취소", "cancel", 1, 1);
    addButton(grid, "영역 전체 삭제", "clear", 2, 0);
    addButton(grid, "저장 영역 불러오기", "load", 2, 1);
    layout->addLayout(grid);
    auto* camera = new QPushButton("카메라 조작 (확대·축소)", this);
    connect(camera, &QPushButton::clicked, this, [this]() { chooseTool("rviz_default_plugins/MoveCamera"); });
    layout->addWidget(camera);
    message_ = new QLabel("영역을 지정하면 해당 구역의 장애물만 표시합니다.", this);
    message_->setObjectName("roi_message");
    message_->setWordWrap(true);
    message_->setMinimumHeight(42);
    layout->addWidget(message_);
    layout->addStretch();
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this]() { tick(); });
  }

  void onInitialize() override {
    node_ = std::make_shared<rclcpp::Node>("gl5_region_panel");
    executor_.add_node(node_);
    for (const auto& entry : buttons_) {
      clients_[entry.first] = node_->create_client<std_srvs::srv::Trigger>("/gl5/region/" + entry.first);
    }
    subscription_ = node_->create_subscription<std_msgs::msg::String>(
      "/gl5/obstacle_state", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        last_state_ = std::chrono::steady_clock::now();
        const std::map<std::string, QString> labels{
          {"NO_REGION", "영역 없음"}, {"EDITING", "영역 편집 중 — 확정 버튼을 눌러주세요"},
          {"NO_DATA", "라이다 데이터 없음"}, {"CLEAR", "영역 내 장애물 없음"},
          {"OCCUPIED", "영역 내 장애물 감지"}};
        auto it = labels.find(msg->data);
        status_->setText("상태: " + (it == labels.end() ? QString::fromStdString(msg->data) : it->second));
        status_->setStyleSheet(msg->data == "OCCUPIED" ?
          "font-weight: bold; color: #bb2222; padding: 5px;" : "font-weight: bold; padding: 5px;");
      });
    timer_->start(50);
  }

  ~RegionPanel() override {
    timer_->stop();
    if (node_) executor_.remove_node(node_);
  }

 private:
  using Client = rclcpp::Client<std_srvs::srv::Trigger>;
  void addButton(QGridLayout* grid, const QString& text, const std::string& action, int row, int column) {
    auto* button = new QPushButton(text, this);
    button->setObjectName(QString::fromStdString("roi_" + action));
    buttons_[action] = button;
    connect(button, &QPushButton::clicked, this, [this, action]() { request(action); });
    grid->addWidget(button, row, column);
  }
  void chooseTool(const QString& id) {
    auto* context = getDisplayContext();
    if (!context) return;
    auto* manager = context->getToolManager();
    for (int i = 0; i < manager->numTools(); ++i) {
      if (manager->getTool(i)->getClassId() == id) {
        manager->setCurrentTool(manager->getTool(i)); return;
      }
    }
    manager->setCurrentTool(manager->addTool(id));
  }
  void request(const std::string& action) {
    auto found = clients_.find(action);
    if (found == clients_.end() || !found->second->service_is_ready()) {
      message_->setText("감지 노드에 연결되지 않았습니다. 실행 상태를 확인하세요."); return;
    }
    pending_ = true;
    active_client_ = found->second;
    deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    for (auto& entry : buttons_) entry.second->setEnabled(false);
    message_->setText("처리 중…");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = active_client_->async_send_request(request, [this, action](Client::SharedFuture future) {
      pending_ = false;
      const auto response = future.get();
      if (response->success) {
        message_->setText(action == "clear" ? "영역과 저장된 영역을 모두 삭제했습니다." :
          action == "edit" ? "화면에 꼭짓점을 순서대로 찍고 영역 확정을 누르세요." :
          action == "finish" ? "영역을 확정하고 저장했습니다." : "완료했습니다.");
        if (action == "edit") chooseTool("rviz_default_plugins/PublishPoint");
        else if (action != "undo") chooseTool("rviz_default_plugins/Interact");
      } else {
        message_->setText("실패: " + QString::fromStdString(response->message));
      }
      for (auto& entry : buttons_) entry.second->setEnabled(true);
    });
    request_id_ = result.request_id;
  }
  void tick() {
    if (!node_ || !rclcpp::ok()) return;
    executor_.spin_some(std::chrono::milliseconds(5));
    const auto now = std::chrono::steady_clock::now();
    if (last_state_.time_since_epoch().count() == 0 || now - last_state_ > std::chrono::seconds(2))
      status_->setText("상태: 감지 노드 연결 대기");
    if (pending_ && now > deadline_) {
      active_client_->remove_pending_request(request_id_);
      pending_ = false;
      message_->setText("응답 시간 초과. 현재 영역 상태를 확인하세요.");
      for (auto& entry : buttons_) entry.second->setEnabled(true);
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
}

PLUGINLIB_EXPORT_CLASS(gl5_driver::RegionPanel, rviz_common::Panel)
