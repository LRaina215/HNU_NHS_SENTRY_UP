// src/bt_brain_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <std_msgs/msg/bool.hpp>  // <--- 新增这行！

using namespace BT;

// ================= 1. 自定义动作节点 =================
// 负责向 Python 四肢发送执行指令
class SendCommand : public SyncActionNode
{
public:
    SendCommand(const std::string& name, const NodeConfiguration& config, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub)
        : SyncActionNode(name, config), publisher_(pub) {}

    static PortsList providedPorts() { return { InputPort<std::string>("tactic") }; }

    NodeStatus tick() override
    {
        std::string tactic;
        if (!getInput("tactic", tactic)) { return NodeStatus::FAILURE; }
        
        std_msgs::msg::String msg;
        msg.data = tactic;
        publisher_->publish(msg); // 发送给 Python
        
        return NodeStatus::SUCCESS;
    }
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

// ================= 自定义条件节点：检测人类接管 =================
class IsHumanOverride : public ConditionNode
{
public:
    IsHumanOverride(const std::string& name, const NodeConfiguration& config)
        : ConditionNode(name, config) {}

    static PortsList providedPorts() { return {}; }

    NodeStatus tick() override
    {
        bool is_override = false;
        // 从黑板中读取状态，读不到默认 false
        if (config().blackboard->get<bool>("is_human_override", is_override)) {
            if (is_override) {
                return NodeStatus::SUCCESS; // 处于接管期，条件成立！
            }
        }
        return NodeStatus::FAILURE; // 未被接管，条件不成立
    }
};

// ================= 2. ROS 2 宿主节点 =================
class BrainNode : public rclcpp::Node
{
public:
    BrainNode() : Node("rmuc_brain_node")
    {
        cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/sentry/tactic_cmd", 10);
        
        // 初始化黑板数据 (模拟)
        blackboard_ = Blackboard::create();
        BT::BehaviorTreeFactory factory;
        blackboard_->set<int>("hp", 2000);            // 默认满血
        blackboard_->set<bool>("has_enemy", false);   // 默认无敌情
        blackboard_->set<float>("target_distance", 10.0f); // 默认距离远

        // 订阅 Python 提纯后的战场数据
        bb_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sentry/blackboard_data", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() >= 3) {
                    blackboard_->set<int>("hp", static_cast<int>(msg->data[0]));
                    blackboard_->set<bool>("has_enemy", msg->data[1] > 0.5);
                    blackboard_->set<float>("target_distance", msg->data[2]);
                }
            });

        // 新增：订阅 Python 发来的接管状态
        override_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/sentry/human_override", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
        // 收到消息后，立刻写进行为树的黑板！
        blackboard_->set<bool>("is_human_override", msg->data);
        });
        // 注册我们新写的节点
        factory.registerNodeType<IsHumanOverride>("IsHumanOverride");
    }

    void run_tree()
    {
        // 1. 创建工厂
        BehaviorTreeFactory factory;

        // 2. 🚨 关键：在这里注册 IsHumanOverride 节点！
        factory.registerNodeType<IsHumanOverride>("IsHumanOverride");

        // 3. 注册其他的条件节点 (Lambda)
        factory.registerSimpleCondition("IsLowHP", [&](TreeNode&) {
            int hp = blackboard_->get<int>("hp");
            return (hp < 150) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        });

        factory.registerSimpleCondition("HasEnemy", [&](TreeNode&) {
            bool has_enemy = blackboard_->get<bool>("has_enemy");
            return has_enemy ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        });

        // 注册动作节点
        factory.registerBuilder<SendCommand>("SendCommand", [&](const std::string& name, const NodeConfiguration& config) {
            return std::make_unique<SendCommand>(name, config, cmd_pub_);
        });

        factory.registerSimpleCondition("IsEnemyClose", [&](TreeNode&) {
            float dist = blackboard_->get<float>("target_distance");
            return (dist < 2.0) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        });

        // 4. 加载 XML
        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("rmuc_bt_brain");
        std::string xml_path = pkg_share_dir + "/tree.xml";
        
        // 🚨 这里的 factory 现在包含了 IsHumanOverride，不会再报错了！
        auto tree = factory.createTreeFromFile(xml_path, blackboard_);

        // ... (后续的 ZMQ 和 while 循环代码保持不变) ...
        PublisherZMQ publisher_zmq(tree, 25, 2666, 2667);
        RCLCPP_INFO(this->get_logger(), "C++ 大脑启动成功！");

        rclcpp::Rate rate(10);
        while (rclcpp::ok()) {
            tree.tickRoot();
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }

    

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
    Blackboard::Ptr blackboard_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bb_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr override_sub_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BrainNode>();
    node->run_tree();
    rclcpp::shutdown();
    return 0;
}