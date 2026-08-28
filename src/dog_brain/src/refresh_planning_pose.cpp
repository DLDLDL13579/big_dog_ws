#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dog_nav_bt
{
class RefreshPlanningPose : public BT::SyncActionNode
{
public:
  RefreshPlanningPose(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(xml_tag_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_pose", "Original goal pose"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_pose", "Goal pose with refreshed timestamp (Time=0)")
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped input_pose;
    if (!getInput("input_pose", input_pose)) {
      return BT::NodeStatus::FAILURE;
    }

    // 复制原始 pose，保留 x/y/yaw/frame_id 不变
    geometry_msgs::msg::PoseStamped output_pose = input_pose;

    // 只刷新时间戳为 0
    // TF2 中 Time(0) 代表请求最新可用变换，避免旧 Goal 时间戳超出 TF buffer
    output_pose.header.stamp = builtin_interfaces::msg::Time();

    setOutput("output_pose", output_pose);
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace dog_nav_bt

// 注册为 BT 插件
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<dog_nav_bt::RefreshPlanningPose>("RefreshPlanningPose");
}
