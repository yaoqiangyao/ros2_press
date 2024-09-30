#ifndef PERFORMANCE_TEST__PLUGINS__ROS2__RCLCPP_CALLBACK_COMMUNICATOR_HPP_
#define PERFORMANCE_TEST__PLUGINS__ROS2__RCLCPP_CALLBACK_COMMUNICATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <iostream>
#include <regex>
#include <std_msgs/msg/string.hpp>

#include "performance_test/plugin/publisher.hpp"
#include "performance_test/plugin/subscriber.hpp"
#include "rclcpp_common/rclcpp_resource_manager.hpp"
#include "rclcpp_common/ros2_qos_adapter.hpp"

namespace performance_test
{

template<class Msg, class Executor>
class RclcppCallbackSubscriber : public Subscriber
{
public:
  using DataType = typename Msg::RosType;

  explicit RclcppCallbackSubscriber(const ExperimentConfiguration & ec)
  : m_node(RclcppResourceManager::get().rclcpp_node(ec)),
    m_ROS2QOSAdapter(ROS2QOSAdapter(ec.qos).get())
  {
    std::string topic_name = ec.topic_name + ec.sub_topic_postfix();          // 默认情况

    // 创建订阅者，接收 std_msgs::msg::String
    m_subscription = m_node->create_subscription<std_msgs::msg::String>(
      topic_name,
      m_ROS2QOSAdapter,
      [this](const std_msgs::msg::String::SharedPtr message) {this->callback(message);}
    );

    std::cout << "订阅topic: " << topic_name << std::endl;
    m_executor.add_node(this->m_node);
  }

  void prepare() override
  {
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
    if (ec.expected_num_pubs > 0) {
      m_subscription->wait_for_matched(
        ec.expected_num_pubs,
        ec.wait_for_matched_timeout);
    }
#endif
  }

  void update_subscription(MessageReceivedListener & listener) override
  {
    m_listener = &listener;
    m_executor.spin_once(std::chrono::milliseconds(100));
  }

private:
  std::shared_ptr<rclcpp::Node> m_node;
  rclcpp::QoS m_ROS2QOSAdapter;
  Executor m_executor;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> m_subscription;
  MessageReceivedListener * m_listener;

  void callback(const std_msgs::msg::String::SharedPtr & message)
  {
    auto data = std::make_shared<DataType>();
    parse_message(message, data);
    const auto received_time = now_int64_t();

    m_listener->on_message_received(
      data->time,
      received_time,
      data->id,
      sizeof(DataType)
    );
  }

  void parse_message(
    const std_msgs::msg::String::SharedPtr & message,
    typename DataType::SharedPtr data)
  {
    std::istringstream ss(message->data);
    std::string token;

    if (std::getline(ss, token, ',')) {
      data->time = std::stoll(token);
    }

    if (std::getline(ss, token, ',')) {
      data->id = static_cast<decltype(data->id)>(std::stoi(token));
    }
    // 继续解析其他字段（如果有的话）
  }
};

#ifdef PERFORMANCE_TEST_RCLCPP_STE_ENABLED
template<class Msg>
using RclcppSingleThreadedExecutorSubscriber =
  RclcppCallbackSubscriber<Msg, rclcpp::executors::SingleThreadedExecutor>;
#endif
#ifdef PERFORMANCE_TEST_RCLCPP_SSTE_ENABLED
template<class Msg>
using RclcppStaticSingleThreadedExecutorSubscriber =
  RclcppCallbackSubscriber<Msg, rclcpp::executors::StaticSingleThreadedExecutor>;
#endif
}  // namespace performance_test
#endif  // PERFORMANCE_TEST__PLUGINS__ROS2__RCLCPP_CALLBACK_COMMUNICATOR_HPP_
