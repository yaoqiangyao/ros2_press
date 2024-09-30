// Copyright 2022-2024 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PERFORMANCE_TEST__PLUGINS__RCLCPP_COMMON__RCLCPP_PUBLISHER_HPP_
#define PERFORMANCE_TEST__PLUGINS__RCLCPP_COMMON__RCLCPP_PUBLISHER_HPP_


#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <utility>
#include <regex>

#include "performance_test/experiment_configuration/qos_abstraction.hpp"
#include "performance_test/plugin/publisher.hpp"
#include "performance_test/plugin/subscriber.hpp"
#include "performance_test/utilities/msg_traits.hpp"
#include "rclcpp_common/rclcpp_resource_manager.hpp"
#include "rclcpp_common/ros2_qos_adapter.hpp"
#include <std_msgs/msg/string.hpp>

namespace performance_test
{
template<class Msg>
class RclcppPublisher : public Publisher
{
public:
//        using DataType = typename Msg::RosType;
  using DataType = std_msgs::msg::String;

  explicit RclcppPublisher(const ExperimentConfiguration & ec)
  : Publisher(ec),
    m_node(RclcppResourceManager::get().rclcpp_node(ec)),
    m_ROS2QOSAdapter(ROS2QOSAdapter(ec.qos).get())
  {
    // 初始化发布的 topic 名称
    std::string topic_name = ec.topic_name + ec.pub_topic_postfix();          // 默认情况

    // 创建发布者
    m_publisher = m_node->create_publisher<DataType>(topic_name, m_ROS2QOSAdapter);

    std::cout << "发布topic: " << topic_name << std::endl;
  }


  void prepare() override
  {
#ifdef PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED
    if (m_ec.expected_num_subs > 0) {
      m_publisher->wait_for_matched(
        m_ec.expected_num_subs,
        m_ec.wait_for_matched_timeout);
    }
#endif
  }

  void publish_copy(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
//            init_msg(m_data, timestamp_provider, sample_id);

    std::string message = create_message(timestamp_provider, sample_id);
    DataType msg;
    msg.data = message;          // 将生成的字符串赋值给 msg.data
//            std::cout << "Publishing loaned message111: " << msg.data << std::endl;

    m_publisher->publish(msg);
  }

  void publish_loaned(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
#ifdef PERFORMANCE_TEST_RCLCPP_ZERO_COPY_ENABLED
    if (!m_publisher->can_loan_messages()) {
      throw std::runtime_error("RMW implementation does not support zero copy!");
    }
    auto borrowed_message = m_publisher->borrow_loaned_message();

    // 生成消息字符串
    std::string message = create_message(timestamp_provider, sample_id);

    // 设置借用消息的数据
    borrowed_message.get().data = message;

    // 确保借用的消息在发布之前没有被修改
    std::cout << "Publishing loaned message222: " << borrowed_message.get().data << std::endl;

    // 发布借用的消息
    m_publisher->publish(std::move(borrowed_message));
#else
    throw std::runtime_error("ROS2 distribution does not support zero copy!");
#endif
  }

private:
  std::string create_message(const TimestampProvider & timestamp_provider, std::uint64_t sample_id)
  {
    // 根据需要格式化字符串，使用分隔符连接时间戳和 ID
    std::string timestamp = std::to_string(timestamp_provider.get());
    return timestamp + "," + std::to_string(sample_id);          // 用逗号分隔
  }
  std::shared_ptr<rclcpp::Node> m_node;
  rclcpp::QoS m_ROS2QOSAdapter;
  std::shared_ptr<::rclcpp::Publisher<DataType>> m_publisher;
  DataType m_data;
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGINS__RCLCPP_COMMON__RCLCPP_PUBLISHER_HPP_
