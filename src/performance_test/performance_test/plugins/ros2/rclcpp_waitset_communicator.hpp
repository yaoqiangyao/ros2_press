// Copyright 2021-2024 Apex.AI, Inc.
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

#ifndef PERFORMANCE_TEST__PLUGINS__ROS2__RCLCPP_WAITSET_COMMUNICATOR_HPP_
#define PERFORMANCE_TEST__PLUGINS__ROS2__RCLCPP_WAITSET_COMMUNICATOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <algorithm>
#include <vector>

#include "rclcpp_common/rclcpp_resource_manager.hpp"
#include "rclcpp_common/ros2_qos_adapter.hpp"

namespace performance_test
{

template<class Msg>
class RclcppWaitsetSubscriber : public Subscriber
{
public:
  using DataType = typename Msg::RosType;

  explicit RclcppWaitsetSubscriber(const ExperimentConfiguration & ec)
  : m_node(RclcppResourceManager::get().rclcpp_node(ec)),
    m_ROS2QOSAdapter(ROS2QOSAdapter(ec.qos).get()),
    m_subscription(this->m_node->template create_subscription<DataType>(
        ec.topic_name + ec.sub_topic_postfix(),
        m_ROS2QOSAdapter,
        [](const typename DataType::SharedPtr) {})),
    m_timeout(std::max(
        10 * ec.period_ns(),
        std::chrono::nanoseconds(100 * 1000 * 1000)))
  {
    m_waitset.add_subscription(m_subscription);
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
    // This timeout ensures the wait will unblock when the program receives an INT signal
    const auto wait_ret = m_waitset.wait(m_timeout);
    if (wait_ret.kind() == rclcpp::Ready) {
      take(listener);
    }
  }

  void take(MessageReceivedListener & listener) override
  {
    DataType msg;
    rclcpp::MessageInfo msg_info;
    bool success = m_subscription->take(msg, msg_info);
    const auto received_time = now_int64_t();
    if (success) {
      listener.on_message_received(
        msg.time,
        received_time,
        msg.id,
        sizeof(DataType)
      );
    }
  }

private:
  std::shared_ptr<rclcpp::Node> m_node;
  rclcpp::QoS m_ROS2QOSAdapter;
  std::shared_ptr<::rclcpp::Subscription<DataType>> m_subscription;
  rclcpp::WaitSet m_waitset;
  std::chrono::nanoseconds m_timeout;
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGINS__ROS2__RCLCPP_WAITSET_COMMUNICATOR_HPP_
