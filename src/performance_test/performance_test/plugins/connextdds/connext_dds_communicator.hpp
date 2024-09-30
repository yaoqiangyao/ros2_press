// Copyright 2017-2024 Apex.AI, Inc.
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

#ifndef PERFORMANCE_TEST__PLUGINS__CONNEXTDDS__CONNEXT_DDS_COMMUNICATOR_HPP_
#define PERFORMANCE_TEST__PLUGINS__CONNEXTDDS__CONNEXT_DDS_COMMUNICATOR_HPP_

#include <mutex>
#include <vector>

#include <ndds/ndds_cpp.h>

#include "performance_test/plugin/publisher.hpp"
#include "performance_test/plugin/subscriber.hpp"

namespace performance_test
{

/**
 * \brief Translates abstract QOS settings to specific QOS settings for Connext DDS data writers and readers.
 *
 * The reason that this class is constructed like this is that one usually gets a partially specified QOS from the topic
 * or similar higher level entity and just changes some settings from these.
 */
class ConnextDDSQOSAdapter
{
public:
  /**
   * \brief Constructs the QOS adapter.
   * \param qos The abstract QOS settings the adapter should use to derive the implementation specific QOS settings.
   */
  explicit ConnextDDSQOSAdapter(const QOSAbstraction qos)
  : m_qos(qos)
  {}
  /**
   * \brief  Applies the abstract QOS to an existing QOS leaving unsupported values as they were.
   * \tparam ConnextDDSQos The type of the QOS setting, for example data reader or data writer QOS.
   * \param qos The QOS settings to fill supported values in.
   */
  template<class ConnextDDSQos>
  void apply(ConnextDDSQos & qos)
  {
    if (m_qos.reliability == QOSAbstraction::Reliability::BEST_EFFORT) {
      qos.reliability.kind = DDS_BEST_EFFORT_RELIABILITY_QOS;
    } else if (m_qos.reliability == QOSAbstraction::Reliability::RELIABLE) {
      qos.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.durability == QOSAbstraction::Durability::VOLATILE) {
      qos.durability.kind = DDS_VOLATILE_DURABILITY_QOS;
    } else if (m_qos.durability == QOSAbstraction::Durability::TRANSIENT_LOCAL) {
      qos.durability.kind = DDS_TRANSIENT_LOCAL_DURABILITY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_ALL) {
      qos.history.kind = DDS_KEEP_ALL_HISTORY_QOS;
    } else if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      qos.history.kind = DDS_KEEP_LAST_HISTORY_QOS;
    } else {
      throw std::runtime_error("Unsupported QOS!");
    }

    if (m_qos.history_kind == QOSAbstraction::HistoryKind::KEEP_LAST) {
      qos.history.depth = static_cast<decltype(qos.history.depth)>(m_qos.history_depth);
    } else {
      // Keep all, keeps all. No depth required.
    }
  }

private:
  const QOSAbstraction m_qos;
};

template<class Topic>
class RTIDDSTopicManager
{
public:
  static DDSTopic * register_topic(
    DDSDomainParticipant * m_participant,
    const ExperimentConfiguration & m_ec)
  {
    if (m_topic == nullptr) {
      auto retcode = TypeSupport::register_type(
        m_participant,
        Topic::msg_name().c_str());

      if (retcode != DDS_RETCODE_OK) {
        throw std::runtime_error("failed to register type");
      }

      m_topic = m_participant->create_topic(
        m_ec.topic_name.c_str(),
        Topic::msg_name().c_str(),
        DDS_TOPIC_QOS_DEFAULT,
        nullptr,
        DDS_STATUS_MASK_NONE);

      if (m_topic == nullptr) {
        throw std::runtime_error("topic == nullptr");
      }
    }
    return m_topic;
  }

private:
  static DDSTopic * m_topic;
};

template<class Topic>
DDSTopic * RTIDDSTopicManager<Topic>::m_topic = nullptr;

class RTIDDSResourceManager
{
public:
  static RTIDDSResourceManager & get()
  {
    static RTIDDSResourceManager instance;
    return instance;
  }

  RTIDDSResourceManager(RTIDDSResourceManager const &) = delete;
  RTIDDSResourceManager(RTIDDSResourceManager &&) = delete;
  RTIDDSResourceManager & operator=(RTIDDSResourceManager const &) = delete;
  RTIDDSResourceManager & operator=(RTIDDSResourceManager &&) = delete;

  DDSDomainParticipant * connext_dds_participant(const ExperimentConfiguration & ec) const
  {
    std::lock_guard<std::mutex> lock(m_global_mutex);

    if (!m_connext_dds_participant) {
      m_connext_dds_participant = DDSTheParticipantFactory->create_participant(
        ec.dds_domain_id, DDS_PARTICIPANT_QOS_DEFAULT,
        NULL /* listener */, DDS_STATUS_MASK_NONE);
      if (m_connext_dds_participant == NULL) {
        throw std::runtime_error("Participant is nullptr");
      }
    }
    return m_connext_dds_participant;
  }

  /**
   * \brief Creates a new Connext DDS publisher.
   * \param publisher Will be overwritten with the created publisher.
   * \param dw_qos Will be overwritten with the default QOS from the created publisher.
   */
  void connext_dds_publisher(
    const ExperimentConfiguration & ec,
    DDSPublisher * & publisher,
    DDS_DataWriterQos & dw_qos) const
  {
    auto participant = connext_dds_participant(ec);
    std::lock_guard<std::mutex> lock(m_global_mutex);
    publisher = participant->create_publisher(
      DDS_PUBLISHER_QOS_DEFAULT, nullptr, DDS_STATUS_MASK_NONE);
    if (publisher == nullptr) {
      throw std::runtime_error("Pulisher is nullptr");
    }
    auto retcode = publisher->get_default_datawriter_qos(dw_qos);
    if (retcode != DDS_RETCODE_OK) {
      throw std::runtime_error("Failed to get default datawriter");
    }
  }

  /**
   * \brief Creates a new Connext DDS subscriber.
   * \param subscriber Will be overwritten with the created subscriber.
   * \param dr_qos Will be overwritten with the default QOS from the created subscriber.
   */
  void connext_dds_subscriber(
    const ExperimentConfiguration & ec,
    DDSSubscriber * & subscriber,
    DDS_DataReaderQos & dr_qos) const
  {
    auto participant = connext_dds_participant(ec);
    std::lock_guard<std::mutex> lock(m_global_mutex);
    subscriber = participant->create_subscriber(
      DDS_SUBSCRIBER_QOS_DEFAULT, nullptr,
      DDS_STATUS_MASK_NONE);
    if (subscriber == nullptr) {
      throw std::runtime_error("m_subscriber == nullptr");
    }
    auto retcode = subscriber->get_default_datareader_qos(dr_qos);
    if (retcode != DDS_RETCODE_OK) {
      throw std::runtime_error("failed get_default_datareader_qos");
    }
  }

private:
  RTIDDSResourceManager()
  : m_connext_dds_participant(nullptr) {}

  mutable DDSDomainParticipant * m_connext_dds_participant;
  mutable std::mutex m_global_mutex;
};

/**
 * \brief The plugin for Connext DDS.
 * \tparam Topic The topic type to use.
 *
 * The code in here is derived from the C++ example in the Connext DDS installation folder.
 */
template<class Topic>
class RTIDDSPublisher : public Publisher
{
public:
  using DataType = typename Topic::ConnextDDSType;
  using TypeSupport = typename DataType::TypeSupport;
  using DataWriterType = typename DataType::DataWriter;
  using DataTypeSeq = typename DataType::Seq;

  explicit RTIDDSPublisher(const ExperimentConfiguration & ec)
  : Publisher(ec),
    m_participant(RTIDDSResourceManager::get().connext_dds_participant(ec)),
    m_datawriter(nullptr),
    m_topic(RTIDDSTopicManager::register_topic(m_participant, ec))
  {
    DDSPublisher * publisher;
    DDS_DataWriterQos dw_qos;
    RTIDDSResourceManager::get().connext_dds_publisher(ec, publisher, dw_qos);

    dw_qos.resource_limits.max_samples = 100;
    dw_qos.resource_limits.max_samples_per_instance = 100;
    dw_qos.resource_limits.max_instances = 1;
    dw_qos.resource_limits.initial_instances = 1;

    ConnextDDSQOSAdapter qos_adapter(m_ec.qos);
    qos_adapter.apply(dw_qos);

    m_datawriter = publisher->create_datawriter(
      m_topic, dw_qos, nullptr, DDS_STATUS_MASK_NONE);
    if (m_datawriter == nullptr) {
      throw std::runtime_error("Could not create datawriter");
    }

    m_typed_datawriter = DataWriterType::narrow(m_datawriter);
    if (m_typed_datawriter == nullptr) {
      throw std::runtime_error("failed datawriter narrow");
    }
  }

  void publish_copy(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
    init_msg(m_data, timestamp_provider, sample_id);
    auto retcode = m_typed_datawriter->write(m_data, DDS_HANDLE_NIL);
    if (retcode != DDS_RETCODE_OK) {
      throw std::runtime_error("Failed to write to sample");
    }
  }

  void publish_loaned(
    const TimestampProvider & timestamp_provider,
    std::uint64_t sample_id) override
  {
    throw std::runtime_error("This plugin does not support zero copy transfer");
  }

private:
  DDSDomainParticipant * m_participant;

  DDSDataWriter * m_datawriter;

  DataWriterType * m_typed_datawriter;

  static DDSTopic * m_topic;

  DataType m_data;
};

/**
 * \brief The plugin for Connext DDS.
 * \tparam Topic The topic type to use.
 *
 * The code in here is derived from the C++ example in the Connext DDS installation folder.
 */
template<class Topic>
class RTIDDSSubscriber : public Subscriber
{
public:
  using DataType = typename Topic::ConnextDDSType;
  using TypeSupport = typename DataType::TypeSupport;
  using DataWriterType = typename DataType::DataWriter;
  using DataReaderType = typename DataType::DataReader;
  using DataTypeSeq = typename DataType::Seq;

  explicit RTIDDSSubscriber(const ExperimentConfiguration & ec)
  : m_ec(ec),
    m_participant(RTIDDSResourceManager::get().connext_dds_participant(ec)),
    m_datareader(nullptr),
    m_typed_datareader(nullptr),
    m_topic(RTIDDSTopicManager::register_topic(m_participant, m_ec))
  {
    DDSSubscriber * subscriber = nullptr;
    DDS_DataReaderQos dr_qos;
    RTIDDSResourceManager::get().connext_dds_subscriber(ec, subscriber, dr_qos);

    dr_qos.resource_limits.max_samples = 100;
    dr_qos.resource_limits.max_instances = 1;
    dr_qos.resource_limits.initial_instances = 1;
    dr_qos.resource_limits.max_samples_per_instance = 100;
    /* if there are more remote writers, you need to increase these limits */
    dr_qos.reader_resource_limits.max_remote_writers = 10;
    dr_qos.reader_resource_limits.max_remote_writers_per_instance = 10;

    ConnextDDSQOSAdapter qos_adapter(m_ec.qos);
    qos_adapter.apply(dr_qos);

    /* Only DDS_DATA_AVAILABLE_STATUS supported currently */
    m_datareader = subscriber->create_datareader(
      m_topic,
      dr_qos,
      nullptr,
      DDS_STATUS_MASK_NONE);

    if (m_datareader == nullptr) {
      throw std::runtime_error("datareader == nullptr");
    }

    m_condition = m_datareader->get_statuscondition();
    m_condition->set_enabled_statuses(DDS_DATA_AVAILABLE_STATUS);
    m_waitset.attach_condition(m_condition);

    m_typed_datareader = DataReaderType::narrow(m_datareader);
    if (m_typed_datareader == nullptr) {
      throw std::runtime_error("m_typed_datareader == nullptr");
    }

    if (!m_condition_seq.ensure_length(2, 2)) {
      throw std::runtime_error("Error ensuring length of active_conditions_seq.");
    }
  }

  void update_subscription(MessageReceivedListener & listener) override
  {
    DDS_Duration_t wait_timeout = {15, 0};
    m_waitset.wait(m_condition_seq, wait_timeout);

    auto ret = m_typed_datareader->take(
      m_data_seq, m_sample_info_seq, DDS_LENGTH_UNLIMITED,
      DDS_ANY_SAMPLE_STATE, DDS_ANY_VIEW_STATE, DDS_ANY_INSTANCE_STATE);
    const auto received_time = now_int64_t();
    if (ret == DDS_RETCODE_OK) {
      for (decltype(m_data_seq.length()) j = 0; j < m_data_seq.length(); ++j) {
        const auto & data = m_data_seq[j];
        if (m_sample_info_seq[j].valid_data) {
          listener.on_message_received(
            data.time,
            received_time,
            data.id,
            sizeof(DataType)
          );
        }
      }
    }

    m_typed_datareader->return_loan(m_data_seq, m_sample_info_seq);
  }

private:
  const ExperimentConfiguration & m_ec;

  DDSDomainParticipant * m_participant;

  DDSDataReader * m_datareader;

  DDSWaitSet m_waitset;
  DDSStatusCondition * m_condition;
  DDSConditionSeq m_condition_seq;

  DataReaderType * m_typed_datareader;

  DataTypeSeq m_data_seq;
  DDS_SampleInfoSeq m_sample_info_seq;
  static DDSTopic * m_topic;
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__PLUGINS__CONNEXTDDS__CONNEXT_DDS_COMMUNICATOR_HPP_
