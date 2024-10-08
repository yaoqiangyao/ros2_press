# Copyright 2021-2024 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

option(ENABLE_MSGS_ALL "Enable all messages" OFF)
set(ENABLE_MSGS_DEFAULT ${ENABLE_MSGS_ALL})

option(
  ENABLE_MSGS_ARRAY
  "Enable messages that contain an array"
  ON
)
if(ENABLE_MSGS_ARRAY)
  list(APPEND SUPPORTED_IDL_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Array32.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Array64.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Array128.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Array256.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Array512.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Array1k.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Array4k.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Array16k.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Array32k.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Array60k.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Array64k.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Array256k.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Array1m.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Array2m.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Array4m.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Array8m.idl
  )
endif()

option(
  ENABLE_MSGS_STRUCT
  "Enable messages that contain deeply-nested structs"
  ON
)
if(ENABLE_MSGS_STRUCT)
  list(APPEND SUPPORTED_IDL_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Struct16.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Struct256.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Struct4k.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/Struct32k.idl
  )
endif()

option(
  ENABLE_MSGS_POINT_CLOUD
  "Enable messages that contain a bounded sequence"
  ON
)
if(ENABLE_MSGS_POINT_CLOUD)
  list(APPEND SUPPLEMENTAL_IDL_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/FixedTime.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/FixedHeader.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/FixedPointField.idl
  )
  list(APPEND SUPPORTED_IDL_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/PointCloud512k.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/PointCloud1m.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/PointCloud2m.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/PointCloud4m.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/PointCloud8m.idl
  )
endif()

option(
  ENABLE_MSGS_BOUNDED_SEQUENCE
  "Enable messages that contain a bounded sequence"
  ${ENABLE_MSGS_DEFAULT}
)
if(ENABLE_MSGS_BOUNDED_SEQUENCE)
  list(APPEND SUPPORTED_IDL_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/BoundedSequence1k.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/BoundedSequence4k.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/BoundedSequence16k.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/BoundedSequence32k.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/BoundedSequence64k.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/BoundedSequence256k.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/BoundedSequence1m.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/BoundedSequence2m.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/BoundedSequence4m.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/BoundedSequence8m.idl
  )
endif()

option(
  ENABLE_MSGS_UNBOUNDED_SEQUENCE
  "Enable messages that contain an unbounded sequence or string"
  ${ENABLE_MSGS_DEFAULT}
)
if(ENABLE_MSGS_UNBOUNDED_SEQUENCE)
  if(APEX_CERT)
    message(FATAL_ERROR
      "ENABLE_MSGS_UNBOUNDED_SEQUENCE is not compatible with APEX_CERT")
  endif()
  list(APPEND SUPPORTED_IDL_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/UnboundedSequence.idl
    ${CMAKE_CURRENT_SOURCE_DIR}/UnboundedString.idl
  )
endif()

set(IDL_FILES ${SUPPLEMENTAL_IDL_FILES} ${SUPPORTED_IDL_FILES})

set(GENERATED_MESSAGES_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/gen/message")
list(APPEND IDLGEN_INCLUDE_DIR ${GENERATED_MESSAGES_INCLUDE_DIR})
file(MAKE_DIRECTORY "${GENERATED_MESSAGES_INCLUDE_DIR}/performance_test/generated_messages")

set(CMAKE_MESSAGE_CLASS_TYPES "// Generated message classes")
set(CMAKE_MESSAGE_INCLUDE_STATEMENTS "// Generated message files")
foreach(idl ${SUPPORTED_IDL_FILES})
  get_filename_component(filename ${idl} NAME)
  string(REGEX REPLACE "\\.idl" "" basename ${filename})
  set(CMAKE_MESSAGE_NAME ${basename})

  # if a lower case letter is followed by an uper case letter, split them with an underscore
  string(REGEX REPLACE "([a-z])([A-Z])" "\\1_\\2" CMAKE_MESSAGE_SNAKE_NAME "${CMAKE_MESSAGE_NAME}")
  string(TOLOWER "${CMAKE_MESSAGE_SNAKE_NAME}" CMAKE_MESSAGE_SNAKE_NAME)

  set(CMAKE_MESSAGE_HEADER_FILE "performance_test/generated_messages/${CMAKE_MESSAGE_SNAKE_NAME}.hpp")

  configure_file(message_file.hpp.in "${GENERATED_MESSAGES_INCLUDE_DIR}/${CMAKE_MESSAGE_HEADER_FILE}")

  set(CMAKE_MESSAGE_INCLUDE_STATEMENTS "${CMAKE_MESSAGE_INCLUDE_STATEMENTS}\n#include <${CMAKE_MESSAGE_HEADER_FILE}>")
  set(CMAKE_MESSAGE_CLASS_TYPES "${CMAKE_MESSAGE_CLASS_TYPES},\n${CMAKE_MESSAGE_NAME}")
endforeach()

configure_file(messages.hpp.in "${GENERATED_MESSAGES_INCLUDE_DIR}/performance_test/generated_messages/messages.hpp")

if(PERFORMANCE_TEST_FASTRTPS_ENABLED)
  add_subdirectory(fast_rtps)
  list(APPEND IDLGEN_INCLUDE_DIR ${FAST_RTPS_IDL_INCLUDE_DIR})
  list(APPEND IDLGEN_LIBRARIES fast_rtps_idl fastcdr)
endif()

if(PERFORMANCE_TEST_CONNEXTDDS_ENABLED)
  add_subdirectory(connextdds)
  list(APPEND IDLGEN_LIBRARIES rti_connextdds_idl)
endif()

if(PERFORMANCE_TEST_CONNEXTDDSMICRO_ENABLED)
  # There's nothing here to generate the messages from the IDL files, but Apex.AI
  # is unable to add or test the correct code without an RTI Connext Micro license.
  # add_subdirectory(connextdds_micro)
  # list(APPEND IDLGEN_INCLUDE_DIR ${RTI_MICRO_IDL_INCLUDE_DIR})
endif()

if(PERFORMANCE_TEST_CYCLONEDDS_ENABLED)
  add_subdirectory(cyclonedds)
  list(APPEND IDLGEN_INCLUDE_DIR ${CYCLONEDDS_IDL_INCLUDE_DIR})
  list(APPEND IDLGEN_LIBRARIES cyclonedds_idl)
endif()

if(PERFORMANCE_TEST_CYCLONEDDS_CXX_ENABLED)
  add_subdirectory(cyclonedds-cxx)
  list(APPEND IDLGEN_INCLUDE_DIR ${CYCLONEDDS_CXX_IDL_INCLUDE_DIR})
  list(APPEND IDLGEN_LIBRARIES cyclonedds_cxx_idl)
endif()

if(PERFORMANCE_TEST_OPENDDS_ENABLED)
  add_subdirectory(opendds)
  list(APPEND IDLGEN_INCLUDE_DIR ${OPENDDS_IDL_INCLUDE_DIR} ${PARENT_DDS_ROOT_INCLUDE})

  set(DDS_DYN_LIBS  ${PARENT_DDS_ROOT_LIBS}/libACE.so
                    ${PARENT_DDS_ROOT_LIBS}/libTAO.so
                    ${PARENT_DDS_ROOT_LIBS}/libOpenDDS_Dcps.so
                    ${PARENT_DDS_ROOT_LIBS}/libOpenDDS_Rtps.so
                    ${PARENT_DDS_ROOT_LIBS}/libOpenDDS_Rtps_Udp.so
                    ${PARENT_DDS_ROOT_LIBS}/libTAO_PortableServer.so
                    ${PARENT_DDS_ROOT_LIBS}/libTAO_PortableServer.so
                    ${PARENT_DDS_ROOT_LIBS}/libTAO_PI.so
                    ${PARENT_DDS_ROOT_LIBS}/libTAO_CodecFactory.so
                    ${PARENT_DDS_ROOT_LIBS}/libTAO_AnyTypeCode.so
                    ${PARENT_DDS_ROOT_LIBS}/libTAO_BiDirGIOP.so)
  list(APPEND IDLGEN_LIBRARIES opendds_idl ${DDS_DYN_LIBS})
endif()

if(PERFORMANCE_TEST_RCLCPP_ENABLED OR PERFORMANCE_TEST_APEX_OS_POLLING_SUBSCRIPTION_ENABLED)
  add_subdirectory(rclcpp)
  set(ROSIDL_GEN_LIST ${ROSIDL_GEN_LIST} PARENT_SCOPE)
endif()

# iceoryx has no message generator, so performance_test uses ROS 2's
if(PERFORMANCE_TEST_ICEORYX_ENABLED)
  add_subdirectory(rclcpp)
  set(ROSIDL_GEN_LIST ${ROSIDL_GEN_LIST} PARENT_SCOPE)
endif()

set(IDLGEN_INCLUDE_DIR ${IDLGEN_INCLUDE_DIR} PARENT_SCOPE)
set(IDLGEN_LIBRARIES ${IDLGEN_LIBRARIES} PARENT_SCOPE)
