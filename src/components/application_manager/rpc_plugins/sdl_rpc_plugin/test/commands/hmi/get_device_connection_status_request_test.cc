/*
 * Copyright (c) 2018, Ford Motor Company
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of the Ford Motor Company nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <string>
#include <memory>
#include "gtest/gtest.h"
#include "application_manager/smart_object_keys.h"
#include "application_manager/commands/command.h"
#include "application_manager/mock_application_manager.h"
#include "application_manager/event_engine/event_dispatcher.h"
#include "application_manager/commands/commands_test.h"
#include "application_manager/commands/command_request_test.h"
#include "application_manager/policies/mock_policy_handler_interface.h"
#include "policy/mock_policy_manager.h"
#include "protocol_handler/mock_session_observer.h"
#include "connection_handler/mock_connection_handler.h"
#include "hmi/get_device_connection_status_request.h"
#include "json/json.h"
#include "formatters/CFormatterJsonBase.h"
#include "interfaces/HMI_API.h"
#include "utils/shared_ptr.h"
#include "utils/make_shared.h"
#include "application_manager/rpc_service.h"

namespace test {
namespace components {
namespace commands_test {
namespace hmi_commands_test {
namespace get_device_connection_status_request {

using testing::_;
using ::testing::Mock;
using ::testing::Return;
using ::testing::ReturnRef;
using ::testing::NiceMock;
using ::testing::SetArgReferee;
using ::testing::SetArgPointee;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::Mock;

namespace am = ::application_manager;
namespace strings = am::strings;
using test::components::commands_test::HMIResultCodeIs;
using NsSmartDeviceLink::NsSmartObjects::SmartType;
using sdl_rpc_plugin::commands::GetDeviceConnectionStatusRequest;

typedef SharedPtr<GetDeviceConnectionStatusRequest> CommandPtr;

namespace {
const std::string kDeviceId1 = "device_id_1";
const std::string kDeviceId2 = "device_id_2";
const std::string kDeviceName1 = "device_name_1";
const std::string kDeviceName2 = "device_name_2";
const connection_handler::DeviceHandle kDeviceHandle1 = 0u;
const connection_handler::DeviceHandle kDeviceHandle2 = 1u;

const std::string kMacAddress1 = "test_mac_address1";
const std::string kMacAddress2 = "test_mac_address2";

const std::string kConnectionType = "USB_AOA";
const hmi_apis::Common_TransportType::eType kTransportType =
    hmi_apis::Common_TransportType::USB_AOA;

const int32_t kTransportTypeIntValue = static_cast<int32_t>(kTransportType);

const hmi_apis::Common_UserSetting::eType kUsbTransportEnabled =
    hmi_apis::Common_UserSetting::ENABLED;
const hmi_apis::Common_UserSetting::eType kUsbTransportDisabled =
    hmi_apis::Common_UserSetting::DISABLED;

const std::string kInvalidSyntaxString = "\n\t ";
}  // namespace

class GetDeviceConnectionStatusRequestTest
    : public CommandsTest<CommandsTestMocks::kIsNice> {
 protected:
  void SetUp() OVERRIDE {
    ON_CALL(app_mngr_, GetPolicyHandler())
        .WillByDefault(ReturnRef(mock_policy_handler_));
    mock_policy_manager_ =
        utils::MakeShared<policy_manager_test::MockPolicyManager>();
    ASSERT_TRUE(mock_policy_manager_);
    ON_CALL(app_mngr_, event_dispatcher())
        .WillByDefault(ReturnRef(mock_event_dispatcher_));

    command_msg_ = CreateMessage(SmartType::SmartType_Map);
  }

  void SetRequestExpectations() {
    EXPECT_CALL(app_mngr_, GetPolicyHandler())
        .WillRepeatedly(ReturnRef(mock_policy_handler_));
    EXPECT_CALL(app_mngr_, connection_handler())
        .WillRepeatedly(ReturnRef(mock_connection_handler_));
    EXPECT_CALL(mock_connection_handler_, get_session_observer())
        .WillRepeatedly(ReturnRef(mock_session_observer_));
    EXPECT_CALL(app_mngr_, GetRPCService())
        .WillRepeatedly(ReturnRef(mock_rpc_service_));
  }

  void SetSuccessfulDataRetrievalExpectations() {
    EXPECT_CALL(mock_policy_handler_, GetDeviceUSBTransportStatus(kDeviceId1))
        .WillOnce(Return(kUsbTransportEnabled));
    EXPECT_CALL(mock_policy_handler_, GetDeviceUSBTransportStatus(kDeviceId2))
        .WillOnce(Return(kUsbTransportDisabled));

    EXPECT_CALL(mock_policy_handler_, GetDeviceConnectionType(kDeviceId1, _))
        .WillOnce(DoAll(SetArgReferee<1>(kConnectionType), Return(true)));
    EXPECT_CALL(mock_policy_handler_, GetDeviceConnectionType(kDeviceId2, _))
        .WillOnce(DoAll(SetArgReferee<1>(kConnectionType), Return(true)));

    EXPECT_CALL(app_mngr_, GetDeviceTransportType(kConnectionType))
        .WillRepeatedly(Return(kTransportType));
  }

  NiceMock<event_engine_test::MockEventDispatcher> mock_event_dispatcher_;
  SharedPtr<policy_manager_test::MockPolicyManager> mock_policy_manager_;
  connection_handler_test::MockConnectionHandler mock_connection_handler_;
  MessageSharedPtr command_msg_;
  connection_handler::DeviceHandle device_handle_;
  protocol_handler_test::MockSessionObserver mock_session_observer_;
  std::vector<std::string> list_devices_ids_;
};

TEST_F(GetDeviceConnectionStatusRequestTest,
       RUN_ValidRequest_SendRequestedDevicesInfo) {
  MessageSharedPtr command_msg_ = CreateMessage();
  SmartObject& devices = (*command_msg_)[strings::msg_params][strings::device];
  devices = SmartObject(SmartType::SmartType_Array);
  devices[0] = SmartObject(SmartType::SmartType_Map);
  devices[1] = SmartObject(SmartType::SmartType_Map);

  SmartObject& device_1 = devices[0];
  device_1[strings::id] = kDeviceId1;
  device_1[strings::name] = kDeviceName1;

  SmartObject& device_2 = devices[1];
  device_2[strings::id] = kDeviceId2;
  device_2[strings::name] = kDeviceName2;

  CommandPtr command(
      CreateCommand<GetDeviceConnectionStatusRequest>(command_msg_));

  SetRequestExpectations();

  EXPECT_CALL(mock_session_observer_,
              GetDataOnDeviceID(kDeviceHandle1, _, _, _, _))
      .WillOnce(DoAll(SetArgPointee<1u>(kDeviceName1), Return(0)));

  EXPECT_CALL(mock_session_observer_,
              GetDataOnDeviceID(kDeviceHandle2, _, _, _, _))
      .WillOnce(DoAll(SetArgPointee<1u>(kDeviceName2), Return(0)));

  EXPECT_CALL(mock_connection_handler_, GetDeviceID(kDeviceId1, _))
      .WillOnce(DoAll(SetArgPointee<1>(kDeviceHandle1), Return(true)));
  EXPECT_CALL(mock_connection_handler_, GetDeviceID(kDeviceId2, _))
      .WillOnce(DoAll(SetArgPointee<1>(kDeviceHandle2), Return(true)));

  SetSuccessfulDataRetrievalExpectations();

  command->Run();

  // After command runnning, [devices] was rewritten and ref 'devices' not valid
  SmartObject& new_devices =
      (*command_msg_)[strings::msg_params][strings::device];

  EXPECT_EQ(2u, new_devices.length());

  EXPECT_EQ(kTransportTypeIntValue,
            new_devices[0][strings::transport_type].asInt());
  EXPECT_EQ(kUsbTransportEnabled,
            static_cast<hmi_apis::Common_UserSetting::eType>(
                new_devices[0][strings::usb_transport_status].asInt()));

  EXPECT_EQ(kTransportTypeIntValue,
            new_devices[1][strings::transport_type].asInt());
  EXPECT_EQ(kUsbTransportDisabled,
            static_cast<hmi_apis::Common_UserSetting::eType>(
                new_devices[1][strings::usb_transport_status].asInt()));
}

MATCHER_P(HMIRequestResultCodeIs, result_code, "") {
  return result_code ==
         static_cast<hmi_apis::Common_Result::eType>(
             (*arg)[strings::params][am::hmi_response::code].asInt());
}

TEST_F(
    GetDeviceConnectionStatusRequestTest,
    RUN_InvalidRequestInvalidSymbolsStringMandatoryParam_SendInvalidDataErrorResponse) {
  SmartObject& devices = (*command_msg_)[strings::msg_params][strings::device];
  devices = SmartObject(SmartType::SmartType_Array);
  devices[0] = SmartObject(SmartType::SmartType_Map);

  SmartObject& device_1 = devices[0];
  device_1[strings::id] = kInvalidSyntaxString;
  device_1[strings::name] = kDeviceName1;

  CommandPtr command(
      CreateCommand<GetDeviceConnectionStatusRequest>(command_msg_));

  EXPECT_CALL(app_mngr_, GetPolicyHandler())
      .WillRepeatedly(ReturnRef(mock_policy_handler_));
  EXPECT_CALL(app_mngr_, GetRPCService())
      .WillRepeatedly(ReturnRef(mock_rpc_service_));

  EXPECT_CALL(mock_policy_handler_, GetDeviceUSBTransportStatus(_)).Times(0);
  EXPECT_CALL(mock_policy_handler_, GetDeviceConnectionType(_, _)).Times(0);
  EXPECT_CALL(app_mngr_, GetDeviceTransportType(_)).Times(0);

  EXPECT_CALL(mock_rpc_service_,
              ManageHMICommand(HMIRequestResultCodeIs(
                  hmi_apis::Common_Result::INVALID_DATA)))
      .WillOnce(Return(true));

  command->Run();
}

TEST_F(GetDeviceConnectionStatusRequestTest,
       RUN_EmptyRequest_SendAllDevicesInfo) {
  list_devices_ids_.push_back(kDeviceId1);
  list_devices_ids_.push_back(kDeviceId2);

  CommandPtr command(
      CreateCommand<GetDeviceConnectionStatusRequest>(command_msg_));

  SetRequestExpectations();

  EXPECT_CALL(mock_policy_handler_, GetDevicesIDs())
      .WillOnce(Return(list_devices_ids_));

  EXPECT_CALL(mock_session_observer_,
              GetDataOnDeviceID(kDeviceHandle1, _, _, _, _))
      .WillOnce(DoAll(SetArgPointee<1u>(kDeviceName1), Return(0)));

  EXPECT_CALL(mock_session_observer_,
              GetDataOnDeviceID(kDeviceHandle2, _, _, _, _))
      .WillOnce(DoAll(SetArgPointee<1u>(kDeviceName2), Return(0)));

  EXPECT_CALL(mock_connection_handler_, GetDeviceID(kDeviceId1, _))
      .WillOnce(DoAll(SetArgPointee<1>(kDeviceHandle1), Return(true)));
  EXPECT_CALL(mock_connection_handler_, GetDeviceID(kDeviceId2, _))
      .WillOnce(DoAll(SetArgPointee<1>(kDeviceHandle2), Return(true)));

  SetSuccessfulDataRetrievalExpectations();

  command->Run();

  const SmartObject& msg_params_so = (*command_msg_)[strings::msg_params];
  ASSERT_TRUE(msg_params_so.keyExists(strings::device));

  const SmartObject& devices =
      (*command_msg_)[strings::msg_params][strings::device];
  EXPECT_EQ(2u, devices.length());

  const SmartObject& device_1 = devices[0];
  const SmartObject& device_2 = devices[1];

  EXPECT_EQ(kDeviceId1, device_1[strings::id].asString());
  EXPECT_EQ(kDeviceName1, device_1[strings::name].asString());
  EXPECT_EQ(kTransportTypeIntValue, device_1[strings::transport_type].asInt());
  EXPECT_EQ(kUsbTransportEnabled,
            static_cast<hmi_apis::Common_UserSetting::eType>(
                device_1[strings::usb_transport_status].asInt()));

  EXPECT_EQ(kDeviceId2, device_2[strings::id].asString());
  EXPECT_EQ(kDeviceName2, device_2[strings::name].asString());
  EXPECT_EQ(kTransportTypeIntValue, device_2[strings::transport_type].asInt());
  EXPECT_EQ(kUsbTransportDisabled,
            static_cast<hmi_apis::Common_UserSetting::eType>(
                device_2[strings::usb_transport_status].asInt()));
}

TEST_F(GetDeviceConnectionStatusRequestTest,
       RUN_EmptyRequestErrorGetDeviceHandle_SendAllDevicesInfo) {
  list_devices_ids_.push_back(kDeviceId1);
  list_devices_ids_.push_back(kDeviceId2);

  CommandPtr command(
      CreateCommand<GetDeviceConnectionStatusRequest>(command_msg_));

  SetRequestExpectations();

  EXPECT_CALL(mock_policy_handler_, GetDevicesIDs())
      .WillOnce(Return(list_devices_ids_));

  EXPECT_CALL(mock_session_observer_,
              GetDataOnDeviceID(kDeviceHandle1, _, _, _, _))
      .WillOnce(DoAll(SetArgPointee<1u>(kDeviceName1), Return(0)));

  EXPECT_CALL(mock_session_observer_,
              GetDataOnDeviceID(kDeviceHandle2, _, _, _, _))
      .WillOnce(DoAll(SetArgPointee<1u>(kDeviceName2), Return(0)));

  EXPECT_CALL(mock_connection_handler_, GetDeviceID(kDeviceId1, _))
      .WillOnce(DoAll(SetArgPointee<1>(kDeviceHandle1), Return(true)));
  EXPECT_CALL(mock_connection_handler_, GetDeviceID(kDeviceId2, _))
      .WillOnce(DoAll(SetArgPointee<1>(kDeviceHandle2), Return(true)));

  SetSuccessfulDataRetrievalExpectations();

  command->Run();

  const SmartObject& msg_params_so = (*command_msg_)[strings::msg_params];
  ASSERT_TRUE(msg_params_so.keyExists(strings::device));

  const SmartObject& devices =
      (*command_msg_)[strings::msg_params][strings::device];
  EXPECT_EQ(2u, devices.length());

  const SmartObject& device_1 = devices[0];
  const SmartObject& device_2 = devices[1];

  EXPECT_EQ(kDeviceId1, device_1[strings::id].asString());
  EXPECT_EQ(kDeviceName1, device_1[strings::name].asString());
  EXPECT_EQ(kTransportTypeIntValue, device_1[strings::transport_type].asInt());
  EXPECT_EQ(kUsbTransportEnabled,
            static_cast<hmi_apis::Common_UserSetting::eType>(
                device_1[strings::usb_transport_status].asInt()));

  EXPECT_EQ(kDeviceId2, device_2[strings::id].asString());
  EXPECT_EQ(kDeviceName2, device_2[strings::name].asString());
  EXPECT_EQ(kTransportTypeIntValue, device_2[strings::transport_type].asInt());
  EXPECT_EQ(kUsbTransportDisabled,
            static_cast<hmi_apis::Common_UserSetting::eType>(
                device_2[strings::usb_transport_status].asInt()));
}

TEST_F(GetDeviceConnectionStatusRequestTest,
       RUN_EmptyRequestErrorGetDataOnDeviceHandle_SendAllDevicesInfo) {
  list_devices_ids_.push_back(kDeviceId1);
  list_devices_ids_.push_back(kDeviceId2);

  CommandPtr command(
      CreateCommand<GetDeviceConnectionStatusRequest>(command_msg_));

  SetRequestExpectations();

  EXPECT_CALL(mock_policy_handler_, GetDevicesIDs())
      .WillOnce(Return(list_devices_ids_));

  EXPECT_CALL(mock_session_observer_,
              GetDataOnDeviceID(kDeviceHandle1, _, _, _, _))
      .WillOnce(DoAll(SetArgPointee<1u>(kDeviceName1), Return(0)));

  EXPECT_CALL(mock_session_observer_,
              GetDataOnDeviceID(kDeviceHandle2, _, _, _, _))
      .WillOnce(DoAll(SetArgPointee<1u>(kDeviceName2), Return(0)));

  EXPECT_CALL(mock_connection_handler_, GetDeviceID(kDeviceId1, _))
      .WillOnce(DoAll(SetArgPointee<1>(kDeviceHandle1), Return(true)));
  EXPECT_CALL(mock_connection_handler_, GetDeviceID(kDeviceId2, _))
      .WillOnce(DoAll(SetArgPointee<1>(kDeviceHandle2), Return(true)));

  SetSuccessfulDataRetrievalExpectations();

  command->Run();

  const SmartObject& msg_params_so = (*command_msg_)[strings::msg_params];
  ASSERT_TRUE(msg_params_so.keyExists(strings::device));

  const SmartObject& devices =
      (*command_msg_)[strings::msg_params][strings::device];
  EXPECT_EQ(2u, devices.length());

  const SmartObject& device_1 = devices[0];
  const SmartObject& device_2 = devices[1];

  EXPECT_EQ(kDeviceId1, device_1[strings::id].asString());
  EXPECT_EQ(kDeviceName1, device_1[strings::name].asString());
  EXPECT_EQ(kTransportTypeIntValue, device_1[strings::transport_type].asInt());
  EXPECT_EQ(kUsbTransportEnabled,
            static_cast<hmi_apis::Common_UserSetting::eType>(
                device_1[strings::usb_transport_status].asInt()));

  EXPECT_EQ(kDeviceId2, device_2[strings::id].asString());
  EXPECT_EQ(kDeviceName2, device_2[strings::name].asString());
  EXPECT_EQ(kTransportTypeIntValue, device_2[strings::transport_type].asInt());
  EXPECT_EQ(kUsbTransportDisabled,
            static_cast<hmi_apis::Common_UserSetting::eType>(
                device_2[strings::usb_transport_status].asInt()));
}

TEST_F(GetDeviceConnectionStatusRequestTest,
       RUN_EmptyRequestNoDevicesInPT_SendEmptyList) {
  CommandPtr command(
      CreateCommand<GetDeviceConnectionStatusRequest>(command_msg_));

  SetRequestExpectations();

  EXPECT_CALL(mock_policy_handler_, GetDevicesIDs())
      .WillOnce(Return(list_devices_ids_));

  EXPECT_CALL(mock_policy_handler_, GetDeviceUSBTransportStatus(_)).Times(0);

  EXPECT_CALL(mock_policy_handler_, GetDeviceConnectionType(_, _)).Times(0);
  EXPECT_CALL(app_mngr_, GetDeviceTransportType(_)).Times(0);

  command->Run();

  const SmartObject& msg_params_so = (*command_msg_)[strings::msg_params];
  ASSERT_TRUE(msg_params_so.keyExists(strings::device));

  const SmartObject& devices =
      (*command_msg_)[strings::msg_params][strings::device];
  EXPECT_EQ(0u, devices.length());
}

}  // namespace get_device_connection_status_request
}  // namespace hmi_commands_test
}  // namespace commands_test
}  // namespace components
}  // namespace test
