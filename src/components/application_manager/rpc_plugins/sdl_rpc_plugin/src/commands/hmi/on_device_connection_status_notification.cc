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

#include "sdl_rpc_plugin/commands/hmi/on_device_connection_status_notification.h"
#include "policy/policy_table/enums.h"

namespace sdl_rpc_plugin {
using namespace application_manager;

namespace commands {

OnDeviceConnectionStatusNotification::OnDeviceConnectionStatusNotification(
    const application_manager::commands::MessageSharedPtr& message,
    ApplicationManager& application_manager,
    rpc_service::RPCService& rpc_service,
    HMICapabilities& hmi_capabilities,
    policy::PolicyHandlerInterface& policy_handle)
    : NotificationFromHMI(message,
                          application_manager,
                          rpc_service,
                          hmi_capabilities,
                          policy_handle) {}

OnDeviceConnectionStatusNotification::~OnDeviceConnectionStatusNotification() {}

void OnDeviceConnectionStatusNotification::Run() {
  LOG4CXX_AUTO_TRACE(logger_);

  const smart_objects::SmartObject& msg_params =
      (*message_)[strings::msg_params];
  if (!msg_params.keyExists(strings::device)) {
    LOG4CXX_WARN(logger_,
                 "Invalid data: Key " << strings::device << " does not exist.");
    return;
  }
  const smart_objects::SmartObject& device_info_so =
      msg_params[strings::device];

  std::string device_id;
  hmi_apis::Common_UserSetting::eType usb_transport_status =
      hmi_apis::Common_UserSetting::INVALID_ENUM;
  if (!GetMandatoryParameters(
          device_info_so, device_id, usb_transport_status)) {
    LOG4CXX_WARN(logger_, "Error getting notification mandatory parameters");
    return;
  }

  application_manager_.GetPolicyHandler().OnDeviceConnectionStatus(
      device_id, usb_transport_status);
}

bool OnDeviceConnectionStatusNotification::GetMandatoryParameters(
    const smart_objects::SmartObject& device_data,
    std::string& out_device_id,
    hmi_apis::Common_UserSetting::eType& out_usb_transport_status) const {
  LOG4CXX_AUTO_TRACE(logger_);

  int32_t transport_type;
  int32_t usb_transport_status;

  bool all_key_is_valid = true;
  all_key_is_valid &=
      GetKeyStringValue(device_data, strings::id, out_device_id);
  all_key_is_valid &=
      GetKeyIntValue(device_data, strings::transport_type, transport_type);
  all_key_is_valid &= GetKeyIntValue(
      device_data, strings::usb_transport_status, usb_transport_status);

  if (!all_key_is_valid) {
    LOG4CXX_WARN(logger_, "Invalid data for key value(s)");
    return false;
  }

  const std::string& connection_type =
      application_manager_.GetDeviceConnectionType(transport_type);
  if (connection_type.empty()) {
    LOG4CXX_WARN(logger_, "Invalid data for connection type");
    return false;
  }

  if (!IsUserSettingValid(usb_transport_status)) {
    LOG4CXX_WARN(logger_, "Invalid data for usb_transport_status");
    return false;
  }
  out_usb_transport_status =
      static_cast<hmi_apis::Common_UserSetting::eType>(usb_transport_status);

  return true;
}

bool OnDeviceConnectionStatusNotification::GetKeyStringValue(
    const smart_objects::SmartObject& device_data,
    const std::string& key,
    std::string& out_value) const {
  LOG4CXX_AUTO_TRACE(logger_);

  if (!device_data.keyExists(key)) {
    LOG4CXX_WARN(logger_, "Invalid data: Key " << key << " does not exist.");
    return false;
  }

  out_value = device_data[key].asString();

  if (out_value.empty()) {
    LOG4CXX_WARN(logger_, "Invalid data: Key " << key << " empty.");
    return false;
  }
  return true;
}

bool OnDeviceConnectionStatusNotification::GetKeyIntValue(
    const smart_objects::SmartObject& device_data,
    const std::string& key,
    int32_t& out_value) const {
  LOG4CXX_AUTO_TRACE(logger_);
  if (!device_data.keyExists(key)) {
    LOG4CXX_WARN(logger_, "Invalid data: Key " << key << " does not exist.");
    return false;
  }
  out_value = device_data[key].asInt();
  return true;
}

bool OnDeviceConnectionStatusNotification::IsUserSettingValid(
    const int32_t user_setting_value) const {
  hmi_apis::Common_UserSetting::eType user_setting =
      static_cast<hmi_apis::Common_UserSetting::eType>(user_setting_value);
  switch (user_setting) {
    case hmi_apis::Common_UserSetting::ENABLED:
    case hmi_apis::Common_UserSetting::DISABLED:
      break;
    default:
      LOG4CXX_WARN(logger_,
                   "Invalid data: Invalid value of user_settings enum: "
                       << user_setting_value);
      return false;
  }
  return true;
}

}  // namespace commands
}  // namespace application_manager
