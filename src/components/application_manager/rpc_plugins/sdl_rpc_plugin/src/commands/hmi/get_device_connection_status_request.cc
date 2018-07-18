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

#include "sdl_rpc_plugin/commands/hmi/get_device_connection_status_request.h"

#include <algorithm>

#include "interfaces/HMI_API.h"
#include "connection_handler/connection_handler.h"
#include "protocol_handler/session_observer.h"
#include "application_manager/message_helper.h"
#include "utils/helpers.h"

namespace sdl_rpc_plugin {
using namespace application_manager;
namespace commands {

/**
 * @brief Functor used to check the
 * availability of invalid characters
 * in string as defined in APPLINK-28280
 */
struct InvalidCharacterChecker {
  bool operator()(const char c) const {
    using namespace helpers;
    return Compare<char, EQ, ONE>(c, '\n', '\t', ' ');
  }
};

CREATE_LOGGERPTR_GLOBAL(logger_, "DeviceDataRetriever")

void DeviceDataRetriever::operator()(const std::string& device_id) {
  FillDataForDeviceAtIndex(device_id, index_++);
}

void DeviceDataRetriever::FillDataForDeviceAtIndex(const std::string& device_id,
                                                   size_t index) {
  std::string connection_type;
  application_manager_.GetPolicyHandler().GetDeviceConnectionType(
      device_id, connection_type);
  std::string device_name;
  GetDeviceName(device_id, device_name);

  if (!message_[index].isValid()) {
    message_[index] = smart_objects::SmartObject(smart_objects::SmartType_Map);
  }
  smart_objects::SmartObject& current_device = message_[index];
  current_device[strings::transport_type] =
      application_manager_.GetDeviceTransportType(connection_type);
  const hmi_apis::Common_UserSetting::eType usb_transport_status =
      application_manager_.GetPolicyHandler().GetDeviceUSBTransportStatus(
          device_id);

  current_device[strings::name] = device_name;
  current_device[strings::id] = device_id;
  current_device[strings::usb_transport_status] = usb_transport_status;
}

bool DeviceDataRetriever::GetDeviceName(const std::string& device_id,
                                        std::string& out_device_name) {
  LOG4CXX_AUTO_TRACE(logger_);
  connection_handler::DeviceHandle device_handle;
  connection_handler::ConnectionHandler& connection_handler =
      application_manager_.connection_handler();
  if (!connection_handler.GetDeviceID(device_id, &device_handle)) {
    LOG4CXX_WARN(logger_,
                 "Failed to extract device handle for device: " << device_id);
    return false;
  }

  const protocol_handler::SessionObserver& session_observer =
      connection_handler.get_session_observer();
  const int32_t invalid_result = -1;
  if (invalid_result ==
      session_observer.GetDataOnDeviceID(
          device_handle, &out_device_name, NULL, NULL, NULL)) {
    LOG4CXX_WARN(logger_,
                 "Failed to extract device name for device: "
                     << device_id << " with handle: " << device_handle);
    return false;
  }

  return true;
}

GetDeviceConnectionStatusRequest::GetDeviceConnectionStatusRequest(
    const application_manager::commands::MessageSharedPtr& message,
    ApplicationManager& application_manager,
    rpc_service::RPCService& rpc_service,
    HMICapabilities& hmi_capabilities,
    policy::PolicyHandlerInterface& policy_handler)
    : RequestFromHMI(message,
                     application_manager,
                     rpc_service,
                     hmi_capabilities,
                     policy_handler) {}

GetDeviceConnectionStatusRequest::~GetDeviceConnectionStatusRequest() {}

void GetDeviceConnectionStatusRequest::Run() {
  LOG4CXX_AUTO_TRACE(logger_);

  smart_objects::SmartObject& object = *message_;
  smart_objects::SmartObject& msg_params_so = object[strings::msg_params];

  if (msg_params_so.keyExists(strings::device) &&
      !msg_params_so[strings::device].empty()) {
    smart_objects::SmartObject& devices = msg_params_so[strings::device];
    DeviceDataRetriever device_data_retriever(devices, application_manager_);
    for (size_t i = 0; i < devices.length(); ++i) {
      smart_objects::SmartObject& current_device = devices[i];

      std::string device_id;
      if (!GetMandatoryParameters(current_device, device_id)) {
        LOG4CXX_WARN(logger_,
                     "Invalid data for device: " << device_id << " found");
        SendErrorResponse(
            correlation_id(),
            static_cast<hmi_apis::FunctionID::eType>(function_id()),
            hmi_apis::Common_Result::INVALID_DATA,
            "");
        return;
      }

      device_data_retriever.FillDataForDeviceAtIndex(device_id, i);
    }
  } else {
    const std::vector<std::string> devices_ids =
        application_manager_.GetPolicyHandler().GetDevicesIDs();
    smart_objects::SmartObject& devices = msg_params_so[strings::device];
    devices = smart_objects::SmartObject(smart_objects::SmartType_Array);
    DeviceDataRetriever device_data_retriever(devices, application_manager_);
    std::for_each(
        devices_ids.begin(), devices_ids.end(), device_data_retriever);
  }

  object[strings::params][hmi_response::code] =
      hmi_apis::Common_Result::SUCCESS;
  application_manager_.GetRPCService().ManageHMICommand(message_);
}

bool GetDeviceConnectionStatusRequest::GetMandatoryParameters(
    const smart_objects::SmartObject& device_data,
    std::string& out_device_id) const {
  LOG4CXX_AUTO_TRACE(logger_);
  std::string device_name;
  if (!(GetKeyStringValue(device_data, strings::id, out_device_id) &&
        GetKeyStringValue(device_data, strings::name, device_name))) {
    LOG4CXX_WARN(logger_, "Invalid data for key value(s)");
    return false;
  }
  return true;
}

bool GetDeviceConnectionStatusRequest::GetKeyStringValue(
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
  if (IsSyntaxWrong(out_value)) {
    LOG4CXX_WARN(logger_,
                 "Invalid data: Value of " << key
                                           << " has invalid characters only.");
    return false;
  }
  return true;
}

bool GetDeviceConnectionStatusRequest::IsSyntaxWrong(
    const std::string& str) const {
  InvalidCharacterChecker checker;
  const std::string::size_type invalid_character_count =
      std::count_if(str.begin(), str.end(), checker);
  return str.size() == invalid_character_count;
}

}  // namespace commands
}  // namespace application_manager
