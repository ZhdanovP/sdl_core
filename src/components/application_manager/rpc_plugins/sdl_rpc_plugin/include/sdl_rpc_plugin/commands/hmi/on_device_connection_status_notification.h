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

#ifndef SRC_COMPONENTS_APPLICATION_MANAGER_INCLUDE_APPLICATION_MANAGER_COMMANDS_HMI_ON_DEVICE_CONNECTION_STATUS_NOTIFICATION_H_
#define SRC_COMPONENTS_APPLICATION_MANAGER_INCLUDE_APPLICATION_MANAGER_COMMANDS_HMI_ON_DEVICE_CONNECTION_STATUS_NOTIFICATION_H_

#include <string>

#include "application_manager/commands/notification_from_hmi.h"
#include "application_manager/application_manager.h"

namespace sdl_rpc_plugin {
namespace app_mngr = application_manager;

namespace commands {
/**
 * @brief OnDeviceConnectionStatusNotification command class
 * Store the value of <usbTransportStatus> of the message from HMI to
 * "usb_transport_status" param of "device_data" section at PolicyTable
 **/
class OnDeviceConnectionStatusNotification
    : public app_mngr::commands::NotificationFromHMI {
 public:
  /**
   * @brief OnDeviceConnectionStatusNotification class constructor
   * @param message Incoming SmartObject message
   **/
  OnDeviceConnectionStatusNotification(
      const app_mngr::commands::MessageSharedPtr& message,
      app_mngr::ApplicationManager& application_manager,
      app_mngr::rpc_service::RPCService& rpc_service,
      app_mngr::HMICapabilities& hmi_capabilities,
      policy::PolicyHandlerInterface& policy_handle);

  /**
   * @brief OnDeviceChosenNotification class destructor
   **/
  virtual ~OnDeviceConnectionStatusNotification();

  /**
   * @brief Execute command
   **/
  void Run() OVERRIDE;

 private:
  /**
   * @brief Reads the mandatory parameters On successfull
   * @param device_data Corresponding to <device> parameter DeviceInfo
   * structure.
   * @param out_device_id On success the "id" of the device is stored here.
   * @param out_usb_transport_status On success the "usb_transport_status"
   * of the device is stored here.
   * @return True if the mandatory parameters are valid.
   */
  bool GetMandatoryParameters(
      const smart_objects::SmartObject& device_data,
      std::string& out_device_id,
      hmi_apis::Common_UserSetting::eType& out_usb_transport_status) const;

  /**
   * @brief Reads the string value corresponding to key from device_data
   * @param device_data Corresponding to <device> parameter DeviceInfo
   * structure.
   * @param key Key used to read the corresponding string value from device_data
   * @param out_value Corresponding to key string value from device_data
   * @param True if the out_value is valid
   */
  bool GetKeyStringValue(const smart_objects::SmartObject& device_data,
                         const std::string& key,
                         std::string& out_value) const;

  /**
   * @brief Reads the int32_t value corresponding to key from device_data
   * @param device_data Corresponding to <device> parameter DeviceInfo
   * structure.
   * @param key Key used to read the corresponding string value from device_data
   * @param out_value Corresponding to key int32_t value from device_data
   * @param True if the out_value is valid
   */
  bool GetKeyIntValue(const smart_objects::SmartObject& device_data,
                      const std::string& key,
                      int32_t& out_value) const;

  /**
   * @brief Converts integer to boolean after
   * comparison to hmi_apis::Common_UserSetting
   * @param user_setting_value Integer value to convert.
   * @return True if value corresponds to
   * valid hmi_apis::Common_UserSetting enum.
   */
  bool IsUserSettingValid(const int32_t user_setting_value) const;

  DISALLOW_COPY_AND_ASSIGN(OnDeviceConnectionStatusNotification);
};
}  // namespace commands
}  // namespace application_manager

#endif  // SRC_COMPONENTS_APPLICATION_MANAGER_INCLUDE_APPLICATION_MANAGER_COMMANDS_HMI_ON_DEVICE_CONNECTION_STATUS_NOTIFICATION_H_
