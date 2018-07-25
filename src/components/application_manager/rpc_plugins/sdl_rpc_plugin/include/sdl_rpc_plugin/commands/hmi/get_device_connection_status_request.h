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

#ifndef SRC_COMPONENTS_APPLICATION_MANAGER_RPC_PLUGINS_SDL_RPC_PLUGIN_INCLUDE_SDL_RPC_PLUGIN_COMMANDS_HMI_GET_DEVICE_CONNECTION_STATUS_REQUEST_H_
#define SRC_COMPONENTS_APPLICATION_MANAGER_RPC_PLUGINS_SDL_RPC_PLUGIN_INCLUDE_SDL_RPC_PLUGIN_COMMANDS_HMI_GET_DEVICE_CONNECTION_STATUS_REQUEST_H_

#include "application_manager/commands/request_from_hmi.h"

namespace sdl_rpc_plugin {
namespace app_mngr = application_manager;

namespace commands {

/**
 * @brief GetDeviceConnectionStatusRequest command class
 * Retrieve requested values at <device> param from
 * "device_data" section of PolicyTable.
 * If <device> param is empty all the <ID>, <name> of device,
 * <usb_transport_status>, <transport_type> params are retrieved
 * from "device_data" section of PolicyTable.
 **/
class GetDeviceConnectionStatusRequest
    : public app_mngr::commands::RequestFromHMI {
 public:
  /**
   * @brief GetDeviceConnectionStatusRequest class constructor
   * @param message Incoming SmartObject message
   **/
  GetDeviceConnectionStatusRequest(
      const app_mngr::commands::MessageSharedPtr& message,
      app_mngr::ApplicationManager& application_manager,
      app_mngr::rpc_service::RPCService& rpc_service,
      app_mngr::HMICapabilities& hmi_capabilities,
      policy::PolicyHandlerInterface& policy_handler);

  /**
   * @brief GetDeviceConnectionStatusRequest class destructor
   **/
  virtual ~GetDeviceConnectionStatusRequest();

  /**
   * @brief Execute command
   **/
  void Run() OVERRIDE;

 private:
  /**
   * @brief Reads the mandatory parameters
   * @param device_data Corresponding to <device> parameter DeviceInfo
   * structure.
   * @param out_device_id On success the "id" of the device is stored here.
   * @return True if the mandatory parameters are valid.
   */
  bool GetMandatoryParameters(const smart_objects::SmartObject& device_data,
                              std::string& out_device_id) const;

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
   * @brief Check whether the string contains invalid characters
   */
  bool IsSyntaxWrong(const std::string& str) const;

  /**
   * @brief Functor to retrieve all the data from the "device_data"
   * section of the policy table and store <ID>, <name> of device,
   * <usb_transport_status>, <transport_type> as array with key <device>
   * in the message
   */
  class DeviceDataRetriever {
   public:
    /**
     * @param message Message to store the retrieved data
     */
    DeviceDataRetriever(smart_objects::SmartObject& message,
                        app_mngr::ApplicationManager& application_manager)
        : message_(message)
        , application_manager_(application_manager)
        , index_(0) {}

    /**
     * @brief Retrieves the corresponding to data device_id
     * from Policy Table
     * @param device_id ID of the device in the Policy Table
     */
    void operator()(const std::string& device_id);

    /**
     * @brief Retrieves the corresponding to data device_id
     * from Policy Table and stores in the index of the
     * "device" section of the message
     * @param device_id ID of the device in the Policy Table
     * @param index Index position in "device" section of the message
     * to store data.
     */
    void FillDataForDeviceAtIndex(const std::string& device_id, const size_t index);

   private:
    /**
     * @brief Reads he corresponding to device_id
     * name of the device.
     * @param device_id ID of the device in the Policy Table
     * @param out_device_name On success the corresponding
     * to device_id name is stored here
     * @return True if the out_device_name is valid
     */
    bool GetDeviceName(const std::string& device_id,
                       std::string& out_device_name);

    smart_objects::SmartObject& message_;
    app_mngr::ApplicationManager& application_manager_;
    size_t index_;
  };

  DISALLOW_COPY_AND_ASSIGN(GetDeviceConnectionStatusRequest);
};

}  // namespace commands
}  // namespace application_manager

#endif  // SRC_COMPONENTS_APPLICATION_MANAGER_RPC_PLUGINS_SDL_RPC_PLUGIN_INCLUDE_SDL_RPC_PLUGIN_COMMANDS_HMI_GET_DEVICE_CONNECTION_STATUS_REQUEST_H_
