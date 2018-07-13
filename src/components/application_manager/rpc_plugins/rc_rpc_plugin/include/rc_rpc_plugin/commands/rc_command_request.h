/*
 Copyright (c) 2018, Ford Motor Company
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following
 disclaimer in the documentation and/or other materials provided with the
 distribution.

 Neither the name of the Ford Motor Company nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SRC_COMPONENTS_APPLICATION_MANAGER_RPC_PLUGINS_RC_RPC_PLUGIN_INCLUDE_RC_RPC_PLUGIN_COMMANDS_RC_COMMAND_REQUEST_H_
#define SRC_COMPONENTS_APPLICATION_MANAGER_RPC_PLUGINS_RC_RPC_PLUGIN_INCLUDE_RC_RPC_PLUGIN_COMMANDS_RC_COMMAND_REQUEST_H_

#include "rc_rpc_plugin/resource_allocation_manager.h"
#include "rc_rpc_plugin/rc_app_extension.h"
#include "application_manager/commands/command_request_impl.h"
#include "rc_rpc_plugin/interior_data_cache.h"
#include "rc_rpc_plugin/commands/rc_command_params.h"

namespace rc_rpc_plugin {
namespace app_mngr = application_manager;

enum TypeAccess { kDisallowed, kAllowed };

namespace commands {

class RCCommandRequest : public app_mngr::commands::CommandRequestImpl {
 public:
  /**
   * @brief RCCommandRequest class constructor
   * @param message MessageSharedPtr
   * @param params RCCommandParams
   **/
  RCCommandRequest(
      const application_manager::commands::MessageSharedPtr& message,
      const RCCommandParams& params);

  /**
   * @brief destructor of RCCommandRequest
   */
  virtual ~RCCommandRequest();

  /**
   * @brief Send to mobile GENERIC_ERROR when reques timeout expired
   */
  void onTimeOut() OVERRIDE;

  /**
   * @brief Run the command request
   */
  void Run() OVERRIDE;

  /**
   * @brief Run the command request
   */
  virtual void on_event(const app_mngr::event_engine::Event& event) OVERRIDE;

 protected:
  bool is_subscribed;
  bool auto_allowed_;

  ResourceAllocationManager& resource_allocation_manager_;
  InteriorDataCache& interior_data_cache_;

  /**
   * @brief AcquireResource try to allocate resource for application
   * In case if allocation of resource is not required, return ALLOWED by
   * default.
   * This method should be overrided in RPCs that requires resource allocation
   * @return result of resource allocation, in case if allocation os not
   * required, return ALLOWED
   */
  virtual AcquireResult::eType AcquireResource(
      const app_mngr::commands::MessageSharedPtr& message) {
    return AcquireResult::ALLOWED;
  }

  /**
   * @brief IsResourceFree check resource state
   * This is default implementation which has to be redefined for RPCs which
   * need to manage the resources
   * @param module_type Resource name
   * @return True if free, otherwise - false
   */
  virtual bool IsResourceFree(const std::string& module_type) const {
    UNUSED(module_type);
    return true;
  }
  /**
   * @brief SetResourceState changes state of resource
   * This is default implementation which has to be redefined for RPCs which
   * need to manage the resources
   * @param module_type Resource name
   * @param State to set for resource
   */
  virtual void SetResourceState(const std::string& module_type,
                                const ResourceState::eType) {}

  /**
   * @brief Checks if module for application is present in policy table
   * @param module type Resource name
   * @param app_id id of application
   * @return kAllowed if module is present, otherwise - kDisallowed
   */
  TypeAccess CheckModule(const std::string& module_type,
                         application_manager::ApplicationSharedPtr app);

  /**
   * @brief Is auto is allowed
   * @return bool
   */
  bool auto_allowed() const {
    return auto_allowed_;
  }

  /**
   * @brief Set Is auto is allowed
   * @param bool
   */
  void set_auto_allowed(const bool value) {
    auto_allowed_ = value;
  }

  /**
   * @brief executes specific logic of children classes
   */
  void virtual Execute() = 0;

  /**
   * @brief Set the disalowed info
   * @param disallowed info
   */
  void set_disallowed_info(const std::string& info) {
    disallowed_info_ = info;
  }

  /**
   * @brief Get the module type
   * @return std::string
   */
  virtual std::string ModuleType() = 0;

 private:
  /**
   * @brief CheckDriverConsent checks driver consent defined in policy table
   * @return True if no consent is required, otherwise - false
   */
  bool CheckDriverConsent();

  /**
   * @brief AcquireResources checks whether resource status is busy or not and
   * then tries to acquire this resource. In case driver consent is required -
   * sends consent request to HMI.
   * @return True in case of resource is free and successfully acquired,
   * otherwise false
   */
  bool AcquireResources();

  /**
   * @brief Send responce DISALLOWED to mobile
   * @param access TypeAccess
   */
  void SendDisallowed(TypeAccess access);

  /**
   * @brief SendGetUserConsent sends consent request to HMI
   * @param module_type Resource name
   */
  void SendGetUserConsent(const std::string& module_type);

  /**
   * @brief Processing the access responce
   * @param event event_engine::Even
   */
  void ProcessAccessResponse(const app_mngr::event_engine::Event& event);

  /**
   * @brief Check if interface is available
   * @param bool
   */
  bool IsInterfaceAvailable(
      const app_mngr::HmiInterfaces::InterfaceID interface) const;

  std::string disallowed_info_;
};
}
}

#endif  // SRC_COMPONENTS_APPLICATION_MANAGER_RPC_PLUGINS_RC_RPC_PLUGIN_INCLUDE_RC_RPC_PLUGIN_COMMANDS_RC_COMMAND_REQUEST_H_
