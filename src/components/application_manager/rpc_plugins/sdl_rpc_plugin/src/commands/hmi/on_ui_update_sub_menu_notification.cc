/*
 Copyright (c) 2019, Ford Motor Company, Livio
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following
 disclaimer in the documentation and/or other materials provided with the
 distribution.

 Neither the name of the the copyright holders nor the names of their
 contributors may be used to endorse or promote products derived from this
 software without specific prior written permission.

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

#include "sdl_rpc_plugin/commands/hmi/on_ui_update_sub_menu_notification.h"
#include "application_manager/application_impl.h"
#include "application_manager/rpc_service.h"
#include "interfaces/MOBILE_API.h"

namespace sdl_rpc_plugin {
using namespace application_manager;
namespace commands {

OnUIUpdateSubMenuNotification::OnUIUpdateSubMenuNotification(
    const application_manager::commands::MessageSharedPtr& message,
    ApplicationManager& application_manager,
    app_mngr::rpc_service::RPCService& rpc_service,
    app_mngr::HMICapabilities& hmi_capabilities,
    policy::PolicyHandlerInterface& policy_handler)
    : NotificationFromHMI(message,
                         application_manager,
                         rpc_service,
                         hmi_capabilities,
                         policy_handler) {}

OnUIUpdateSubMenuNotification::~OnUIUpdateSubMenuNotification() {}

void OnUIUpdateSubMenuNotification::Run() {
  LOG4CXX_AUTO_TRACE(logger_);
    // Prepare SmartObject for mobile factory
  (*message_)[strings::params][strings::function_id] = static_cast<int32_t>(
      mobile_apis::FunctionID::OnUpdateFileID);

  const auto app_id =
      (*message_)[strings::msg_params][strings::app_id].asUInt();
  auto app = application_manager_.application(app_id);

  if (!app) {
    LOG4CXX_ERROR(logger_, "Application with app_id " << app_id << " is not registered");
    return;
  }

  // Remove app_id from notification to mobile
  (*message_)[strings::params][strings::connection_key] =
      (*message_)[strings::msg_params][strings::app_id];
  (*message_)[strings::msg_params].erase(strings::app_id);

  SendNotificationToMobile(message_);
}

}  // namespace commands
}  // namespace app_service_rpc_plugin
