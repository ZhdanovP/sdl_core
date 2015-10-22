/*
 Copyright (c) 2013, Ford Motor Company
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

#include "can_cooperation/validators/get_interior_vehicle_data_response_validator.h"
#include "can_cooperation/validators/struct_validators/module_data_validator.h"
#include "can_cooperation/can_module_constants.h"
#include "can_cooperation/message_helper.h"

namespace can_cooperation {

namespace validators {

CREATE_LOGGERPTR_GLOBAL(logger_, "GetInteriorVehicleDataResponseValidator")

using namespace message_params;

GetInteriorVehicleDataResponseValidator::
GetInteriorVehicleDataResponseValidator() {
  // name="moduleType"
  is_subscribed_[ValidationParams::TYPE] = ValueType::BOOL;
  is_subscribed_[ValidationParams::ARRAY] = 0;
  is_subscribed_[ValidationParams::MANDATORY] = 0;

  validation_scope_map_[kIsSubscribed] = &is_subscribed_;
};

ValidationResult GetInteriorVehicleDataResponseValidator::Validate(
                                                   const Json::Value& json,
                                                   Json::Value& outgoing_json) {
  LOG4CXX_AUTO_TRACE(logger_);

  ValidationResult result = ValidateSimpleValues(json, outgoing_json);

  if (result != ValidationResult::SUCCESS) {
    return result;
  }

  if (IsMember(json, kModuleData)) {
    result = ModuleDataValidator::instance()->
        Validate(json[kModuleData], outgoing_json[kModuleData]);
  } else {
    result = ValidationResult::INVALID_DATA;
    LOG4CXX_ERROR(logger_, "Mandatory param " <<kModuleData <<" missing!" );
  }

  return result;
}

}  // namespace valdiators

}  // namespace can_cooperation

