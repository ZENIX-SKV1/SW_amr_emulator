// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
#include "vda5050++/handler/map_action_handler.h"

#include "vda5050++/core/common/exception.h"
#include "vda5050++/core/instance.h"
#include "vda5050++/misc/action_declarations.h"

vda5050pp::handler::MapActionHandler::MapActionHandler()
    : SimpleMultiActionHandler({
          vda5050pp::misc::action_declarations::k_enable_map,
          vda5050pp::misc::action_declarations::k_download_map,
          vda5050pp::misc::action_declarations::k_delete_map,
      }) {}

void vda5050pp::handler::MapActionHandler::stateAddMap(const vda5050::Map &map) const {
  auto evt = std::make_shared<vda5050pp::events::AddMap>();
  evt->map = map;

  vda5050pp::core::Instance::ref().getStatusEventManager().dispatch(evt);
}

void vda5050pp::handler::MapActionHandler::stateEnableMap(std::string_view map_id,
                                                          std::string_view map_version) const {
  auto evt = std::make_shared<vda5050pp::events::EnableMap>();
  evt->map_id = map_id;
  evt->map_version = map_version;

  vda5050pp::core::Instance::ref().getStatusEventManager().dispatch(evt);
}
void vda5050pp::handler::MapActionHandler::stateDeleteMap(std::string_view map_id,
                                                          std::string_view map_version) const {
  auto evt = std::make_shared<vda5050pp::events::DeleteMap>();
  evt->map_id = map_id;
  evt->map_version = map_version;

  vda5050pp::core::Instance::ref().getStatusEventManager().dispatch(evt);
}

vda5050pp::handler::ActionCallbacks vda5050pp::handler::MapActionHandler::prepare(
    std::shared_ptr<vda5050pp::handler::ActionState>,
    std::shared_ptr<ParametersMap> parameters) noexcept(false) {
  vda5050pp::handler::ActionCallbacks callbacks;

  callbacks.on_cancel = [](auto &) { /* NOP */ };
  callbacks.on_pause = [](auto &) { /* NOP */ };
  callbacks.on_resume = [](auto &) { /* NOP */ };
  callbacks.on_start = [parameters, this](ActionState &action_state) {
    std::list<vda5050::Error> result;

    if (action_state.getAction().actionType == "enableMap") {
      result = this->enableMap(std::get<std::string>(parameters->at("mapId")),
                               std::get<std::string>(parameters->at("mapVersion")));
    } else if (action_state.getAction().actionType == "downloadMap") {
      result = this->downloadMap(
          std::get<std::string>(parameters->at("mapId")),
          std::get<std::string>(parameters->at("mapVersion")),
          std::get<std::string>(parameters->at("mapDownloadLink")),
          parameters->find("mapHash") != parameters->end()
              ? std::make_optional(std::get<std::string>(parameters->at("mapHash")))
              : std::nullopt);
    } else if (action_state.getAction().actionType == "deleteMap") {
      result = this->deleteMap(std::get<std::string>(parameters->at("mapId")),
                               std::get<std::string>(parameters->at("mapVersion")));
    } else {
      throw vda5050pp::VDA5050PPInvalidArgument(MK_EX_CONTEXT("Unknown action state"));
    }

    if (result.empty()) {
      action_state.setFinished();
    } else {
      action_state.setFailed(result);
    }
  };

  return callbacks;
}

void vda5050pp::handler::MapActionHandler::reset() {
  /* NOP, because nothing meaningful can be reset here */
}