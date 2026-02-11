// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#include "vda5050++/handler/map_action_handler.h"

#include <catch2/catch_all.hpp>

#include "vda5050++/core/instance.h"

using namespace std::chrono_literals;

class TestMapHandler : public vda5050pp::handler::MapActionHandler {
private:
  std::promise<std::pair<std::string, std::string>> enable_promise_;
  std::promise<std::pair<std::string, std::string>> delete_promise_;
  std::promise<std::tuple<std::string, std::string, std::string>> download_promise_;

public:
  void resetPromise() {
    enable_promise_ = std::promise<std::pair<std::string, std::string>>();
    delete_promise_ = std::promise<std::pair<std::string, std::string>>();
    download_promise_ = std::promise<std::tuple<std::string, std::string, std::string>>();
  }

  std::future<std::pair<std::string, std::string>> getEnableFuture() {
    return enable_promise_.get_future();
  }
  std::future<std::pair<std::string, std::string>> getDeleteFuture() {
    return delete_promise_.get_future();
  }
  std::future<std::tuple<std::string, std::string, std::string>> getDownloadFuture() {
    return download_promise_.get_future();
  }

protected:
  std::list<vda5050::Error> enableMap(std::string_view map_id,
                                      std::string_view map_version) override {
    enable_promise_.set_value({std::string(map_id), std::string(map_version)});
    return {};
  }

  std::list<vda5050::Error> deleteMap(std::string_view map_id,
                                      std::string_view map_version) override {
    delete_promise_.set_value({std::string(map_id), std::string(map_version)});
    return {};
  }

  std::list<vda5050::Error> downloadMap(std::string_view map_id, std::string_view map_version,
                                        std::string_view map_download_link,
                                        std::optional<std::string_view> map_hash) override {
    download_promise_.set_value(
        {std::string(map_id), std::string(map_version), std::string(map_download_link)});
    return {};
  }
};

TEST_CASE("handler::MapActionHandler", "[handler]") {
  vda5050pp::core::events::EventManagerOptions evt_opts;
  evt_opts.synchronous_event_dispatch = false;  // TODO: make this succeed with true
  vda5050pp::Config cfg;
  cfg.refGlobalConfig().useWhiteList();
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_action_event_handler_key);
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_interpreter_event_handler_key);
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_order_event_handler_key);
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_message_event_handler_key);
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_state_event_handler_key);
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_validation_event_handler_key);
  cfg.refGlobalConfig().setLogLevel(vda5050pp::config::LogLevel::k_debug);
  vda5050pp::core::Instance::reset();
  auto instance = vda5050pp::core::Instance::init(cfg, evt_opts).lock();

  auto handler = std::make_shared<TestMapHandler>();
  instance->addActionHandler(handler);

  WHEN("An enable map instant action received") {
    handler->resetPromise();
    auto ia = std::make_shared<vda5050pp::core::events::ReceiveInstantActionMessageEvent>();
    ia->instant_actions = std::make_shared<vda5050::InstantActions>();
    ia->instant_actions->actions.push_back(
        vda5050::Action{"enableMap",
                        "1",
                        std::nullopt,
                        vda5050::BlockingType::NONE,
                        {{{"mapId", "Map1"}, {"mapVersion", "V1"}}}});

    instance->getMessageEventManager().synchronousDispatch(ia);
    auto future = handler->getEnableFuture();

    THEN("The enable map function is called") {
      REQUIRE(future.wait_for(100ms) == std::future_status::ready);
      REQUIRE(future.get() == std::pair<std::string, std::string>{"Map1", "V1"});
    }
  }

  WHEN("An delete map instant action received") {
    handler->resetPromise();
    auto ia = std::make_shared<vda5050pp::core::events::ReceiveInstantActionMessageEvent>();
    ia->instant_actions = std::make_shared<vda5050::InstantActions>();
    ia->instant_actions->actions.push_back(
        vda5050::Action{"deleteMap",
                        "2",
                        std::nullopt,
                        vda5050::BlockingType::NONE,
                        {{{"mapId", "Map2"}, {"mapVersion", "V2"}}}});

    instance->getMessageEventManager().synchronousDispatch(ia);
    auto future = handler->getDeleteFuture();

    THEN("The delete map function is called") {
      REQUIRE(future.wait_for(100ms) == std::future_status::ready);
      REQUIRE(future.get() == std::pair<std::string, std::string>{"Map2", "V2"});
    }
  }

  WHEN("An download map instant action is received") {
    handler->resetPromise();
    auto ia = std::make_shared<vda5050pp::core::events::ReceiveInstantActionMessageEvent>();
    ia->instant_actions = std::make_shared<vda5050::InstantActions>();
    ia->instant_actions->actions.push_back(vda5050::Action{
        "downloadMap",
        "3",
        std::nullopt,
        vda5050::BlockingType::NONE,
        {{{"mapId", "Map2"}, {"mapVersion", "V2"}, {"mapDownloadLink", "http://example.com"}}}});

    instance->getMessageEventManager().synchronousDispatch(ia);
    auto future = handler->getDownloadFuture();

    THEN("The download map function is called") {
      REQUIRE(future.wait_for(100ms) == std::future_status::ready);
      REQUIRE(future.get() == std::tuple<std::string, std::string, std::string>{
                                  "Map2", "V2", "http://example.com"});
    }
  }

  // Use sync dispatch
  vda5050pp::core::Instance::reset();
  evt_opts.synchronous_event_dispatch = true;
  instance = vda5050pp::core::Instance::init(cfg, evt_opts).lock();

  WHEN("stateAddMap is called") {
    vda5050::Map map{
        "id",
        "version",
        std::nullopt,
        vda5050::MapStatus::DISABLED,
    };

    handler->stateAddMap(map);

    THEN("It is in the state") { REQUIRE(instance->getMapManager().hasMap("id")); }

    WHEN("stateEnableMap is called") {
      handler->stateEnableMap("id", "version");

      THEN("It is in the state") {
        REQUIRE(instance->getMapManager().getMap("id") != std::nullopt);
        REQUIRE(instance->getMapManager().getMap("id")->mapVersion == "version");
      }

      WHEN("stateDeleteMap is called") {
        handler->stateDeleteMap("id", "version");

        THEN("It is not in the state") { REQUIRE_FALSE(instance->getMapManager().hasMap("id")); }
      }
    }
  }
}
