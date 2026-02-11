//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//
#include "vda5050++/core/state/state_event_handler.h"

#include <functional>

#include "vda5050++/config/state_subconfig.h"
#include "vda5050++/core/common/exception.h"
#include "vda5050++/events/status_event.h"
using namespace vda5050pp::core::state;

void StateEventHandler::handleMessageErrorEvent(
    std::shared_ptr<vda5050pp::core::events::MessageErrorEvent> data) const noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("MessageError Event is empty"));
  }

  if (data->error_type == misc::MessageErrorType::k_delivery) {
    // Not of interest
    return;
  }

  auto err_evt = std::make_shared<vda5050pp::events::ErrorAdd>();
  err_evt->error.errorType = "validationError";
  err_evt->error.errorLevel = vda5050::ErrorLevel::WARNING;
  err_evt->error.errorDescription = fmt::format("Deserialzation Error: {}", data->description);

  Instance::ref().getStatusEventManager().dispatch(err_evt);
}

void StateEventHandler::handleGraphExtensionEvent(
    std::shared_ptr<vda5050pp::core::events::YieldGraphExtension> data) const noexcept(false) {
  if (data == nullptr || data->graph == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("YieldNewExtension Event is empty"));
  }

  // Add Graph (copy) and retrieve delta bounds
  vda5050pp::core::state::Graph new_graph(*data->graph);
  auto [delta_first, _] = vda5050pp::core::Instance::ref().getOrderManager().extendGraph(
      vda5050pp::core::state::Graph(new_graph), data->order_update_id);
  auto graph = Instance::ref().getOrderManager().getCurrentGraph();
  auto [base_first, delta_last] = graph.baseBounds();

  if (delta_first > delta_last) {
    getStateLogger()->warn("Graph extension resulted in empty delta bounds. Ignoring.");
    return;
  }

  // Extract delta nodes/edges
  auto base_delta = graph.subgraph({delta_first, delta_last});
  auto base_increased = std::make_shared<vda5050pp::events::NavigationBaseIncreased>();
  base_increased->base_expand_nodes = base_delta.getNodes();
  base_increased->base_expand_edges = base_delta.getEdges();
  Instance::ref().getNavigationEventManager().dispatch(base_increased);

  // Extract whole base nodes/edges
  auto base = graph.subgraph({base_first, delta_last});
  auto base_changed = std::make_shared<vda5050pp::events::OrderBaseChanged>();
  base_changed->base_nodes = base.getNodesVec();
  base_changed->base_edges = base.getEdgesVec();
  Instance::ref().getAgvOrderEventManager().dispatch(base_changed);

  // Extract horizon node/edges if available otherwise dispatch empty horizon update
  auto horz_update = std::make_shared<vda5050pp::events::NavigationHorizonUpdate>();
  auto horz_changed = std::make_shared<vda5050pp::events::OrderHorizonChanged>();
  if (graph.hasHorizon()) {
    auto horz = graph.subgraph(graph.horizonBounds());
    horz_update->horz_replace_nodes = horz.getNodes();
    horz_update->horz_replace_edges = horz.getEdges();
    horz_changed->horz_nodes = horz.getNodesVec();
    horz_changed->horz_edges = horz.getEdgesVec();
  }
  Instance::ref().getNavigationEventManager().dispatch(horz_update);
  Instance::ref().getAgvOrderEventManager().dispatch(horz_changed);
}

void StateEventHandler::handleGraphReplacementEvent(
    std::shared_ptr<vda5050pp::core::events::YieldGraphReplacement> data) const noexcept(false) {
  if (data == nullptr || data->graph == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(
        MK_EX_CONTEXT("YieldGraphReplacement Event is empty"));
  }

  // Synchronously clear errors
  auto clear_errors = std::make_shared<vda5050pp::events::ErrorsAlter>();
  clear_errors->alter_function = [](auto &errors) { errors.clear(); };
  Instance::ref().getStatusEventManager().dispatch(clear_errors, true);

  // Create graph and extract base node/edges
  vda5050pp::core::state::Graph graph(*data->graph);
  auto base = graph.subgraph(graph.baseBounds());
  auto base_increased = std::make_shared<vda5050pp::events::NavigationBaseIncreased>();
  base_increased->base_expand_nodes = base.getNodes();
  base_increased->base_expand_edges = base.getEdges();
  auto base_changed = std::make_shared<vda5050pp::events::OrderBaseChanged>();
  base_changed->base_nodes = base.getNodesVec();
  base_changed->base_edges = base.getEdgesVec();

  // Extract horizon node/edges if available
  auto horz_update = std::make_shared<vda5050pp::events::NavigationHorizonUpdate>();
  auto horz_changed = std::make_shared<vda5050pp::events::OrderHorizonChanged>();
  if (graph.hasHorizon()) {
    auto horz = graph.subgraph(graph.horizonBounds());
    horz_update->horz_replace_nodes = horz.getNodes();
    horz_update->horz_replace_edges = horz.getEdges();
    horz_changed->horz_nodes = horz.getNodesVec();
    horz_changed->horz_edges = horz.getEdgesVec();
  }

  if (vda5050pp::core::Instance::ref().getOrderManager().replaceGraph(std::move(graph),
                                                                      data->order_id)) {
    // Last node id also changed
    auto last_node_update = std::make_shared<vda5050pp::events::OrderLastNodeChanged>();
    last_node_update->last_node_id = Instance::ref().getOrderManager().getAGVLastNodeId();
    last_node_update->seq_id = std::nullopt;
    Instance::ref().getAgvOrderEventManager().dispatch(last_node_update);
  }

  Instance::ref().getNavigationEventManager().dispatch(base_increased);
  Instance::ref().getAgvOrderEventManager().dispatch(base_changed);
  Instance::ref().getNavigationEventManager().dispatch(horz_update);
  Instance::ref().getAgvOrderEventManager().dispatch(horz_changed);
}

void StateEventHandler::handleNewActionEvent(
    std::shared_ptr<vda5050pp::core::events::YieldNewAction> data) const noexcept(false) {
  if (data == nullptr || data->action == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("YieldNewAction Event is empty"));
  }

  vda5050pp::core::Instance::ref().getOrderManager().addNewAction(data->action);

  auto actions_changed = std::make_shared<vda5050pp::events::OrderActionStatesChanged>();
  actions_changed->action_states = Instance::ref().getOrderManager().getActionStates();
  Instance::ref().getAgvOrderEventManager().dispatch(actions_changed);
}

void StateEventHandler::handleClearActions(
    std::shared_ptr<vda5050pp::core::events::YieldClearActions> data) const noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("YieldClearActions Event is empty"));
  }

  vda5050pp::core::Instance::ref().getOrderManager().clearActions();

  auto actions_changed = std::make_shared<vda5050pp::events::OrderActionStatesChanged>();
  actions_changed->action_states = Instance::ref().getOrderManager().getActionStates();
  Instance::ref().getAgvOrderEventManager().dispatch(actions_changed);
}

void StateEventHandler::handleOrderNewLastNodeId(
    std::shared_ptr<vda5050pp::core::events::OrderNewLastNodeId> data) const noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("OrderNewLastNodeId Event is empty"));
  }

  vda5050pp::core::Instance::ref().getOrderManager().setAGVLastNode(data->seq_id);
  auto &status_manager = Instance::ref().getStatusManager();
  status_manager.resetDistanceSinceLastNode();

  auto base_changed = std::make_shared<vda5050pp::events::OrderBaseChanged>();
  if (Instance::ref().getOrderManager().hasGraph()) {
    const auto &graph = Instance::ref().getOrderManager().getCurrentGraph();
    auto base = graph.subgraph(graph.baseBounds());
    base_changed->base_nodes = base.getNodesVec();
    base_changed->base_edges = base.getEdgesVec();
  }
  Instance::ref().getAgvOrderEventManager().dispatch(base_changed);

  auto update = std::make_shared<vda5050pp::core::events::RequestStateUpdateEvent>();
  update->urgency = StateUpdateUrgency::high();
  Instance::ref().getStateEventManager().dispatch(update);
}

void StateEventHandler::handleOrderActionStatusChanged(
    std::shared_ptr<vda5050pp::core::events::OrderActionStatusChanged> data) const noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("OrderNewLastNodeId Event is empty"));
  }

  auto status = vda5050pp::core::Instance::ref().getOrderManager().getActionState(data->action_id);
  status->actionStatus = data->action_status;
  status->resultDescription = data->result;

  auto action_states_changed = std::make_shared<vda5050pp::events::OrderActionStatesChanged>();
  action_states_changed->action_states = Instance::ref().getOrderManager().getActionStates();
  Instance::ref().getAgvOrderEventManager().dispatch(action_states_changed);

  auto update = std::make_shared<vda5050pp::core::events::RequestStateUpdateEvent>();
  update->urgency = StateUpdateUrgency::high();
  Instance::ref().getStateEventManager().dispatch(update);
}

void StateEventHandler::handleOrderStatus(
    std::shared_ptr<vda5050pp::core::events::OrderStatus> data) const noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("OrderStatus Event is empty"));
  }

  auto &order_manager = Instance::ref().getOrderManager();

  auto is_pause = [](auto status) {
    return status == vda5050pp::misc::OrderStatus::k_order_paused ||
           status == vda5050pp::misc::OrderStatus::k_order_canceling_paused ||
           status == vda5050pp::misc::OrderStatus::k_order_idle_paused ||
           status == vda5050pp::misc::OrderStatus::k_order_resuming;
  };

  auto pre = order_manager.getOrderStatus();
  order_manager.setOrderStatus(data->status);

  // Update if paused state changed
  if (is_pause(pre) != is_pause(data->status)) {
    auto update = std::make_shared<vda5050pp::core::events::RequestStateUpdateEvent>();
    update->urgency = StateUpdateUrgency::high();
    Instance::ref().getStateEventManager().dispatch(update);
  }
}

void StateEventHandler::handleOrderClearAfterCancel(
    std::shared_ptr<vda5050pp::core::events::OrderClearAfterCancel> data) const {
  if (data == nullptr || data->cancel_action == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(
        MK_EX_CONTEXT("OrderClearAfterCancel Event is empty"));
  }
  auto &om = Instance::ref().getOrderManager();
  om.clearGraph();
  om.cancelWaitingActions();

  // Empty base/horz
  auto base_changed = std::make_shared<vda5050pp::events::OrderBaseChanged>();
  auto horz_changed = std::make_shared<vda5050pp::events::OrderHorizonChanged>();
  auto actions_changed = std::make_shared<vda5050pp::events::OrderActionStatesChanged>();
  actions_changed->action_states = om.getActionStates();
  Instance::ref().getAgvOrderEventManager().dispatch(base_changed);
  Instance::ref().getAgvOrderEventManager().dispatch(horz_changed);
  Instance::ref().getAgvOrderEventManager().dispatch(actions_changed);
}

void StateEventHandler::handleOrderClearAfterReset(
    std::shared_ptr<vda5050pp::core::events::OrderClearAfterReset> data) const {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(
        MK_EX_CONTEXT("OrderClearAfterReset Event is empty"));
  }
  auto &om = Instance::ref().getOrderManager();
  om.clearGraph();
  om.clearActions();

  // Empty base/horz
  auto base_changed = std::make_shared<vda5050pp::events::OrderBaseChanged>();
  auto horz_changed = std::make_shared<vda5050pp::events::OrderHorizonChanged>();
  auto actions_changed = std::make_shared<vda5050pp::events::OrderActionStatesChanged>();
  actions_changed->action_states = om.getActionStates();
  Instance::ref().getAgvOrderEventManager().dispatch(base_changed);
  Instance::ref().getAgvOrderEventManager().dispatch(horz_changed);
  Instance::ref().getAgvOrderEventManager().dispatch(actions_changed);
}

void StateEventHandler::handleNavigationStatusPosition(
    std::shared_ptr<vda5050pp::events::NavigationStatusPosition> data) const noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(
        MK_EX_CONTEXT("NavigationStatusPosition Event is empty"));
  }

  auto cfg = Instance::ref().getConfig().lookupModuleConfigAs<vda5050pp::config::StateSubConfig>(
      module_keys::k_state_event_handler_key);

  auto pos = data->position;

  // If there is no position set, try to use order map, default map or ""
  if (pos.mapId == "") {
    std::optional<std::string_view> map;

    if (cfg->getUseAgvPositionFromOrder() && Instance::ref().getOrderManager().hasGraph()) {
      map = Instance::ref().getOrderManager().getCurrentGraph().currentMap();
    }

    pos.mapId = map.value_or(cfg->getDefaultAgvPositionMap().value_or(""));
  }

  Instance::ref().getStatusManager().setAGVPosition(pos);
}

void StateEventHandler::handleNavigationStatusVelocity(
    std::shared_ptr<vda5050pp::events::NavigationStatusVelocity> data) const noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(
        MK_EX_CONTEXT("NavigationStatusVelocity Event is empty"));
  }

  Instance::ref().getStatusManager().setVelocity(data->velocity);
}

void StateEventHandler::handleNavigationStatusDistanceSinceLastNode(
    std::shared_ptr<vda5050pp::events::NavigationStatusDistanceSinceLastNode> data) const
    noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(
        MK_EX_CONTEXT("NavigationStatusPosition Event is empty"));
  }

  Instance::ref().getStatusManager().setDistanceSinceLastNode(data->distance_since_last_node);
}

void StateEventHandler::handleNavigationStatusDriving(
    std::shared_ptr<vda5050pp::events::NavigationStatusDriving> data) const noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(
        MK_EX_CONTEXT("NavigationStatusDriving Event is empty"));
  }

  if (Instance::ref().getStatusManager().setDriving(data->is_driving)) {
    auto update = std::make_shared<vda5050pp::core::events::RequestStateUpdateEvent>();
    update->urgency = StateUpdateUrgency::high();
    Instance::ref().getStateEventManager().dispatch(update);
  }
}

void StateEventHandler::handleNavigationStatusNodeReached(
    std::shared_ptr<vda5050pp::events::NavigationStatusNodeReached> data) const noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(
        MK_EX_CONTEXT("NavigationStatusDriving Event is empty"));
  }

  if (!data->last_node_id.has_value()) {
    return;
  }

  if (Instance::ref().getOrderManager().setAGVLastNodeId(*data->last_node_id)) {
    auto update = std::make_shared<vda5050pp::core::events::RequestStateUpdateEvent>();
    update->urgency = StateUpdateUrgency::high();
    Instance::ref().getStateEventManager().dispatch(update);

    auto base_changed = std::make_shared<vda5050pp::events::OrderBaseChanged>();
    if (Instance::ref().getOrderManager().hasGraph()) {
      const auto &graph = Instance::ref().getOrderManager().getCurrentGraph();
      auto base = graph.subgraph(graph.baseBounds());
      base_changed->base_nodes = base.getNodesVec();
      base_changed->base_edges = base.getEdgesVec();
    }
    Instance::ref().getAgvOrderEventManager().dispatch(base_changed);
  }
}

void StateEventHandler::handleLoadAdd(std::shared_ptr<vda5050pp::events::LoadAdd> data) const
    noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("LoadAdd Event is empty"));
  }

  if (vda5050pp::core::Instance::ref().getStatusManager().addLoad(data->load)) {
    auto update = std::make_shared<vda5050pp::core::events::RequestStateUpdateEvent>();
    update->urgency = StateUpdateUrgency::high();
    Instance::ref().getStateEventManager().dispatch(update);
  }
}

void StateEventHandler::handleLoadRemove(std::shared_ptr<vda5050pp::events::LoadRemove> data) const
    noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("LoadRemove Event is empty"));
  }

  if (vda5050pp::core::Instance::ref().getStatusManager().removeLoad(data->load_id)) {
    auto update = std::make_shared<vda5050pp::core::events::RequestStateUpdateEvent>();
    update->urgency = StateUpdateUrgency::high();
    Instance::ref().getStateEventManager().dispatch(update);
  }
}

void StateEventHandler::handleLoadsGet(std::shared_ptr<vda5050pp::events::LoadsGet> data) const
    noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("LoadsGet Event is empty"));
  }

  if (auto tkn = data->acquireResultToken(); tkn.isAcquired()) {
    tkn.setValue(vda5050pp::core::Instance::ref().getStatusManager().getLoads());
  } else {
    throw vda5050pp::VDA5050PPSynchronizedEventNotAcquired(
        MK_EX_CONTEXT("StateEventHandler could not acquire the result token of LoadsGet Event"));
  }
}

void StateEventHandler::handleLoadsAlter(std::shared_ptr<vda5050pp::events::LoadsAlter> data) const
    noexcept(false) {
  if (data == nullptr || data->alter_function == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("LoadsAlter Event is empty"));
  }

  if (vda5050pp::core::Instance::ref().getStatusManager().loadsAlter(data->alter_function)) {
    auto update = std::make_shared<vda5050pp::core::events::RequestStateUpdateEvent>();
    update->urgency = StateUpdateUrgency::high();
    Instance::ref().getStateEventManager().dispatch(update);
  }
}

void StateEventHandler::handleOperatingModeSet(
    std::shared_ptr<vda5050pp::events::OperatingModeSet> data) const noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("OperatingModeSet Event is empty"));
  }

  if (vda5050pp::core::Instance::ref().getStatusManager().setOperatingMode(data->operating_mode)) {
    auto update = std::make_shared<vda5050pp::core::events::RequestStateUpdateEvent>();
    update->urgency = StateUpdateUrgency::high();
    Instance::ref().getStateEventManager().dispatch(update);

    auto changed = std::make_shared<vda5050pp::core::events::OperatingModeChangedEvent>();
    changed->operating_mode = data->operating_mode;
    Instance::ref().getStateEventManager().dispatch(changed);
  }
}

void StateEventHandler::handleOperatingModeGet(
    std::shared_ptr<vda5050pp::events::OperatingModeGet> data) const noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("OperatingModeGet Event is empty"));
  }

  if (auto tkn = data->acquireResultToken(); tkn.isAcquired()) {
    tkn.setValue(vda5050pp::core::Instance::ref().getStatusManager().getOperatingMode());
  } else {
    throw vda5050pp::VDA5050PPSynchronizedEventNotAcquired(MK_EX_CONTEXT(
        "StateEventHandler could not acquire the result token of OperatingModeGet Event"));
  }
}

void StateEventHandler::handleOperatingModeAlter(
    std::shared_ptr<vda5050pp::events::OperatingModeAlter> data) const noexcept(false) {
  if (data == nullptr || data->alter_function == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("OperatingModeAlter Event is empty"));
  }

  if (vda5050pp::core::Instance::ref().getStatusManager().operatingModeAlter(
          data->alter_function)) {
    auto update = std::make_shared<vda5050pp::core::events::RequestStateUpdateEvent>();
    update->urgency = StateUpdateUrgency::high();
    Instance::ref().getStateEventManager().dispatch(update);

    auto changed = std::make_shared<vda5050pp::core::events::OperatingModeChangedEvent>();
    changed->operating_mode = Instance::ref().getStatusManager().getOperatingMode();
    Instance::ref().getStateEventManager().dispatch(changed);
  }
}

void StateEventHandler::handleBatteryStateSet(
    std::shared_ptr<vda5050pp::events::BatteryStateSet> data) const noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("BatteryStateSet Event is empty"));
  }

  vda5050pp::core::Instance::ref().getStatusManager().setBatteryState(data->battery_state);
}

void StateEventHandler::handleBatteryStateGet(
    std::shared_ptr<vda5050pp::events::BatteryStateGet> data) const noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("BatteryStateGet Event is empty"));
  }

  if (auto tkn = data->acquireResultToken(); tkn.isAcquired()) {
    tkn.setValue(vda5050pp::core::Instance::ref().getStatusManager().getBatteryState());
  } else {
    throw vda5050pp::VDA5050PPSynchronizedEventNotAcquired(MK_EX_CONTEXT(
        "StateEventHandler could not acquire the result token of BatteryStateGet Event"));
  }
}

void StateEventHandler::handleBatteryStateAlter(
    std::shared_ptr<vda5050pp::events::BatteryStateAlter> data) const noexcept(false) {
  if (data == nullptr || data->alter_function == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("BatteryStateAlter Event is empty"));
  }

  vda5050pp::core::Instance::ref().getStatusManager().alterBatteryState(data->alter_function);
}
void StateEventHandler::handleSafetyStateSet(
    std::shared_ptr<vda5050pp::events::SafetyStateSet> data) const noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("SafetyStateSet Event is empty"));
  }
  if (vda5050pp::core::Instance::ref().getStatusManager().setSafetyState(data->safety_state)) {
    auto update = std::make_shared<vda5050pp::core::events::RequestStateUpdateEvent>();
    update->urgency = StateUpdateUrgency::high();
    Instance::ref().getStateEventManager().dispatch(update);
  }
}

void StateEventHandler::handleSafetyStateGet(
    std::shared_ptr<vda5050pp::events::SafetyStateGet> data) const noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("SafetyStateGet Event is empty"));
  }

  if (auto tkn = data->acquireResultToken(); tkn.isAcquired()) {
    tkn.setValue(vda5050pp::core::Instance::ref().getStatusManager().getSafetyState());
  } else {
    throw vda5050pp::VDA5050PPSynchronizedEventNotAcquired(MK_EX_CONTEXT(
        "StateEventHandler could not acquire the result token of SafetyStateGet Event"));
  }
}

void StateEventHandler::handleSafetyStateAlter(
    std::shared_ptr<vda5050pp::events::SafetyStateAlter> data) const noexcept(false) {
  if (data == nullptr || data->alter_function == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("SafetyStateAlter Event is empty"));
  }

  if (vda5050pp::core::Instance::ref().getStatusManager().alterSafetyState(data->alter_function)) {
    auto update = std::make_shared<vda5050pp::core::events::RequestStateUpdateEvent>();
    update->urgency = StateUpdateUrgency::high();
    Instance::ref().getStateEventManager().dispatch(update);
  }
}

void StateEventHandler::handleRequestNewBase(
    std::shared_ptr<vda5050pp::events::RequestNewBase> data) const noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("RequestNewBase Event is empty"));
  }

  vda5050pp::core::Instance::ref().getStatusManager().requestNewBase();

  auto update = std::make_shared<vda5050pp::core::events::RequestStateUpdateEvent>();
  update->urgency = StateUpdateUrgency::high();
  Instance::ref().getStateEventManager().dispatch(update);
}

void StateEventHandler::handleErrorAdd(std::shared_ptr<vda5050pp::events::ErrorAdd> data) const
    noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("ErrorAdd Event is empty"));
  }

  if (vda5050pp::core::Instance::ref().getStatusManager().addError(data->error)) {
    auto update = std::make_shared<vda5050pp::core::events::RequestStateUpdateEvent>();
    update->urgency = StateUpdateUrgency::high();
    Instance::ref().getStateEventManager().dispatch(update);

    auto changed = std::make_shared<vda5050pp::events::OrderErrorsChanged>();
    changed->errors = Instance::ref().getStatusManager().getErrors();
    Instance::ref().getAgvOrderEventManager().dispatch(changed);
  }
}

void StateEventHandler::handleErrorsAlter(
    std::shared_ptr<vda5050pp::events::ErrorsAlter> data) const noexcept(false) {
  if (data == nullptr || data->alter_function == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("ErrorsAlter Event is empty"));
  }

  if (vda5050pp::core::Instance::ref().getStatusManager().alterErrors(data->alter_function)) {
    auto update = std::make_shared<vda5050pp::core::events::RequestStateUpdateEvent>();
    update->urgency = StateUpdateUrgency::high();
    Instance::ref().getStateEventManager().dispatch(update);

    auto changed = std::make_shared<vda5050pp::events::OrderErrorsChanged>();
    changed->errors = Instance::ref().getStatusManager().getErrors();
    Instance::ref().getAgvOrderEventManager().dispatch(changed);
  }
}

void StateEventHandler::handleInfoAdd(std::shared_ptr<vda5050pp::events::InfoAdd> data) const
    noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("InfoAdd Event is empty"));
  }

  vda5050pp::core::Instance::ref().getStatusManager().addInfo(data->info);

  auto update = std::make_shared<vda5050pp::core::events::RequestStateUpdateEvent>();
  update->urgency = StateUpdateUrgency::high();
  Instance::ref().getStateEventManager().dispatch(update);
}

void StateEventHandler::handleInfosAlter(std::shared_ptr<vda5050pp::events::InfosAlter> data) const
    noexcept(false) {
  if (data == nullptr || data->alter_function == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("InfosAlter Event is empty"));
  }

  vda5050pp::core::Instance::ref().getStatusManager().alterInfos(data->alter_function);

  auto update = std::make_shared<vda5050pp::core::events::RequestStateUpdateEvent>();
  update->urgency = StateUpdateUrgency::high();
  Instance::ref().getStateEventManager().dispatch(update);
}

void StateEventHandler::handleAddMap(std::shared_ptr<vda5050pp::events::AddMap> data) const
    noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("AddMap Event is empty"));
  }

  getStateLogger()->info("Adding map: {} (version={})", data->map.mapId, data->map.mapVersion);
  Instance::ref().getMapManager().addMap(data->map);
}

void StateEventHandler::handleEnableMap(std::shared_ptr<vda5050pp::events::EnableMap> data) const
    noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("EnableMap Event is empty"));
  }

  getStateLogger()->info("Enabling map: {} (version={})", data->map_id, data->map_version);
  Instance::ref().getMapManager().enableMap(data->map_id, data->map_version);
}

void StateEventHandler::handleDeleteMap(std::shared_ptr<vda5050pp::events::DeleteMap> data) const
    noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("DeleteMap Event is empty"));
  }

  getStateLogger()->info("Delete map: {} (version={})", data->map_id, data->map_version);
  Instance::ref().getMapManager().deleteMap(data->map_id, data->map_version);
}

void StateEventHandler::initialize(vda5050pp::core::Instance &instance) {
  this->messages_subscriber_ = instance.getMessageEventManager().getScopedSubscriber();
  this->messages_subscriber_->subscribe<vda5050pp::core::events::MessageErrorEvent>(std::bind(
      std::mem_fn(&StateEventHandler::handleMessageErrorEvent), this, std::placeholders::_1));

  this->interpreter_subscriber_ = instance.getInterpreterEventManager().getScopedSubscriber();

  this->interpreter_subscriber_->subscribe<vda5050pp::core::events::YieldGraphExtension>(std::bind(
      std::mem_fn(&StateEventHandler::handleGraphExtensionEvent), this, std::placeholders::_1));
  this->interpreter_subscriber_->subscribe<vda5050pp::core::events::YieldGraphReplacement>(
      std::bind(std::mem_fn(&StateEventHandler::handleGraphReplacementEvent), this,
                std::placeholders::_1));
  this->interpreter_subscriber_->subscribe<vda5050pp::core::events::YieldNewAction>(std::bind(
      std::mem_fn(&StateEventHandler::handleNewActionEvent), this, std::placeholders::_1));
  this->interpreter_subscriber_->subscribe<vda5050pp::core::events::YieldClearActions>(
      std::bind(std::mem_fn(&StateEventHandler::handleClearActions), this, std::placeholders::_1));

  this->order_subscriber_ = instance.getOrderEventManager().getScopedSubscriber();
  this->order_subscriber_->subscribe<vda5050pp::core::events::OrderNewLastNodeId>(std::bind(
      std::mem_fn(&StateEventHandler::handleOrderNewLastNodeId), this, std::placeholders::_1));
  this->order_subscriber_->subscribe<vda5050pp::core::events::OrderActionStatusChanged>(
      std::bind(std::mem_fn(&StateEventHandler::handleOrderActionStatusChanged), this,
                std::placeholders::_1));
  this->order_subscriber_->subscribe<vda5050pp::core::events::OrderClearAfterCancel>(std::bind(
      std::mem_fn(&StateEventHandler::handleOrderClearAfterCancel), this, std::placeholders::_1));
  this->order_subscriber_->subscribe<vda5050pp::core::events::OrderClearAfterReset>(std::bind(
      std::mem_fn(&StateEventHandler::handleOrderClearAfterReset), this, std::placeholders::_1));
  this->order_subscriber_->subscribe<vda5050pp::core::events::OrderStatus>(
      std::bind(std::mem_fn(&StateEventHandler::handleOrderStatus), this, std::placeholders::_1));

  this->navigation_subscriber_ =
      instance.getNavigationStatusManager().getScopedNavigationStatusSubscriber();
  this->navigation_subscriber_->subscribe(
      std::bind(std::mem_fn(&StateEventHandler::handleNavigationStatusPosition), this,
                std::placeholders::_1));
  this->navigation_subscriber_->subscribe(
      std::bind(std::mem_fn(&StateEventHandler::handleNavigationStatusVelocity), this,
                std::placeholders::_1));
  this->navigation_subscriber_->subscribe(std::bind(
      std::mem_fn(&StateEventHandler::handleNavigationStatusDriving), this, std::placeholders::_1));
  this->navigation_subscriber_->subscribe(
      std::bind(std::mem_fn(&StateEventHandler::handleNavigationStatusDistanceSinceLastNode), this,
                std::placeholders::_1));
  this->navigation_subscriber_->subscribe(
      std::bind(std::mem_fn(&StateEventHandler::handleNavigationStatusNodeReached), this,
                std::placeholders::_1));

  this->status_subscriber_ = instance.getStatusEventManager().getScopedStatusEventSubscriber();

  this->status_subscriber_->subscribe(
      std::bind(std::mem_fn(&StateEventHandler::handleLoadAdd), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(
      std::bind(std::mem_fn(&StateEventHandler::handleLoadRemove), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(
      std::bind(std::mem_fn(&StateEventHandler::handleLoadsGet), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(
      std::bind(std::mem_fn(&StateEventHandler::handleLoadsAlter), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(std::bind(
      std::mem_fn(&StateEventHandler::handleOperatingModeSet), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(std::bind(
      std::mem_fn(&StateEventHandler::handleOperatingModeGet), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(std::bind(
      std::mem_fn(&StateEventHandler::handleOperatingModeAlter), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(std::bind(
      std::mem_fn(&StateEventHandler::handleBatteryStateSet), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(std::bind(
      std::mem_fn(&StateEventHandler::handleBatteryStateGet), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(std::bind(
      std::mem_fn(&StateEventHandler::handleBatteryStateAlter), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(std::bind(
      std::mem_fn(&StateEventHandler::handleSafetyStateSet), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(std::bind(
      std::mem_fn(&StateEventHandler::handleSafetyStateGet), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(std::bind(
      std::mem_fn(&StateEventHandler::handleSafetyStateAlter), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(std::bind(
      std::mem_fn(&StateEventHandler::handleRequestNewBase), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(
      std::bind(std::mem_fn(&StateEventHandler::handleErrorAdd), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(
      std::bind(std::mem_fn(&StateEventHandler::handleErrorsAlter), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(
      std::bind(std::mem_fn(&StateEventHandler::handleInfoAdd), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(
      std::bind(std::mem_fn(&StateEventHandler::handleInfosAlter), this, std::placeholders::_1));

  this->status_subscriber_->subscribe(
      std::bind(std::mem_fn(&StateEventHandler::handleAddMap), this, std::placeholders::_1));
  this->status_subscriber_->subscribe(
      std::bind(std::mem_fn(&StateEventHandler::handleDeleteMap), this, std::placeholders::_1));
  this->status_subscriber_->subscribe(
      std::bind(std::mem_fn(&StateEventHandler::handleEnableMap), this, std::placeholders::_1));
}

void StateEventHandler::deinitialize(vda5050pp::core::Instance &) {
  this->messages_subscriber_.reset();
  this->interpreter_subscriber_.reset();
  this->order_subscriber_.reset();
  this->navigation_subscriber_.reset();
  this->status_subscriber_.reset();
}

std::string_view StateEventHandler::describe() const { return "StateEventHandler"; }

std::shared_ptr<vda5050pp::config::ModuleSubConfig> StateEventHandler::generateSubConfig() const {
  return std::make_shared<vda5050pp::config::StateSubConfig>();
}
