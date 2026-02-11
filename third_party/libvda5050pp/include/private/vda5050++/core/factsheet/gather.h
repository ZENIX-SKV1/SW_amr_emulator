// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
// This file contains factsheet gather functions.
//
//

#ifndef VDA5050_2B_2B_CORE_FACTSHEET_GATHER_H_
#define VDA5050_2B_2B_CORE_FACTSHEET_GATHER_H_

#include "vda5050/AgvFactsheet.h"

namespace vda5050pp::core::factsheet {

///
///\brief Gather geometry data from the AGVDescription
///
///\return vda5050::AgvGeometry the geometry
///
vda5050::AgvGeometry gatherGeometry();

///
///\brief Gather load specification data from the AGVDescription
///
///\return vda5050::LoadSpecification the load specification
///
vda5050::LoadSpecification gatherLoadSpecification();

///
///\brief Gather physical parameters from the AGVDescription
///
///\return vda5050::PhysicalParameters the physical parameters
///
vda5050::PhysicalParameters gatherPhysicalParameters();

///
///\brief Gather protocol features from the registered action handlers and internal control action
/// handlers.
///
///\return vda5050::ProtocolFeatures the protocol features
///
///\throws VDA5050PPSynchronizedEventTimedOut if the internal FactsheetControlActionListEvent is not
/// handled.
///\throws VDA5050PPSynchronizedEventTimedOut if the FactsheetActionListEvent is not handled.
///\throws VDA5050PPNullPointer if the action list result is empty
///
vda5050::ProtocolFeatures gatherProtocolFeatures();

///
///\brief Gather protocol limits from the AGVDescription and fill the rest with default values for
/// this library.
///
///\return vda5050::ProtocolLimits the protocol limits
///
vda5050::ProtocolLimits gatherProtocolLimits();

///
///\brief Gather type specification from the AGVDescription
///
///\return vda5050::TypeSpecification the type specification
///
vda5050::TypeSpecification gatherTypeSpecification();

///
///\brief Gather the vehicle configuration of the AGV
///
///\return vda5050::VehicleConfig the vehicle config
///
vda5050::VehicleConfig gatherVehicleConfig();

}  // namespace vda5050pp::core::factsheet

#endif  // VDA5050_2B_2B_CORE_FACTSHEET_GATHER_H_
