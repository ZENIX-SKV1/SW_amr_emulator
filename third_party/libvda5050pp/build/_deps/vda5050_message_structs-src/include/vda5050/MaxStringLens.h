// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_MAXSTRINGLENS_H_
#define VDA5050_MAXSTRINGLENS_H_

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>

namespace vda5050 {
struct MaxStringLens {
  /// Maximum MQTT message length
  std::optional<uint32_t> msgLen;

  /// Maximum length of serial-number part in MQTT-topics.
  /// Affected parameters:
  /// order.serialNumber
  /// instantActions.serialNumber
  /// state.SerialNumber
  /// visualization.serialNumber
  /// connection.serialNumber
  std::optional<uint32_t> topicSerialLen;

  /// Maximum length of all other parts in MQTT-topics.
  /// Affected parameters:
  /// order.timestamp
  /// order.version
  /// order.manufacturer
  /// instantActions.timestamp
  /// instantActions.version
  /// instantActions.manufacturer
  /// state.timestamp
  /// state.version
  /// state.manufacturer
  /// visualization.timestamp
  /// visualization.version
  /// visualization.manufacturer
  /// connection.timestamp
  /// connection.version
  /// connection.manufacturer
  std::optional<uint32_t> topicElemLen;

  /// Maximum length of ID-Strings.
  /// Affected parameters:
  /// order.orderId
  /// order.zoneSetId
  /// node.nodeId
  /// nodePosition.mapId
  /// action.actionId
  /// edge.edgeId
  /// edge.startNodeId
  /// edge.endNodeId
  std::optional<uint32_t> idLen;

  /// If "true" ID-strings need to contain numerical values only.
  std::optional<bool> idNumericalOnly;

  /// Maximum length of ENUM- and Key-Strings.
  /// Affected parameters:
  /// action.actionType action.blockingType
  /// edge.direction
  /// actionParameter.key
  /// state.operatingMode
  /// load.loadPosition
  /// load.loadType
  /// actionState.actionStatus
  /// error.errorType
  /// error.errorLevel
  /// errorReference.referenceKey
  /// info.infoType
  /// info.infoLevel
  /// safetyState.eStop
  /// connection.connectionState
  std::optional<uint32_t> enumLen;

  /// Maximum length of loadId Strings
  std::optional<uint32_t> loadIdLen;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const MaxStringLens &other) const {
    if (this->enumLen != other.enumLen) return false;
    if (this->idLen != other.idLen) return false;
    if (this->idNumericalOnly != other.idNumericalOnly) return false;
    if (this->loadIdLen != other.loadIdLen) return false;
    if (this->msgLen != other.msgLen) return false;
    if (this->topicElemLen != other.topicElemLen) return false;
    if (this->topicSerialLen != other.topicSerialLen) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const MaxStringLens &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const MaxStringLens &d);
void from_json(const json &j, MaxStringLens &d);

}  // namespace vda5050
#endif  // VDA5050_MAXSTRINGLENS_H_
