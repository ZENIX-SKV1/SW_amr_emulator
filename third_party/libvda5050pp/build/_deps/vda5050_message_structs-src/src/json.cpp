// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#include <ctime>
#include <iomanip>
#include <sstream>

#include "vda5050/AgvFactsheet.h"
#include "vda5050/Connection.h"
#include "vda5050/Envelope2d.h"
#include "vda5050/InstantActions.h"
#include "vda5050/Order.h"
#include "vda5050/PolygonPoint.h"
#include "vda5050/State.h"
#include "vda5050/Visualization.h"

namespace vda5050 {

constexpr const char *k_iso8601_fmt = "%Y-%m-%dT%H:%M:%S";

void to_json(json &j, const HeaderVDA5050 &d) {
  j["headerId"] = d.headerId;

  // Split timestamp into seconds and milliseconds (seconds for time_t, milliseconds for manual
  // formatting)
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(d.timestamp.time_since_epoch());
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(d.timestamp.time_since_epoch() -
                                                                  seconds);
  if (ms.count() < 0) {
    // Handle negative durations (i.e. before epoch)
    ms += std::chrono::seconds(1);
    seconds -= std::chrono::seconds(1);
  }

  // Write ISO8601 UTC timestamp
  auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::time_point(seconds));
  std::stringstream ss;
  std::tm tm = {};

  // Convert tt to tm
#if VDA5050_MESSAGE_STRUCTS_HAS_GMTIME_S
  auto err = gmtime_s(&tm, &tt);
#elif VDA5050_MESSAGE_STRUCTS_HAS_GMTIME_R
  gmtime_r(&tt, &tm);
#else
#error "No gmtime_s or gmtime_r available!"
#endif

  // Write seconds
  ss << std::put_time(&tm, k_iso8601_fmt);
  // Write milliseconds manually
  ss << '.' << std::setfill('0') << std::setw(3) << ms.count() << 'Z';

  j["timestamp"] = ss.str();
  j["version"] = d.version;
  j["manufacturer"] = d.manufacturer;
  j["serialNumber"] = d.serialNumber;
}

void from_json(const json &j, HeaderVDA5050 &d) {
  d.headerId = j.at("headerId");

  // Parse ISO8601 UTC timestamp
  using timestampT = decltype(vda5050::HeaderVDA5050::timestamp);
  std::string timestamp_str = j.at("timestamp");
  std::stringstream ss(timestamp_str);
  std::tm tm = {};
  ss >> std::get_time(&tm, k_iso8601_fmt);

  // Read fractional part if present
  std::chrono::milliseconds frac(0);
  if (ss.peek() == '.') {
    // Milliseconds are present
    char sep;
    uint32_t count = 0;
    ss >> sep >> count;

    // Convert fractional part to seconds as float
    float frac_float = static_cast<float>(count);
    while (frac_float >= 1.0f) {
      frac_float /= 10.0f;
    }
    frac = std::chrono::milliseconds(static_cast<int64_t>(frac_float * 1000.0f));
  }

  if (ss.peek() == 'Z') {
    ss.get();
  }

  // Convert tm to timestamp
#if VDA5050_MESSAGE_STRUCTS_HAS_TIMEGM
  d.timestamp = timestampT::clock::from_time_t(timegm(&tm));
#elif VDA5050_MESSAGE_STRUCTS_HAS_MKGMTIME
  d.timestamp = timestampT::clock::from_time_t(_mkgmtime(&tm));
#else
#error "No timegm or _mkgmtime available!"
#endif

  // Add fractional part to timestamp
  d.timestamp += std::chrono::duration_cast<timestampT::duration>(frac);

  d.version = j.at("version");
  d.manufacturer = j.at("manufacturer");
  d.serialNumber = j.at("serialNumber");
}

void to_json(json &j, const ConnectionState &d) {
  switch (d) {
    case ConnectionState::ONLINE:
      j = "ONLINE";
      break;
    case ConnectionState::OFFLINE:
      j = "OFFLINE";
      break;
    case ConnectionState::CONNECTIONBROKEN:
      j = "CONNECTIONBROKEN";
      break;
    default:
      j = "UNKNOWN";
      break;
  }
}
void from_json(const json &j, ConnectionState &d) {
  auto str = j.get<std::string>();
  if (str == "ONLINE") {
    d = ConnectionState::ONLINE;
  } else if (str == "OFFLINE") {
    d = ConnectionState::OFFLINE;
  } else if (str == "CONNECTIONBROKEN") {
    d = ConnectionState::CONNECTIONBROKEN;
  }
}

void to_json(json &j, const Connection &d) {
  to_json(j, d.header);
  j["connectionState"] = d.connectionState;
}
void from_json(const json &j, Connection &d) {
  from_json(j, d.header);
  j.at("connectionState").get_to(d.connectionState);
}

void to_json(json &j, const AGVPosition &d) {
  j["positionInitialized"] = d.positionInitialized;
  j["x"] = d.x;
  j["y"] = d.y;
  j["theta"] = d.theta;
  if (d.deviationRange.has_value()) {
    j["deviationRange"] = *d.deviationRange;
  }
  if (d.localizationScore.has_value()) {
    j["localizationScore"] = *d.localizationScore;
  }
  j["mapId"] = d.mapId;
  if (d.mapDescription.has_value()) {
    j["mapDescription"] = *d.mapDescription;
  }
}
void from_json(const json &j, AGVPosition &d) {
  j.at("positionInitialized").get_to(d.positionInitialized);
  j.at("x").get_to(d.x);
  j.at("y").get_to(d.y);
  j.at("theta").get_to(d.theta);
  if (j.contains("deviationRange")) {
    d.deviationRange = j.at("deviationRange");
  }
  if (j.contains("localizationScore")) {
    d.localizationScore = j.at("localizationScore");
  }
  j.at("mapId").get_to(d.mapId);
  if (j.contains("mapDescription")) {
    d.mapDescription = j.at("mapDescription");
  }
}

void to_json(json &j, const Velocity &d) {
  if (d.vx.has_value()) {
    j["vx"] = *d.vx;
  }
  if (d.vy.has_value()) {
    j["vy"] = *d.vy;
  }
  if (d.omega.has_value()) {
    j["omega"] = *d.omega;
  }
}
void from_json(const json &j, Velocity &d) {
  if (j.contains("omega")) {
    d.omega = j.at("omega");
  }
  if (j.contains("vx")) {
    d.vx = j.at("vx");
  }
  if (j.contains("vy")) {
    d.vy = j.at("vy");
  }
}

void to_json(json &j, const Visualization &d) {
  to_json(j, d.header);
  if (d.agvPosition.has_value()) {
    j["agvPosition"] = *d.agvPosition;
  }
  if (d.velocity.has_value() && *d.velocity != Velocity{}) {
    j["velocity"] = *d.velocity;
  }
}
void from_json(const json &j, Visualization &d) {
  from_json(j, d.header);
  if (j.contains("agvPosition")) {
    d.agvPosition = j.at("agvPosition");
  }
  if (j.contains("velocity")) {
    d.velocity = j.at("velocity");
  }
}

void to_json(json &j, const ActionParameter &d) {
  j["key"] = d.key;
  j["value"] = d.value;
}
void from_json(const json &j, ActionParameter &d) {
  d.key = j.at("key");
  d.value = j.at("value");
}

void to_json(json &j, const BlockingType &d) {
  switch (d) {
    case BlockingType::SOFT:
      j = "SOFT";
      break;
    case BlockingType::HARD:
      j = "HARD";
      break;
    case BlockingType::NONE:
      j = "NONE";
      break;
    default:
      j = "UNKNOWN";
      break;
  }
}
void from_json(const json &j, BlockingType &d) {
  auto str = j.get<std::string>();
  if (str == "SOFT") {
    d = BlockingType::SOFT;
  } else if (str == "HARD") {
    d = BlockingType::HARD;
  } else if (str == "NONE") {
    d = BlockingType::NONE;
  }
}

void to_json(json &j, const Action &d) {
  if (d.actionDescription.has_value()) {
    j["actionDescription"] = *d.actionDescription;
  }
  j["actionId"] = d.actionId;
  if (d.actionParameters.has_value()) {
    j["actionParameters"] = *d.actionParameters;
  }
  j["actionType"] = d.actionType;
  j["blockingType"] = d.blockingType;
}
void from_json(const json &j, Action &d) {
  if (j.contains("actionDescription")) {
    d.actionDescription = j.at("actionDescription");
  }
  d.actionId = j.at("actionId");
  if (j.contains("actionParameters")) {
    d.actionParameters = j.at("actionParameters").get<std::vector<ActionParameter>>();
  }
  d.actionType = j.at("actionType");
  d.blockingType = j.at("blockingType");
}

void to_json(json &j, const NodePosition &d) {
  if (d.allowedDeviationTheta.has_value()) {
    j["allowedDeviationTheta"] = *d.allowedDeviationTheta;
  }
  if (d.allowedDeviationXY.has_value()) {
    j["allowedDeviationXY"] = *d.allowedDeviationXY;
  }
  if (d.mapDescription.has_value()) {
    j["mapDescription"] = *d.mapDescription;
  }
  j["mapId"] = d.mapId;
  if (d.theta.has_value()) {
    j["theta"] = *d.theta;
  }
  j["x"] = d.x;
  j["y"] = d.y;
}
void from_json(const json &j, NodePosition &d) {
  if (j.contains("allowedDeviationTheta")) {
    d.allowedDeviationTheta = j.at("allowedDeviationTheta");
  }
  if (j.contains("allowedDeviationXY")) {
    d.allowedDeviationXY = j.at("allowedDeviationXY");
  }
  if (j.contains("mapDescription")) {
    d.mapDescription = j.at("mapDescription");
  }
  d.mapId = j.at("mapId");
  if (j.contains("theta")) {
    d.theta = j.at("theta");
  }
  d.x = j.at("x");
  d.y = j.at("y");
}

void to_json(json &j, const Node &d) {
  j["actions"] = d.actions;
  if (d.nodeDescription.has_value()) {
    j["nodeDescription"] = *d.nodeDescription;
  }
  j["nodeId"] = d.nodeId;
  if (d.nodePosition.has_value()) {
    j["nodePosition"] = *d.nodePosition;
  }
  j["released"] = d.released;
  j["sequenceId"] = d.sequenceId;
}
void from_json(const json &j, Node &d) {
  d.actions = j.at("actions").get<std::vector<Action>>();
  if (j.contains("nodeDescription")) {
    d.nodeDescription = j.at("nodeDescription");
  }
  d.nodeId = j.at("nodeId");
  if (j.contains("nodePosition")) {
    d.nodePosition = j.at("nodePosition");
  }
  d.released = j.at("released");
  d.sequenceId = j.at("sequenceId");
}

void to_json(json &j, const ControlPoint &d) {
  j["weight"] = d.weight;
  j["x"] = d.x;
  j["y"] = d.y;
}
void from_json(const json &j, ControlPoint &d) {
  d.weight = j.at("weight");
  d.x = j.at("x");
  d.y = j.at("y");
}

void to_json(json &j, const Trajectory &d) {
  j["controlPoints"] = d.controlPoints;
  j["degree"] = d.degree;
  j["knotVector"] = d.knotVector;
}
void from_json(const json &j, Trajectory &d) {
  d.controlPoints = j.at("controlPoints").get<std::vector<ControlPoint>>();
  d.degree = j.at("degree");
  d.knotVector = j.at("knotVector").get<std::vector<double>>();
}

void to_json(json &j, const CorridorRefPoint &d) {
  switch (d) {
    case CorridorRefPoint::KINEMATICCENTER:
      j = "KINEMATICCENTER";
      break;
    case CorridorRefPoint::CONTOUR:
      j = "CONTOUR";
      break;
    default:
      j = "UNKNOWN";
      break;
  }
}
void from_json(const json &j, CorridorRefPoint &d) {
  auto str = j.get<std::string>();
  if (str == "KINEMATICCENTER") {
    d = CorridorRefPoint::KINEMATICCENTER;
  } else if (str == "CONTOUR") {
    d = CorridorRefPoint::CONTOUR;
  }
}

void to_json(json &j, const Corridor &d) {
  j["leftWidth"] = d.leftWidth;
  j["rightWidth"] = d.rightWidth;
  if (d.corridorRefPoint.has_value()) {
    j["corridorRefPoint"] = *d.corridorRefPoint;
  }
}
void from_json(const json &j, Corridor &d) {
  d.leftWidth = j.at("leftWidth");
  d.rightWidth = j.at("rightWidth");
  if (j.contains("corridorRefPoint")) {
    d.corridorRefPoint = j.at("corridorRefPoint");
  }
}

void to_json(json &j, const Edge &d) {
  j["actions"] = d.actions;
  if (d.direction.has_value()) {
    j["direction"] = *d.direction;
  }
  if (d.edgeDescription.has_value()) {
    j["edgeDescription"] = *d.edgeDescription;
  }
  j["edgeId"] = d.edgeId;
  j["endNodeId"] = d.endNodeId;
  if (d.length.has_value()) {
    j["length"] = *d.length;
  }
  if (d.corridor.has_value()) {
    j["corridor"] = *d.corridor;
  }
  if (d.maxHeight.has_value()) {
    j["maxHeight"] = *d.maxHeight;
  }
  if (d.maxRotationSpeed.has_value()) {
    j["maxRotationSpeed"] = *d.maxRotationSpeed;
  }
  if (d.maxSpeed.has_value()) {
    j["maxSpeed"] = *d.maxSpeed;
  }
  if (d.minHeight.has_value()) {
    j["minHeight"] = *d.minHeight;
  }
  if (d.orientation.has_value()) {
    j["orientation"] = *d.orientation;
  }
  if (d.orientationType.has_value()) {
    j["orientationType"] = *d.orientationType;
  }
  j["released"] = d.released;
  if (d.rotationAllowed.has_value()) {
    j["rotationAllowed"] = *d.rotationAllowed;
  }
  j["sequenceId"] = d.sequenceId;
  j["startNodeId"] = d.startNodeId;
  if (d.trajectory.has_value()) {
    j["trajectory"] = *d.trajectory;
  }
}
void from_json(const json &j, Edge &d) {
  d.actions = j.at("actions").get<std::vector<Action>>();
  if (j.contains("direction")) {
    d.direction = j.at("direction");
  }
  if (j.contains("edgeDescription")) {
    d.edgeDescription = j.at("edgeDescription");
  }
  d.edgeId = j.at("edgeId");
  d.endNodeId = j.at("endNodeId");
  if (j.contains("length")) {
    d.length = j.at("length");
  }
  if (j.contains("corridor")) {
    d.corridor = j.at("corridor");
  }
  if (j.contains("maxHeight")) {
    d.maxHeight = j.at("maxHeight");
  }
  if (j.contains("maxRotationSpeed")) {
    d.maxRotationSpeed = j.at("maxRotationSpeed");
  }
  if (j.contains("maxSpeed")) {
    d.maxSpeed = j.at("maxSpeed");
  }
  if (j.contains("minHeight")) {
    d.minHeight = j.at("minHeight");
  }
  if (j.contains("orientation")) {
    d.orientation = j.at("orientation");
  }
  if (j.contains("orientationType")) {
    d.orientationType = j.at("orientationType");
  }
  d.released = j.at("released");
  if (j.contains("rotationAllowed")) {
    d.rotationAllowed = j.at("rotationAllowed");
  }
  d.sequenceId = j.at("sequenceId");
  d.startNodeId = j.at("startNodeId");
  if (j.contains("trajectory")) {
    d.trajectory = j.at("trajectory");
  }
}

void to_json(json &j, const Order &d) {
  to_json(j, d.header);
  j["edges"] = d.edges;
  j["nodes"] = d.nodes;
  j["orderId"] = d.orderId;
  j["orderUpdateId"] = d.orderUpdateId;
  if (d.zoneSetId.has_value()) {
    j["zoneSetId"] = *d.zoneSetId;
  }
}
void from_json(const json &j, Order &d) {
  from_json(j, d.header);
  d.edges = j.at("edges").get<std::vector<Edge>>();
  d.nodes = j.at("nodes").get<std::vector<Node>>();
  d.orderId = j.at("orderId");
  d.orderUpdateId = j.at("orderUpdateId");
  if (j.contains("zoneSetId")) {
    d.zoneSetId = j.at("zoneSetId");
  }
}

void to_json(json &j, const InstantActions &d) {
  to_json(j, d.header);
  j["actions"] = d.actions;
}
void from_json(const json &j, InstantActions &d) {
  from_json(j, d.header);
  d.actions = j.at("actions").get<std::vector<Action>>();
}

void to_json(json &j, const ActionStatus &d) {
  switch (d) {
    case ActionStatus::FINISHED:
      j = "FINISHED";
      break;
    case ActionStatus::INITIALIZING:
      j = "INITIALIZING";
      break;
    case ActionStatus::PAUSED:
      j = "PAUSED";
      break;
    case ActionStatus::RUNNING:
      j = "RUNNING";
      break;
    case ActionStatus::WAITING:
      j = "WAITING";
      break;
    case ActionStatus::FAILED:
      j = "FAILED";
      break;
    default:
      j = "UNKNOWN";
      break;
  }
}
void from_json(const json &j, ActionStatus &d) {
  auto str = j.get<std::string>();
  if (str == "FAILED") {
    d = ActionStatus::FAILED;
  } else if (str == "FINISHED") {
    d = ActionStatus::FINISHED;
  } else if (str == "INITIALIZING") {
    d = ActionStatus::INITIALIZING;
  } else if (str == "PAUSED") {
    d = ActionStatus::PAUSED;
  } else if (str == "RUNNING") {
    d = ActionStatus::RUNNING;
  } else if (str == "WAITING") {
    d = ActionStatus::WAITING;
  }
}

void to_json(json &j, const ActionState &d) {
  if (d.actionDescription.has_value()) {
    j["actionDescription"] = *d.actionDescription;
  }
  j["actionId"] = d.actionId;
  j["actionStatus"] = d.actionStatus;
  if (d.actionType.has_value()) {
    j["actionType"] = *d.actionType;
  }
  if (d.resultDescription.has_value()) {
    j["resultDescription"] = *d.resultDescription;
  }
}
void from_json(const json &j, ActionState &d) {
  if (j.contains("actionDescription")) {
    d.actionDescription = j.at("actionDescription");
  }
  d.actionId = j.at("actionId");
  d.actionStatus = j.at("actionStatus");
  if (j.contains("actionType")) {
    d.actionType = j.at("actionType");
  }
  if (j.contains("resultDescription")) {
    d.resultDescription = j.at("resultDescription");
  }
}

void to_json(json &j, const BatteryState &d) {
  j["batteryCharge"] = d.batteryCharge;
  if (d.batteryHealth.has_value()) {
    j["batteryHealth"] = *d.batteryHealth;
  }
  if (d.batteryVoltage.has_value()) {
    j["batteryVoltage"] = *d.batteryVoltage;
  }
  j["charging"] = d.charging;
  if (d.reach.has_value()) {
    j["reach"] = *d.reach;
  }
}
void from_json(const json &j, BatteryState &d) {
  d.batteryCharge = j.at("batteryCharge");
  if (j.contains("batteryHealth")) {
    d.batteryHealth = j.at("batteryHealth");
  }
  if (j.contains("batteryVoltage")) {
    d.batteryVoltage = j.at("batteryVoltage");
  }
  d.charging = j.at("charging");
  if (j.contains("reach")) {
    d.reach = j.at("reach");
  }
}

void to_json(json &j, const EdgeState &d) {
  if (d.edgeDescription.has_value()) {
    j["edgeDescription"] = *d.edgeDescription;
  }
  j["edgeId"] = d.edgeId;
  j["released"] = d.released;
  j["sequenceId"] = d.sequenceId;
  if (d.trajectory.has_value()) {
    j["trajectory"] = *d.trajectory;
  }
}
void from_json(const json &j, EdgeState &d) {
  if (j.contains("edgeDescription")) {
    d.edgeDescription = j.at("edgeDescription");
  }
  d.edgeId = j.at("edgeId");
  d.released = j.at("released");
  d.sequenceId = j.at("sequenceId");
  if (j.contains("trajectory")) {
    d.trajectory = j.at("trajectory");
  }
}

void to_json(json &j, const NodeState &d) {
  if (d.nodeDescription.has_value()) {
    j["nodeDescription"] = *d.nodeDescription;
  }
  j["nodeId"] = d.nodeId;
  j["released"] = d.released;
  j["sequenceId"] = d.sequenceId;
  if (d.nodePosition.has_value()) {
    j["nodePosition"] = *d.nodePosition;
  }
}
void from_json(const json &j, NodeState &d) {
  if (j.contains("nodeDescription")) {
    d.nodeDescription = j.at("nodeDescription");
  }
  d.nodeId = j.at("nodeId");
  d.released = j.at("released");
  d.sequenceId = j.at("sequenceId");
  if (j.contains("nodePosition")) {
    d.nodePosition = j.at("nodePosition");
  }
}

void to_json(json &j, const ErrorReference &d) {
  j["referenceKey"] = d.referenceKey;
  j["referenceValue"] = d.referenceValue;
}
void from_json(const json &j, ErrorReference &d) {
  d.referenceKey = j.at("referenceKey");
  d.referenceValue = j.at("referenceValue");
}

void to_json(json &j, const Error &d) {
  if (d.errorDescription.has_value()) {
    j["errorDescription"] = *d.errorDescription;
  }
  if (d.errorHint.has_value()) {
    j["errorHint"] = *d.errorHint;
  }
  j["errorLevel"] = d.errorLevel;
  j["errorType"] = d.errorType;
  if (d.errorReferences.has_value()) {
    j["errorReferences"] = *d.errorReferences;
  }
}

void to_json(json &j, const ErrorLevel &d) {
  switch (d) {
    case vda5050::ErrorLevel::WARNING:
      j = "WARNING";
      break;
    case vda5050::ErrorLevel::FATAL:
      j = "FATAL";
      break;
    default:
      j = "UNKNOWN";
      break;
  }
}
void from_json(const json &j, ErrorLevel &d) {
  auto str = j.get<std::string>();
  if (str == "WARNING") {
    d = vda5050::ErrorLevel::WARNING;
  } else if (str == "FATAL") {
    d = vda5050::ErrorLevel::FATAL;
  }
}

void from_json(const json &j, Error &d) {
  if (j.contains("errorDescription")) {
    d.errorDescription = j.at("errorDescription");
  }
  if (j.contains("errorHint")) {
    d.errorHint = j.at("errorHint");
  }
  d.errorLevel = j.at("errorLevel");
  d.errorType = j.at("errorType");
  if (j.contains("errorReferences")) {
    std::vector<vda5050::ErrorReference> val = j.at("errorReferences");
    d.errorReferences = val;
  }
}

void to_json(json &j, const InfoReference &d) {
  j["referenceKey"] = d.referenceKey;
  j["referenceValue"] = d.referenceValue;
}
void from_json(const json &j, InfoReference &d) {
  d.referenceKey = j.at("referenceKey");
  d.referenceValue = j.at("referenceValue");
}

void to_json(json &j, const InfoLevel &d) {
  switch (d) {
    case vda5050::InfoLevel::DEBUG:
      j = "DEBUG";
      break;
    case vda5050::InfoLevel::INFO:
      j = "INFO";
      break;
    default:
      j = "UNKNOWN";
      break;
  }
}
void from_json(const json &j, InfoLevel &d) {
  auto str = j.get<std::string>();
  if (str == "DEBUG") {
    d = vda5050::InfoLevel::DEBUG;
  } else if (str == "INFO") {
    d = vda5050::InfoLevel::INFO;
  }
}

void to_json(json &j, const Info &d) {
  if (d.infoDescription.has_value()) {
    j["infoDescription"] = *d.infoDescription;
  }
  j["infoLevel"] = d.infoLevel;
  j["infoType"] = d.infoType;
  if (d.infoReferences.has_value()) {
    j["infoReferences"] = *d.infoReferences;
  }
}
void from_json(const json &j, Info &d) {
  if (j.contains("infoDescription")) {
    d.infoDescription = j.at("infoDescription");
  }
  d.infoLevel = j.at("infoLevel");
  d.infoType = j.at("infoType");
  if (j.contains("infoReferences")) {
    std::vector<vda5050::InfoReference> val = j.at("infoReferences");
    d.infoReferences = val;
  }
}

void to_json(json &j, const BoundingBoxReference &d) {
  j["x"] = d.x;
  j["y"] = d.y;
  j["z"] = d.z;
  if (d.theta.has_value()) {
    j["theta"] = *d.theta;
  }
}
void from_json(const json &j, BoundingBoxReference &d) {
  d.x = j.at("x");
  d.y = j.at("y");
  d.z = j.at("z");
  if (j.contains("theta")) {
    d.theta = j.at("theta");
  }
}

void to_json(json &j, const LoadDimensions &d) {
  if (d.height.has_value()) {
    j["height"] = *d.height;
  }
  j["width"] = d.width;
  j["length"] = d.length;
}
void from_json(const json &j, LoadDimensions &d) {
  if (j.contains("height")) {
    d.height = j.at("height");
  }
  d.width = j.at("width");
  d.length = j.at("length");
}

void to_json(json &j, const Load &d) {
  if (d.boundingBoxReference.has_value()) {
    j["boundingBoxReference"] = *d.boundingBoxReference;
  }
  if (d.loadDimensions.has_value()) {
    j["loadDimensions"] = *d.loadDimensions;
  }
  if (d.loadId.has_value()) {
    j["loadId"] = *d.loadId;
  }
  if (d.loadPosition.has_value()) {
    j["loadPosition"] = *d.loadPosition;
  }
  if (d.loadType.has_value()) {
    j["loadType"] = *d.loadType;
  }
  if (d.weight.has_value()) {
    j["weight"] = *d.weight;
  }
}
void from_json(const json &j, Load &d) {
  if (j.contains("boundingBoxReference")) {
    d.boundingBoxReference = j.at("boundingBoxReference");
  }
  if (j.contains("loadDimensions")) {
    d.loadDimensions = j.at("loadDimensions");
  }
  if (j.contains("loadId")) {
    d.loadId = j.at("loadId");
  }
  if (j.contains("loadPosition")) {
    d.loadPosition = j.at("loadPosition");
  }
  if (j.contains("loadType")) {
    d.loadType = j.at("loadType");
  }
  if (j.contains("weight")) {
    d.weight = j.at("weight");
  }
}

void to_json(json &j, const SafetyState &d) {
  j["eStop"] = d.eStop;
  j["fieldViolation"] = d.fieldViolation;
}
void from_json(const json &j, SafetyState &d) {
  d.eStop = j.at("eStop");
  d.fieldViolation = j.at("fieldViolation");
}

void to_json(json &j, const MapStatus &d) {
  switch (d) {
    case MapStatus::DISABLED:
      j = "DISABLED";
      break;
    case MapStatus::ENABLED:
      j = "ENABLED";
      break;
    default:
      j = "UNKNOWN";
      break;
  }
}
void from_json(const json &j, MapStatus &d) {
  auto str = j.get<std::string>();
  if (str == "DISABLED") {
    d = MapStatus::DISABLED;
  } else if (str == "ENABLED") {
    d = MapStatus::ENABLED;
  }
}

void to_json(json &j, const Map &d) {
  j["mapId"] = d.mapId;
  j["mapVersion"] = d.mapVersion;
  if (d.mapDescription.has_value()) {
    j["mapDescription"] = *d.mapDescription;
  }
  j["mapStatus"] = d.mapStatus;
}
void from_json(const json &j, Map &d) {
  d.mapId = j.at("mapId");
  d.mapVersion = j.at("mapVersion");
  if (j.contains("mapDescription")) {
    d.mapDescription = j.at("mapDescription");
  }
  d.mapStatus = j.at("mapStatus");
}

void to_json(json &j, const State &d) {
  to_json(j, d.header);
  j["actionStates"] = d.actionStates;
  if (d.agvPosition.has_value()) {
    j["agvPosition"] = *d.agvPosition;
  }
  j["batteryState"] = d.batteryState;
  if (d.distanceSinceLastNode.has_value()) {
    j["distanceSinceLastNode"] = *d.distanceSinceLastNode;
  }
  j["driving"] = d.driving;
  j["edgeStates"] = d.edgeStates;
  j["errors"] = d.errors;
  if (d.information.has_value()) {
    j["information"] = *d.information;
  }
  j["lastNodeId"] = d.lastNodeId;
  j["lastNodeSequenceId"] = d.lastNodeSequenceId;
  if (d.loads.has_value()) {
    j["loads"] =
        *d.loads;  // Keep possible "null" loads since they could represent an arbitrary load
  }
  if (d.maps.has_value()) {
    j["maps"] = *d.maps;
  }
  if (d.newBaseRequest.has_value()) {
    j["newBaseRequest"] = *d.newBaseRequest;
  }
  j["nodeStates"] = d.nodeStates;
  j["operatingMode"] = d.operatingMode;
  j["orderId"] = d.orderId;
  j["orderUpdateId"] = d.orderUpdateId;
  if (d.paused.has_value()) {
    j["paused"] = *d.paused;
  }
  j["safetyState"] = d.safetyState;
  if (d.velocity.has_value()) {
    j["velocity"] = *d.velocity;
  }
  if (d.zoneSetId.has_value()) {
    j["zoneSetId"] = *d.zoneSetId;
  }
}

void from_json(const json &j, State &d) {
  from_json(j, d.header);
  d.actionStates = j.at("actionStates").get<std::vector<ActionState>>();
  if (j.contains("agvPosition")) {
    d.agvPosition = j.at("agvPosition");
  }
  d.batteryState = j.at("batteryState");
  if (j.contains("distanceSinceLastNode")) {
    d.distanceSinceLastNode = j.at("distanceSinceLastNode");
  }
  d.driving = j.at("driving");
  d.edgeStates = j.at("edgeStates").get<std::vector<EdgeState>>();
  d.errors = j.at("errors").get<std::vector<Error>>();
  if (j.contains("information")) {
    d.information = j.at("information").get<std::vector<Info>>();
  }
  d.lastNodeId = j.at("lastNodeId");
  d.lastNodeSequenceId = j.at("lastNodeSequenceId");
  if (j.contains("loads")) {
    d.loads = j.at("loads").get<std::vector<Load>>();
  }
  if (j.contains("maps")) {
    d.maps = j.at("maps").get<std::vector<Map>>();
  }
  if (j.contains("newBaseRequest")) {
    d.newBaseRequest = j.at("newBaseRequest");
  }
  d.nodeStates = j.at("nodeStates").get<std::vector<NodeState>>();
  d.operatingMode = j.at("operatingMode");
  d.orderId = j.at("orderId");
  d.orderUpdateId = j.at("orderUpdateId");
  if (j.contains("paused")) {
    d.paused = j.at("paused");
  }
  d.safetyState = j.at("safetyState");
  if (j.contains("velocity")) {
    d.velocity = j.at("velocity");
  }
  if (j.contains("zoneSetId")) {
    d.zoneSetId = j.at("zoneSetId");
  }
}

void to_json(json &j, const TypeSpecification &d) {
  j["seriesName"] = d.seriesName;
  if (d.seriesDescription.has_value()) {
    j["seriesDescription"] = *d.seriesDescription;
  }
  j["agvKinematic"] = d.agvKinematic;
  j["agvClass"] = d.agvClass;
  j["maxLoadMass"] = d.maxLoadMass;
  j["localizationTypes"] = d.localizationTypes;
  j["navigationTypes"] = d.navigationTypes;
}
void from_json(const json &j, TypeSpecification &d) {
  d.seriesName = j.at("seriesName");
  if (j.contains("seriesDescription")) {
    d.seriesDescription = j.at("seriesDescription");
  }
  d.agvKinematic = j.at("agvKinematic");
  d.agvClass = j.at("agvClass");
  d.maxLoadMass = j.at("maxLoadMass");
  d.localizationTypes = j.at("localizationTypes").get<std::vector<std::string>>();
  d.navigationTypes = j.at("navigationTypes").get<std::vector<std::string>>();
}

void to_json(json &j, const OptionalParameter &d) {
  j["parameter"] = d.parameter;
  j["support"] = d.support;
  if (d.description.has_value()) {
    j["description"] = *d.description;
  }
}
void from_json(const json &j, OptionalParameter &d) {
  d.parameter = j.at("parameter");
  d.support = j.at("support");
  if (j.contains("description")) {
    d.description = j.at("description");
  }
}

void to_json(json &j, const ProtocolFeatures &d) {
  j["optionalParameters"] = d.optionalParameters;
  j["agvActions"] = d.agvActions;
}
void from_json(const json &j, ProtocolFeatures &d) {
  d.optionalParameters = j.at("optionalParameters").get<std::vector<OptionalParameter>>();
  d.agvActions = j.at("agvActions").get<std::vector<AgvAction>>();
}

void to_json(json &j, const PhysicalParameters &d) {
  j["speedMin"] = d.speedMin;
  j["speedMax"] = d.speedMax;
  if (d.angularSpeedMin.has_value()) {
    j["angularSpeedMin"] = *d.angularSpeedMin;
  }
  if (d.angularSpeedMax.has_value()) {
    j["angularSpeedMax"] = *d.angularSpeedMax;
  }
  j["accelerationMax"] = d.accelerationMax;
  j["decelerationMax"] = d.decelerationMax;
  j["heightMin"] = d.heightMin;
  j["heightMax"] = d.heightMax;
  j["width"] = d.width;
  j["length"] = d.length;
}
void from_json(const json &j, PhysicalParameters &d) {
  d.speedMin = j.at("speedMin");
  d.speedMax = j.at("speedMax");
  if (j.contains("angularSpeedMin")) {
    d.angularSpeedMin = j.at("angularSpeedMin");
  }
  if (j.contains("angularSpeedMax")) {
    d.angularSpeedMax = j.at("angularSpeedMax");
  }
  d.accelerationMax = j.at("accelerationMax");
  d.decelerationMax = j.at("decelerationMax");
  d.heightMin = j.at("heightMin");
  d.heightMax = j.at("heightMax");
  d.width = j.at("width");
  d.length = j.at("length");
}

void to_json(json &j, const Network &d) {
  if (d.dnsServers.has_value()) {
    j["dnsServers"] = *d.dnsServers;
  }
  if (d.ntpServers.has_value()) {
    j["ntpServers"] = *d.ntpServers;
  }
  if (d.localIpAddress.has_value()) {
    j["localIpAddress"] = *d.localIpAddress;
  }
  if (d.netmask.has_value()) {
    j["netmask"] = *d.netmask;
  }
  if (d.defaultGateway.has_value()) {
    j["defaultGateway"] = *d.defaultGateway;
  }
}
void from_json(const json &j, Network &d) {
  if (j.contains("dnsServers")) {
    d.dnsServers = j.at("dnsServers").get<std::vector<std::string>>();
  }
  if (j.contains("ntpServers")) {
    d.ntpServers = j.at("ntpServers").get<std::vector<std::string>>();
  }
  if (j.contains("localIpAddress")) {
    d.localIpAddress = j.at("localIpAddress");
  }
  if (j.contains("netmask")) {
    d.netmask = j.at("netmask");
  }
  if (j.contains("defaultGateway")) {
    d.defaultGateway = j.at("defaultGateway");
  }
}

void to_json(json &j, const VersionInfo &d) {
  j["key"] = d.key;
  j["value"] = d.value;
}
void from_json(const json &j, VersionInfo &d) {
  d.key = j.at("key");
  d.value = j.at("value");
}

void to_json(json &j, const VehicleConfig &d) {
  if (d.versions.has_value()) {
    j["versions"] = *d.versions;
  }
  if (d.network.has_value()) {
    j["network"] = *d.network;
  }
}
void from_json(const json &j, VehicleConfig &d) {
  if (j.contains("versions")) {
    d.versions = j.at("versions").get<std::vector<VersionInfo>>();
  }
  if (j.contains("network")) {
    d.network = j.at("network");
  }
}

void to_json(json &j, const ActionParameterFactsheet &d) {
  j["key"] = d.key;
  j["valueDataType"] = d.valueDataType;
  if (d.description.has_value()) {
    j["description"] = *d.description;
  }
  if (d.isOptional.has_value()) {
    j["isOptional"] = *d.isOptional;
  }
}
void from_json(const json &j, ActionParameterFactsheet &d) {
  d.key = j.at("key");
  d.valueDataType = j.at("valueDataType");
  if (j.contains("description")) {
    d.description = j.at("description");
  }
  if (j.contains("isOptional")) {
    d.isOptional = j.at("isOptional");
  }
}

void to_json(json &j, const AgvAction &d) {
  j["actionType"] = d.actionType;
  if (d.actionDescription.has_value()) {
    j["actionDescription"] = *d.actionDescription;
  }
  j["actionScopes"] = d.actionScopes;
  if (d.actionParameters.has_value()) {
    j["actionParameters"] = *d.actionParameters;
  }
  if (d.resultDescription.has_value()) {
    j["resultDescription"] = *d.resultDescription;
  }
  if (d.blockingTypes.has_value()) {
    j["blockingTypes"] = *d.blockingTypes;
  }
}
void from_json(const json &j, AgvAction &d) {
  d.actionType = j.at("actionType");
  if (j.contains("actionDescription")) {
    d.actionDescription = j.at("actionDescription");
  }
  d.actionScopes = j.at("actionScopes").get<std::vector<ActionScope>>();
  if (j.contains("actionParameters")) {
    d.actionParameters = j.at("actionParameters").get<std::vector<ActionParameterFactsheet>>();
  }
  if (j.contains("resultDescription")) {
    d.resultDescription = j.at("resultDescription");
  }
  if (j.contains("blockingTypes")) {
    d.blockingTypes = j.at("blockingTypes").get<std::vector<BlockingType>>();
  }
}

void to_json(json &j, const WheelDefinition &d) {
  j["type"] = d.type;
  j["isActiveDriven"] = d.isActiveDriven;
  j["isActiveSteered"] = d.isActiveSteered;
  j["position"] = d.position;
  j["diameter"] = d.diameter;
  j["width"] = d.width;
  if (d.centerDisplacement.has_value()) {
    j["centerDisplacement"] = *d.centerDisplacement;
  }
  if (d.constraints.has_value()) {
    j["constraints"] = *d.constraints;
  }
}
void from_json(const json &j, WheelDefinition &d) {
  d.type = j.at("type");
  d.isActiveDriven = j.at("isActiveDriven");
  d.isActiveSteered = j.at("isActiveSteered");
  d.position = j.at("position");
  d.diameter = j.at("diameter");
  d.width = j.at("width");
  if (j.contains("centerDisplacement")) {
    d.centerDisplacement = j.at("centerDisplacement");
  }
  if (j.contains("constraints")) {
    d.constraints = j.at("constraints");
  }
}

void to_json(json &j, const AgvGeometry &d) {
  if (d.wheelDefinitions.has_value()) {
    j["wheelDefinitions"] = *d.wheelDefinitions;
  }
  if (d.envelopes2d.has_value()) {
    j["envelopes2d"] = *d.envelopes2d;
  }
  if (d.envelopes3d.has_value()) {
    j["envelopes3d"] = *d.envelopes3d;
  }
}
void from_json(const json &j, AgvGeometry &d) {
  if (j.contains("wheelDefinitions")) {
    d.wheelDefinitions = j.at("wheelDefinitions").get<std::vector<WheelDefinition>>();
  }
  if (j.contains("envelopes2d")) {
    d.envelopes2d = j.at("envelopes2d").get<std::vector<Envelope2d>>();
  }
  if (j.contains("envelopes3d")) {
    d.envelopes3d = j.at("envelopes3d").get<std::vector<Envelope3d>>();
  }
}

void to_json(json &j, const LoadSpecification &d) {
  if (d.loadPositions.has_value()) {
    j["loadPositions"] = *d.loadPositions;
  }
  if (d.loadSets.has_value()) {
    j["loadSets"] = *d.loadSets;
  }
}
void from_json(const json &j, LoadSpecification &d) {
  if (j.contains("loadPositions")) {
    d.loadPositions = j.at("loadPositions").get<std::vector<std::string>>();
  }
  if (j.contains("loadSets")) {
    d.loadSets = j.at("loadSets").get<std::vector<LoadSet>>();
  }
}

void to_json(json &j, const MaxStringLens &d) {
  if (d.msgLen.has_value()) {
    j["msgLen"] = *d.msgLen;
  }
  if (d.topicSerialLen.has_value()) {
    j["topicSerialLen"] = *d.topicSerialLen;
  }
  if (d.topicElemLen.has_value()) {
    j["topicElemLen"] = *d.topicElemLen;
  }
  if (d.idLen.has_value()) {
    j["idLen"] = *d.idLen;
  }
  if (d.idNumericalOnly.has_value()) {
    j["idNumericalOnly"] = *d.idNumericalOnly;
  }
  if (d.enumLen.has_value()) {
    j["enumLen"] = *d.enumLen;
  }
  if (d.loadIdLen.has_value()) {
    j["loadIdLen"] = *d.loadIdLen;
  }
}
void from_json(const json &j, MaxStringLens &d) {
  if (j.contains("msgLen")) {
    d.msgLen = j.at("msgLen");
  }
  if (j.contains("topicSerialLen")) {
    d.topicSerialLen = j.at("topicSerialLen");
  }
  if (j.contains("topicElemLen")) {
    d.topicElemLen = j.at("topicElemLen");
  }
  if (j.contains("idLen")) {
    d.idLen = j.at("idLen");
  }
  if (j.contains("idNumericalOnly")) {
    d.idNumericalOnly = j.at("idNumericalOnly");
  }
  if (j.contains("enumLen")) {
    d.enumLen = j.at("enumLen");
  }
  if (j.contains("loadIdLen")) {
    d.loadIdLen = j.at("loadIdLen");
  }
}

void to_json(json &j, const MaxArrayLens &d) {
  if (d.orderNodes.has_value()) {
    j["order.nodes"] = *d.orderNodes;
  }
  if (d.orderEdges.has_value()) {
    j["order.edges"] = *d.orderEdges;
  }
  if (d.nodeActions.has_value()) {
    j["node.actions"] = *d.nodeActions;
  }
  if (d.edgeActions.has_value()) {
    j["edge.actions"] = *d.edgeActions;
  }
  if (d.actionActionsParameters.has_value()) {
    j["action.actionsParameters"] = *d.actionActionsParameters;
  }
  if (d.instantActions.has_value()) {
    j["instant.actions"] = *d.instantActions;
  }
  if (d.trajectoryKnotVector.has_value()) {
    j["trajectory.knotVector"] = *d.trajectoryKnotVector;
  }
  if (d.trajectoryControlPoints.has_value()) {
    j["trajectory.controlPoints"] = *d.trajectoryControlPoints;
  }
  if (d.stateNodeStates.has_value()) {
    j["state.nodeStates"] = *d.stateNodeStates;
  }
  if (d.stateEdgeStates.has_value()) {
    j["state.edgeStates"] = *d.stateEdgeStates;
  }
  if (d.stateLoads.has_value()) {
    j["state.loads"] = *d.stateLoads;
  }
  if (d.stateActionStates.has_value()) {
    j["state.actionStates"] = *d.stateActionStates;
  }
  if (d.stateErrors.has_value()) {
    j["state.errors"] = *d.stateErrors;
  }
  if (d.stateInformations.has_value()) {
    j["state.informations"] = *d.stateInformations;
  }
  if (d.errorErrorReferences.has_value()) {
    j["error.errorReferences"] = *d.errorErrorReferences;
  }
  if (d.informationsInfoReferences.has_value()) {
    j["informations.infoReferences"] = *d.informationsInfoReferences;
  }
}
void from_json(const json &j, MaxArrayLens &d) {
  if (j.contains("order.nodes")) {
    d.orderNodes = j.at("order.nodes");
  }
  if (j.contains("order.edges")) {
    d.orderEdges = j.at("order.edges");
  }
  if (j.contains("node.actions")) {
    d.nodeActions = j.at("node.actions");
  }
  if (j.contains("edge.actions")) {
    d.edgeActions = j.at("edge.actions");
  }
  if (j.contains("action.actionsParameters")) {
    d.actionActionsParameters = j.at("action.actionsParameters");
  }
  if (j.contains("instant.actions")) {
    d.instantActions = j.at("instant.actions");
  }
  if (j.contains("trajectory.knotVector")) {
    d.trajectoryKnotVector = j.at("trajectory.knotVector");
  }
  if (j.contains("trajectory.controlPoints")) {
    d.trajectoryControlPoints = j.at("trajectory.controlPoints");
  }
  if (j.contains("state.nodeStates")) {
    d.stateNodeStates = j.at("state.nodeStates");
  }
  if (j.contains("state.edgeStates")) {
    d.stateEdgeStates = j.at("state.edgeStates");
  }
  if (j.contains("state.loads")) {
    d.stateLoads = j.at("state.loads");
  }
  if (j.contains("state.actionStates")) {
    d.stateActionStates = j.at("state.actionStates");
  }
  if (j.contains("state.errors")) {
    d.stateErrors = j.at("state.errors");
  }
  if (j.contains("state.informations")) {
    d.stateInformations = j.at("state.informations");
  }
  if (j.contains("error.errorReferences")) {
    d.errorErrorReferences = j.at("error.errorReferences");
  }
  if (j.contains("informations.infoReferences")) {
    d.informationsInfoReferences = j.at("informations.infoReferences");
  }
}

void to_json(json &j, const LoadSet &d) {
  j["setName"] = d.setName;
  j["loadType"] = d.loadType;
  if (d.loadPositions.has_value()) {
    j["loadPositions"] = *d.loadPositions;
  }
  if (d.boundingBoxReference.has_value()) {
    j["boundingBoxReference"] = *d.boundingBoxReference;
  }
  if (d.loadDimensions.has_value()) {
    j["loadDimensions"] = *d.loadDimensions;
  }
  if (d.maxWeight.has_value()) {
    j["maxWeight"] = *d.maxWeight;
  }
  if (d.minLoadhandlingHeight.has_value()) {
    j["minLoadhandlingHeight"] = *d.minLoadhandlingHeight;
  }
  if (d.maxLoadhandlingHeight.has_value()) {
    j["maxLoadhandlingHeight"] = *d.maxLoadhandlingHeight;
  }
  if (d.minLoadhandlingDepth.has_value()) {
    j["minLoadhandlingDepth"] = *d.minLoadhandlingDepth;
  }
  if (d.maxLoadhandlingDepth.has_value()) {
    j["maxLoadhandlingDepth"] = *d.maxLoadhandlingDepth;
  }
  if (d.minLoadhandlingTilt.has_value()) {
    j["minLoadhandlingTilt"] = *d.minLoadhandlingTilt;
  }
  if (d.maxLoadhandlingTilt.has_value()) {
    j["maxLoadhandlingTilt"] = *d.maxLoadhandlingTilt;
  }
  if (d.agvSpeedLimit.has_value()) {
    j["agvSpeedLimit"] = *d.agvSpeedLimit;
  }
  if (d.agvAccelerationLimit.has_value()) {
    j["agvAccelerationLimit"] = *d.agvAccelerationLimit;
  }
  if (d.agvDecelerationLimit.has_value()) {
    j["agvDecelerationLimit"] = *d.agvDecelerationLimit;
  }
  if (d.pickTime.has_value()) {
    j["pickTime"] = *d.pickTime;
  }
  if (d.dropTime.has_value()) {
    j["dropTime"] = *d.dropTime;
  }
  if (d.description.has_value()) {
    j["description"] = *d.description;
  }
}
void from_json(const json &j, LoadSet &d) {
  d.setName = j.at("setName");
  d.loadType = j.at("loadType");
  if (j.contains("loadPositions")) {
    d.loadPositions = j.at("loadPositions").get<std::vector<std::string>>();
  }
  if (j.contains("boundingBoxReference")) {
    d.boundingBoxReference = j.at("boundingBoxReference");
  }
  if (j.contains("loadDimensions")) {
    d.loadDimensions = j.at("loadDimensions");
  }
  if (j.contains("maxWeight")) {
    d.maxWeight = j.at("maxWeight");
  }
  if (j.contains("minLoadhandlingHeight")) {
    d.minLoadhandlingHeight = j.at("minLoadhandlingHeight");
  }
  if (j.contains("maxLoadhandlingHeight")) {
    d.maxLoadhandlingHeight = j.at("maxLoadhandlingHeight");
  }
  if (j.contains("minLoadhandlingDepth")) {
    d.minLoadhandlingDepth = j.at("minLoadhandlingDepth");
  }
  if (j.contains("maxLoadhandlingDepth")) {
    d.maxLoadhandlingDepth = j.at("maxLoadhandlingDepth");
  }
  if (j.contains("minLoadhandlingTilt")) {
    d.minLoadhandlingTilt = j.at("minLoadhandlingTilt");
  }
  if (j.contains("maxLoadhandlingTilt")) {
    d.maxLoadhandlingTilt = j.at("maxLoadhandlingTilt");
  }
  if (j.contains("agvSpeedLimit")) {
    d.agvSpeedLimit = j.at("agvSpeedLimit");
  }
  if (j.contains("agvAccelerationLimit")) {
    d.agvAccelerationLimit = j.at("agvAccelerationLimit");
  }
  if (j.contains("agvDecelerationLimit")) {
    d.agvDecelerationLimit = j.at("agvDecelerationLimit");
  }
  if (j.contains("pickTime")) {
    d.pickTime = j.at("pickTime");
  }
  if (j.contains("dropTime")) {
    d.dropTime = j.at("dropTime");
  }
  if (j.contains("description")) {
    d.description = j.at("description");
  }
}

void to_json(json &j, const Timing &d) {
  j["minOrderInterval"] = d.minOrderInterval;
  j["minStateInterval"] = d.minStateInterval;
  if (d.defaultStateInterval.has_value()) {
    j["defaultStateInterval"] = *d.defaultStateInterval;
  }
  if (d.visualizationInterval.has_value()) {
    j["visualizationInterval"] = *d.visualizationInterval;
  }
}
void from_json(const json &j, Timing &d) {
  d.minOrderInterval = j.at("minOrderInterval");
  d.minStateInterval = j.at("minStateInterval");
  if (j.contains("defaultStateInterval")) {
    d.defaultStateInterval = j.at("defaultStateInterval");
  }
  if (j.contains("visualizationInterval")) {
    d.visualizationInterval = j.at("visualizationInterval");
  }
}

void to_json(json &j, const Envelope2d &d) {
  j["set"] = d.set;
  j["polygonPoints"] = d.polygonPoints;
  if (d.description.has_value()) {
    j["description"] = *d.description;
  }
}
void from_json(const json &j, Envelope2d &d) {
  d.set = j.at("set");
  d.polygonPoints = j.at("polygonPoints").get<std::vector<PolygonPoint>>();
  if (j.contains("description")) {
    d.description = j.at("description");
  }
}

void to_json(json &j, const Envelope3d &d) {
  j["set"] = d.set;
  j["format"] = d.format;
  j["data"] = d.data;
  if (d.url.has_value()) {
    j["url"] = *d.url;
  }
  if (d.description.has_value()) {
    j["description"] = *d.description;
  }
}
void from_json(const json &j, Envelope3d &d) {
  d.set = j.at("set");
  d.format = j.at("format");
  d.data = j.at("data");
  if (j.contains("url")) {
    d.url = j.at("url");
  }
  if (j.contains("description")) {
    d.description = j.at("description");
  }
}

void to_json(json &j, const ProtocolLimits &d) {
  j["maxStringLens"] = d.maxStringLens;  // Can be null if the object is {}, not setting it is no
                                         // option, since this is a required field
  j["maxArrayLens"] = d.maxArrayLens;    // See comment above
  j["timing"] = d.timing;
}
void from_json(const json &j, ProtocolLimits &d) {
  d.maxStringLens = j.at("maxStringLens");
  d.maxArrayLens = j.at("maxArrayLens");
  d.timing = j.at("timing");
}

void to_json(json &j, const PolygonPoint &d) {
  j["x"] = d.x;
  j["y"] = d.y;
}
void from_json(const json &j, PolygonPoint &d) {
  d.x = j.at("x");
  d.y = j.at("y");
}

void to_json(json &j, const Position &d) {
  j["x"] = d.x;
  j["y"] = d.y;
  if (d.theta.has_value()) {
    j["theta"] = *d.theta;
  }
}
void from_json(const json &j, Position &d) {
  d.x = j.at("x");
  d.y = j.at("y");
  if (j.contains("theta")) {
    d.theta = j.at("theta");
  }
}

void to_json(json &j, const AgvFactsheet &d) {
  to_json(j, d.header);
  j["typeSpecification"] = d.typeSpecification;
  j["physicalParameters"] = d.physicalParameters;
  j["protocolLimits"] = d.protocolLimits;
  j["protocolFeatures"] = d.protocolFeatures;
  j["agvGeometry"] = d.agvGeometry;  // Can be null if the object is {}, not setting it is no
                                     // option, since this is a required field
  j["loadSpecification"] = d.loadSpecification;  // See comment above
  if (d.vehicleConfig.has_value()) {
    j["vehicleConfig"] = *d.vehicleConfig;
  }
}
void from_json(const json &j, AgvFactsheet &d) {
  from_json(j, d.header);
  d.typeSpecification = j.at("typeSpecification");
  d.physicalParameters = j.at("physicalParameters");
  d.protocolLimits = j.at("protocolLimits");
  d.protocolFeatures = j.at("protocolFeatures");
  d.agvGeometry = j.at("agvGeometry");
  d.loadSpecification = j.at("loadSpecification");
  if (j.contains("vehicleConfig")) {
    d.vehicleConfig = j.at("vehicleConfig");
  }
}

void to_json(json &j, const EStop &d) {
  switch (d) {
    case vda5050::EStop::AUTOACK:
      j = "AUTOACK";
      break;
    case vda5050::EStop::MANUAL:
      j = "MANUAL";
      break;
    case vda5050::EStop::REMOTE:
      j = "REMOTE";
      break;
    case vda5050::EStop::NONE:
      j = "NONE";
      break;
    default:
      j = "UNKNOWN";
      break;
  }
}
void from_json(const json &j, EStop &d) {
  auto str = j.get<std::string>();
  if (str == "AUTOACK") {
    d = vda5050::EStop::AUTOACK;
  } else if (str == "MANUAL") {
    d = vda5050::EStop::MANUAL;
  } else if (str == "REMOTE") {
    d = vda5050::EStop::REMOTE;
  } else if (str == "NONE") {
    d = vda5050::EStop::NONE;
  }
}

void to_json(json &j, const ValueDataType &d) {
  switch (d) {
    case vda5050::ValueDataType::BOOLEAN:
      j = "BOOL";
      break;
    case vda5050::ValueDataType::NUMBER:
      j = "NUMBER";
      break;
    case vda5050::ValueDataType::INTEGER:
      j = "INTEGER";
      break;
    case vda5050::ValueDataType::FLOAT:
      j = "FLOAT";
      break;
    case vda5050::ValueDataType::STRING:
      j = "STRING";
      break;
    case vda5050::ValueDataType::OBJECT:
      j = "OBJECT";
      break;
    case vda5050::ValueDataType::ARRAY:
      j = "ARRAY";
      break;
  }
}
void from_json(const json &j, ValueDataType &d) {
  auto str = j.get<std::string>();
  if (str == "BOOL") {
    d = vda5050::ValueDataType::BOOLEAN;
  } else if (str == "NUMBER") {
    d = vda5050::ValueDataType::NUMBER;
  } else if (str == "INTEGER") {
    d = vda5050::ValueDataType::INTEGER;
  } else if (str == "FLOAT") {
    d = vda5050::ValueDataType::FLOAT;
  } else if (str == "STRING") {
    d = vda5050::ValueDataType::STRING;
  } else if (str == "OBJECT") {
    d = vda5050::ValueDataType::OBJECT;
  } else if (str == "ARRAY") {
    d = vda5050::ValueDataType::ARRAY;
  }
}

void to_json(json &j, const Support &d) {
  switch (d) {
    case vda5050::Support::SUPPORTED:
      j = "SUPPORTED";
      break;
    case vda5050::Support::REQUIRED:
      j = "REQUIRED";
      break;
  }
}
void from_json(const json &j, Support &d) {
  auto str = j.get<std::string>();
  if (str == "SUPPORTED") {
    d = vda5050::Support::SUPPORTED;
  } else if (str == "REQUIRED") {
    d = vda5050::Support::REQUIRED;
  }
}

void to_json(json &j, const ActionScope &d) {
  switch (d) {
    case vda5050::ActionScope::INSTANT:
      j = "INSTANT";
      break;
    case vda5050::ActionScope::NODE:
      j = "NODE";
      break;
    case vda5050::ActionScope::EDGE:
      j = "EDGE";
      break;
  }
}
void from_json(const json &j, ActionScope &d) {
  auto str = j.get<std::string>();
  if (str == "INSTANT") {
    d = vda5050::ActionScope::INSTANT;
  } else if (str == "NODE") {
    d = vda5050::ActionScope::NODE;
  } else if (str == "EDGE") {
    d = vda5050::ActionScope::EDGE;
  }
}

void to_json(json &j, const WheelType &d) {
  switch (d) {
    case vda5050::WheelType::DRIVE:
      j = "DRIVE";
      break;
    case vda5050::WheelType::CASTER:
      j = "CASTER";
      break;
    case vda5050::WheelType::FIXED:
      j = "FIXED";
      break;
    case vda5050::WheelType::MECANUM:
      j = "MECANUM";
      break;
  }
}
void from_json(const json &j, WheelType &d) {
  auto str = j.get<std::string>();
  if (str == "DRIVE") {
    d = vda5050::WheelType::DRIVE;
  } else if (str == "CASTER") {
    d = vda5050::WheelType::CASTER;
  } else if (str == "FIXED") {
    d = vda5050::WheelType::FIXED;
  } else if (str == "MECANUM") {
    d = vda5050::WheelType::MECANUM;
  }
}

void to_json(json &j, const OperatingMode &d) {
  switch (d) {
    case vda5050::OperatingMode::AUTOMATIC:
      j = "AUTOMATIC";
      break;
    case vda5050::OperatingMode::MANUAL:
      j = "MANUAL";
      break;
    case vda5050::OperatingMode::SEMIAUTOMATIC:
      j = "SEMIAUTOMATIC";
      break;
    case vda5050::OperatingMode::SERVICE:
      j = "SERVICE";
      break;
    case vda5050::OperatingMode::TEACHIN:
      j = "TEACHIN";
      break;
    default:
      j = "UNKNOWN";
      break;
  }
}
void from_json(const json &j, OperatingMode &d) {
  auto str = j.get<std::string>();
  if (str == "AUTOMATIC") {
    d = vda5050::OperatingMode::AUTOMATIC;
  } else if (str == "MANUAL") {
    d = vda5050::OperatingMode::MANUAL;
  } else if (str == "SEMIAUTOMATIC") {
    d = vda5050::OperatingMode::SEMIAUTOMATIC;
  } else if (str == "SERVICE") {
    d = vda5050::OperatingMode::SERVICE;
  } else if (str == "TEACHIN") {
    d = vda5050::OperatingMode::TEACHIN;
  }
}

void to_json(json &j, const OrientationType &d) {
  switch (d) {
    case vda5050::OrientationType::GLOBAL:
      j = "GLOBAL";
      break;
    case vda5050::OrientationType::TANGENTIAL:
      [[fallthrough]];
    default:
      j = "TANGENTIAL";
      break;
  }
}
void from_json(const json &j, OrientationType &d) {
  auto str = j.get<std::string>();
  if (str == "TANGENTIAL") {
    d = vda5050::OrientationType::TANGENTIAL;
  } else if (str == "GLOBAL") {
    d = vda5050::OrientationType::GLOBAL;
  }
}

}  // namespace vda5050
