// vda5050_protocol.cpp - VDA5050 2.1 Schema validation

#include "vda5050_protocol.h"
#include "node_edge_info.h"
#include "iamr.h"
#include "ivcu.h"
#include <iostream>
#include <nlohmann/json.hpp>
#include <vda5050++/config.h>
#include <algorithm> 
#include <unordered_set>


void Vda5050Protocol::Vda5050MqttCallback::message_arrived(mqtt::const_message_ptr msg)
{
    std::cout << "[MQTT] Message arrived on topic: " << msg->get_topic() << std::endl;
    // 수신 로그
    if (proto_) 
    {
        proto_->logMessage("IN", msg->get_topic(), msg->to_string());
    }

    if (msg->get_topic() == proto_->order_topic_ || msg->get_topic() == proto_->instant_actions_topic) 
    {
        proto_->handleMessage(msg->to_string(), proto_->amr_);
    }
}

Vda5050Protocol::Vda5050Protocol(const AmrConfig& config)
    : running_(false), 
      config_(config),
      amr_(nullptr),
      state_header_id_(0),
      factsheet_header_id_(0),
      connection_header_id_(0),
      current_order_id_(""),
      current_order_update_id_(0),
      current_zone_set_id_(""),
      order_active_(false),
      has_order_rejection_error_(false),      
      order_rejection_error_type_(""),        
      order_rejection_error_description_("") 
{
    std::cout << "[Vda5050Protocol] Initialized" << std::endl;
}

Vda5050Protocol::~Vda5050Protocol() 
{
    stop();
}

void Vda5050Protocol::setAmr(IAmr* amr)
{
    amr_ = amr;
    std::cout << "[Vda5050Protocol] AMR instance set" << std::endl;
}

void Vda5050Protocol::setAgvId(const std::string& agv_id)
{
    agv_id_ = agv_id;
    std::cout << "[Vda5050Protocol] AGV ID set to: " << agv_id_ << std::endl;
}

void Vda5050Protocol::useDefaultConfig(const std::string& server_address)
{
    std::cout << "[Vda5050Protocol] Configuring MQTT server: " << server_address << std::endl;
    mqtt_server_uri_ = server_address;
    conn_opts_.set_clean_session(true);
    mqtt_client_ = std::make_unique<mqtt::async_client>(mqtt_server_uri_, agv_id_ + "_client");
    mqtt_callback_ = std::make_shared<Vda5050MqttCallback>(this);
    mqtt_client_->set_callback(*mqtt_callback_);

    // VDA5050 topic structure: agv/v2/{manufacturer}/{agv_id}/{topic}
    state_topic_ = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/state";
    order_topic_ = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/order";
    instant_actions_topic = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/instantActions";
    visualization_topic_ = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/visualization";
    connection_topic_ = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/connection";
    factsheet_topic_ = "agv/v2/ZENIXROBOTICS/" + agv_id_ + "/factsheet";
    
    std::cout << "[Vda5050Protocol] Topics configured:" << std::endl;
    std::cout << "  - State: " << state_topic_ << std::endl;
    std::cout << "  - Order: " << order_topic_ << std::endl;
    std::cout << "  - Instant Actions: " << instant_actions_topic << std::endl;
}

void Vda5050Protocol::start()
{
    try 
    {
        std::cout << "[Vda5050Protocol] Connecting to MQTT broker..." << std::endl;
        mqtt_client_->connect(conn_opts_)->wait();

        // Publish connection message
        std::string connect_msg = makeConnectMessage();
        auto pubmsg = mqtt::make_message(connection_topic_, connect_msg);
        pubmsg->set_qos(1);
        pubmsg->set_retained(false);
        mqtt_client_->publish(pubmsg);
        // 로그
        logMessage("OUT", connection_topic_, connect_msg);
        
        // Subscribe to order and instant actions topics
        mqtt_client_->subscribe(order_topic_, 1)->wait();
        mqtt_client_->subscribe(instant_actions_topic, 1)->wait();
        
        running_ = true;
        std::cout << "[Vda5050Protocol] MQTT connected & subscribed to: " 
                  << order_topic_ << " and " << instant_actions_topic << std::endl;

        // Start background thread (for periodic tasks if needed)
        publish_thread_ = std::thread([this]()
        {
            while (running_)
            {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                // Periodic tasks can be added here
                if (amr_)
                {
                    // 주기적으로 오더 완료 체크
                    checkOrderCompletion(amr_);
                }                
            }
        });        
    } 
    catch (const mqtt::exception& e) 
    {
        std::cerr << "[Vda5050Protocol] MQTT connection failed: " << e.what() << std::endl;
        throw;
    }
}

void Vda5050Protocol::stop()
{
    std::cout << "[Vda5050Protocol] Stopping protocol..." << std::endl;
    running_ = false;
    
    if (publish_thread_.joinable())
    {
        publish_thread_.join();
    }

    if (mqtt_client_ && mqtt_client_->is_connected()) 
    {
        try 
        {
            mqtt_client_->unsubscribe(order_topic_)->wait();
            mqtt_client_->disconnect()->wait();
            std::cout << "[Vda5050Protocol] MQTT disconnected" << std::endl;
        } 
        catch (const std::exception& e) 
        {
            std::cerr << "[Vda5050Protocol] Error during disconnect: " << e.what() << std::endl;
        }
    }    
}

std::string Vda5050Protocol::getCurrentTimestampISO8601()
{
    using namespace std::chrono;
    auto now = system_clock::now();
    std::time_t now_c = system_clock::to_time_t(now);
    auto milliseconds = duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::tm utc_tm;
#ifdef _WIN32
    gmtime_s(&utc_tm, &now_c);
#else
    gmtime_r(&now_c, &utc_tm);
#endif

    std::ostringstream oss;
    oss << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%S");
    oss << '.' << std::setfill('0') << std::setw(3) << milliseconds.count() << "Z";
    return oss.str();
}

// CONNECTION MESSAGE (connection.schema.json)
std::string Vda5050Protocol::makeConnectMessage()
{
    nlohmann::json conn_json;

    // Required fields per VDA5050 2.1 connection schema
    conn_json["headerId"] = connection_header_id_++;
    conn_json["timestamp"] = getCurrentTimestampISO8601();
    conn_json["version"] = "2.1.0";
    conn_json["manufacturer"] = "ZenixRobotics";
    conn_json["serialNumber"] = agv_id_;
    conn_json["connectionState"] = detectConnection();

    std::cout << "[Vda5050Protocol] Connection message: " << conn_json.dump() << std::endl;
    return conn_json.dump();
}

std::string Vda5050Protocol::detectConnection()
{
    // TODO: Implement actual connection state detection
    return "ONLINE";
}

// FACTSHEET MESSAGE (factsheet.schema.json)
// std::string Vda5050Protocol::makeFactsheetMessage()
// {
//     nlohmann::json factsheet_json;

//     // Required Header Fields
//     factsheet_json["headerId"] = factsheet_header_id_++;
//     factsheet_json["timestamp"] = getCurrentTimestampISO8601();
//     factsheet_json["version"] = "2.1.0";
//     factsheet_json["manufacturer"] = "ZenixRobotics";
//     factsheet_json["serialNumber"] = agv_id_;

//     // typeSpecification (Required)
//     nlohmann::json typeSpec;
//     typeSpec["seriesName"] = "ZenixAMR";
//     typeSpec["agvKinematic"] = "OMNI";
//     typeSpec["agvClass"] = "CARRIER";
//     typeSpec["maxLoadMass"] = 100.0;
//     typeSpec["localizationTypes"] = nlohmann::json::array({"RFID"});
//     typeSpec["navigationTypes"] = nlohmann::json::array({"AUTONOMOUS"});
//     typeSpec["seriesDescription"] = "Zenix Autonomous Mobile Robot Series";
//     factsheet_json["typeSpecification"] = typeSpec;

//     // physicalParameters (Required)
//     nlohmann::json physicalParams;
//     physicalParams["speedMin"] = config_.amr_params.min_speed;
//     physicalParams["speedMax"] = config_.amr_params.max_speed;
//     physicalParams["accelerationMax"] = config_.amr_params.accelerationMax;
//     physicalParams["decelerationMax"] = config_.amr_params.decelerationMax;
//     physicalParams["heightMin"] = config_.amr_params.heightMin;
//     physicalParams["heightMax"] = config_.amr_params.heightMax;
//     physicalParams["width"] = config_.amr_params.width;
//     physicalParams["length"] = config_.amr_params.length;
//     factsheet_json["physicalParameters"] = physicalParams;

//     // protocolLimits (Required)
//     nlohmann::json protocolLimits;
//     nlohmann::json maxStringLens;
//     maxStringLens["msgLen"] = 10000;
//     maxStringLens["topicSerialLen"] = 256;
//     maxStringLens["topicElemLen"] = 256;
//     maxStringLens["idLen"] = 256;
//     maxStringLens["idNumericalOnly"] = false;
//     maxStringLens["enumLen"] = 256;
//     maxStringLens["loadIdLen"] = 256;
//     protocolLimits["maxStringLens"] = maxStringLens;
//     protocolLimits["maxArrayLens"] = nlohmann::json::object();
    
//     nlohmann::json timing;
//     timing["minOrderInterval"] = 1.0;
//     timing["minStateInterval"] = 1.0;
//     timing["defaultStateInterval"] = 30.0;
//     timing["visualizationInterval"] = 0.1;
//     protocolLimits["timing"] = timing;
//     factsheet_json["protocolLimits"] = protocolLimits;

//     // protocolFeatures (Required)
//     nlohmann::json protocolFeatures;
//     nlohmann::json optionalParameters = nlohmann::json::array();
//     optionalParameters.push_back({
//         {"parameter", "trajectory"},
//         {"support", "SUPPORTED"},
//         {"description", "NURBS trajectory support"}
//     });
//     protocolFeatures["optionalParameters"] = optionalParameters;
    
//     nlohmann::json agvActions = nlohmann::json::array();
//     nlohmann::json pickAction;
//     pickAction["actionType"] = "pick";
//     pickAction["actionDescription"] = "Pick up a load";
//     pickAction["actionScopes"] = nlohmann::json::array({"NODE"});
    
//     nlohmann::json pickParams = nlohmann::json::array();
//     pickParams.push_back({
//         {"key", "stationType"},
//         {"valueDataType", "STRING"},
//         {"description", "Type of station"},
//         {"isOptional", false}
//     });
//     pickAction["actionParameters"] = pickParams;
//     pickAction["resultDescription"] = "Load picked successfully";
//     agvActions.push_back(pickAction);
//     protocolFeatures["agvActions"] = agvActions;
//     factsheet_json["protocolFeatures"] = protocolFeatures;

//     // agvGeometry (Required)
//     nlohmann::json agvGeometry;
//     nlohmann::json envelopes2d = nlohmann::json::array();
//     nlohmann::json envelope;
//     envelope["set"] = "default";
//     nlohmann::json polygonPoints = nlohmann::json::array();
//     polygonPoints.push_back({{"x", -0.5}, {"y", -0.4}});
//     polygonPoints.push_back({{"x", 0.5}, {"y", -0.4}});
//     polygonPoints.push_back({{"x", 0.5}, {"y", 0.4}});
//     polygonPoints.push_back({{"x", -0.5}, {"y", 0.4}});
//     envelope["polygonPoints"] = polygonPoints;
//     envelope["description"] = "Base footprint";
//     envelopes2d.push_back(envelope);
//     agvGeometry["envelopes2d"] = envelopes2d;
//     agvGeometry["envelopes3d"] = nlohmann::json::array();
//     factsheet_json["agvGeometry"] = agvGeometry;

//     // loadSpecification (Required)
//     nlohmann::json loadSpec;
//     loadSpec["loadPositions"] = nlohmann::json::array({"top"});
    
//     nlohmann::json loadSets = nlohmann::json::array();
//     nlohmann::json loadSet;
//     loadSet["setName"] = "europalette";
//     loadSet["loadType"] = "EPAL";
//     loadSet["loadPositions"] = nlohmann::json::array({"top"});
//     loadSet["boundingBoxReference"] = {
//         {"x", 0.0},
//         {"y", 0.0},
//         {"z", 0.0},
//         {"theta", 0}
//     };
//     loadSet["loadDimensions"] = {
//         {"length", 1.2},
//         {"width", 0.8},
//         {"height", 0.15}
//     };
//     loadSet["maxWeight"] = 100.0;
//     loadSet["minLoadhandlingHeight"] = 0.0;
//     loadSet["maxLoadhandlingHeight"] = 1.5;
//     loadSets.push_back(loadSet);
//     loadSpec["loadSets"] = loadSets;
//     factsheet_json["loadSpecification"] = loadSpec;

//     return factsheet_json.dump();
// }

std::string Vda5050Protocol::makeFactsheetMessage()
{
    nlohmann::json factsheet_json;

    // Required Header Fields
    factsheet_json["headerId"] = factsheet_header_id_++;
    factsheet_json["timestamp"] = getCurrentTimestampISO8601();
    factsheet_json["version"] = "2.1.0";
    factsheet_json["manufacturer"] = "ZenixRobotics";
    factsheet_json["serialNumber"] = agv_id_;

    // typeSpecification (Required)
    nlohmann::json typeSpec;
    typeSpec["seriesName"] = "ZenixAMR";
    typeSpec["agvKinematic"] = "OMNI";
    typeSpec["agvClass"] = "CARRIER";
    typeSpec["maxLoadMass"] = 100.0;
    typeSpec["localizationTypes"] = nlohmann::json::array({"RFID"});
    typeSpec["navigationTypes"] = nlohmann::json::array({"AUTONOMOUS"});
    typeSpec["seriesDescription"] = "Zenix Autonomous Mobile Robot Series";
    factsheet_json["typeSpecification"] = typeSpec;

    // physicalParameters (Required)
    nlohmann::json physicalParams;
    physicalParams["speedMin"] = config_.amr_params.min_speed;
    physicalParams["speedMax"] = config_.amr_params.max_speed;
    physicalParams["accelerationMax"] = config_.amr_params.accelerationMax;
    physicalParams["decelerationMax"] = config_.amr_params.decelerationMax;
    physicalParams["heightMin"] = config_.amr_params.heightMin;
    physicalParams["heightMax"] = config_.amr_params.heightMax;
    physicalParams["width"] = config_.amr_params.width;
    physicalParams["length"] = config_.amr_params.length;
    factsheet_json["physicalParameters"] = physicalParams;

    // protocolLimits (Required)
    nlohmann::json protocolLimits;
    nlohmann::json maxStringLens;
    maxStringLens["msgLen"] = 10000;
    maxStringLens["topicSerialLen"] = 256;
    maxStringLens["topicElemLen"] = 256;
    maxStringLens["idLen"] = 256;
    maxStringLens["idNumericalOnly"] = false;
    maxStringLens["enumLen"] = 256;
    maxStringLens["loadIdLen"] = 256;
    protocolLimits["maxStringLens"] = maxStringLens;
    protocolLimits["maxArrayLens"] = nlohmann::json::object();
    
    nlohmann::json timing;
    timing["minOrderInterval"] = 1.0;
    timing["minStateInterval"] = 1.0;
    timing["defaultStateInterval"] = 30.0;
    timing["visualizationInterval"] = 0.1;
    protocolLimits["timing"] = timing;
    factsheet_json["protocolLimits"] = protocolLimits;

    // protocolFeatures (Required)
    nlohmann::json protocolFeatures;
    nlohmann::json optionalParameters = nlohmann::json::array();
    optionalParameters.push_back({
        {"parameter", "trajectory"},
        {"support", "SUPPORTED"},
        {"description", "NURBS trajectory support"}
    });
    protocolFeatures["optionalParameters"] = optionalParameters;
    
    nlohmann::json agvActions = nlohmann::json::array();
    nlohmann::json pickAction;
    pickAction["actionType"] = "pick";
    pickAction["actionDescription"] = "Pick up a load";
    pickAction["actionScopes"] = nlohmann::json::array({"NODE"});
    
    nlohmann::json pickParams = nlohmann::json::array();
    pickParams.push_back({
        {"key", "stationType"},
        {"valueDataType", "STRING"},
        {"description", "Type of station"},
        {"isOptional", false}
    });
    pickAction["actionParameters"] = pickParams;
    pickAction["resultDescription"] = "Load picked successfully";
    agvActions.push_back(pickAction);
    protocolFeatures["agvActions"] = agvActions;
    factsheet_json["protocolFeatures"] = protocolFeatures;

    // agvGeometry (Required)
    nlohmann::json agvGeometry;
    nlohmann::json envelopes2d = nlohmann::json::array();
    nlohmann::json envelope;
    envelope["set"] = "default";
    nlohmann::json polygonPoints = nlohmann::json::array();
    polygonPoints.push_back({{"x", -0.5}, {"y", -0.4}});
    polygonPoints.push_back({{"x", 0.5}, {"y", -0.4}});
    polygonPoints.push_back({{"x", 0.5}, {"y", 0.4}});
    polygonPoints.push_back({{"x", -0.5}, {"y", 0.4}});
    envelope["polygonPoints"] = polygonPoints;
    envelope["description"] = "Base footprint";
    envelopes2d.push_back(envelope);
    agvGeometry["envelopes2d"] = envelopes2d;
    agvGeometry["envelopes3d"] = nlohmann::json::array();
    factsheet_json["agvGeometry"] = agvGeometry;

    // loadSpecification (Required)
    nlohmann::json loadSpec;
    loadSpec["loadPositions"] = nlohmann::json::array({"top"});
    
    nlohmann::json loadSets = nlohmann::json::array();
    nlohmann::json loadSet;
    loadSet["setName"] = "europalette";
    loadSet["loadType"] = "EPAL";
    loadSet["loadPositions"] = nlohmann::json::array({"top"});
    loadSet["boundingBoxReference"] = {
        {"x", 0.0},
        {"y", 0.0},
        {"z", 0.0},
        {"theta", 0}
    };
    loadSet["loadDimensions"] = {
        {"length", 1.2},
        {"width", 0.8},
        {"height", 0.15}
    };
    loadSet["maxWeight"] = 100.0;
    loadSet["minLoadhandlingHeight"] = 0.0;
    loadSet["maxLoadhandlingHeight"] = 1.5;
    loadSets.push_back(loadSet);
    loadSpec["loadSets"] = loadSets;
    factsheet_json["loadSpecification"] = loadSpec;

    return factsheet_json.dump();
}


// INSTANT ACTIONS HANDLER

// ============================================================================
// Order Merge Logic
// ============================================================================

bool Vda5050Protocol::mergeOrder(const nlohmann::json& order_json)
{
    std::cout << "[Vda5050Protocol] ===== Order Merge Start =====" << std::endl;
    
    // 1. Extract order information
    std::string new_order_id = order_json.value("orderId", "");
    int new_order_update_id = order_json.value("orderUpdateId", 0);
    
    std::cout << "[Vda5050Protocol] Received Order:" << std::endl;
    std::cout << "  - orderId: " << new_order_id << std::endl;
    std::cout << "  - orderUpdateId: " << new_order_update_id << std::endl;
    std::cout << "[Vda5050Protocol] Current Order:" << std::endl;
    std::cout << "  - orderId: " << current_order_id_ << std::endl;
    std::cout << "  - orderUpdateId: " << current_order_update_id_ << std::endl;
    
    // 2. Validate order
    std::string error_type, error_desc;
    if (!validateOrder(order_json, error_type, error_desc))
    {
        std::cerr << "[Vda5050Protocol] Order validation failed: " << error_desc << std::endl;
        publishOrderRejectionError(error_type, error_desc);
        return false;
    }
    
    // ========================================================================
    // Cancel Order 처리 (nodes=[], edges=[])
    // ========================================================================
    auto nodes_json = order_json["nodes"];
    auto edges_json = order_json["edges"];
    
    if (nodes_json.empty() && edges_json.empty())
    {
        std::cout << "[Vda5050Protocol] ===== ORDER CANCEL DETECTED =====" << std::endl;
        std::cout << "[Vda5050Protocol] Empty nodes and edges - Cancelling order" << std::endl;
        
        if (!current_order_id_.empty())
        {
            std::cout << "[Vda5050Protocol] Cancelling current order: " 
                      << current_order_id_ << std::endl;
        }
        
        // 1. AMR Cancel 처리
        if (amr_)
        {
            amr_->cancelOrder();
        }
        
        // 2. Protocol 상태 초기화
        current_order_id_ = "";
        current_order_update_id_ = 0;
        current_zone_set_id_ = "";
        order_active_ = false;
        
        // 3. 내부 데이터 클리어
        received_nodes_.clear();
        received_edges_.clear();
        ordered_nodes_.clear();
        node_by_sequence_.clear();
        edge_by_sequence_.clear();
        
        // 4. 즉시 State 발행 (매우 중요!)
        std::cout << "[Vda5050Protocol] Publishing IDLE state..." << std::endl;
        publishStateMessage(amr_);
        
        std::cout << "[Vda5050Protocol] Order cancelled successfully - AGV is IDLE" << std::endl;
        std::cout << "[Vda5050Protocol] ===== Order Merge End =====" << std::endl;
        
        return true;
    }
    // ========================================================================
        
    // 3. Determine if this is a new order or an update
    bool is_new_order = (new_order_id != current_order_id_);
    bool is_update = (!is_new_order && new_order_update_id > current_order_update_id_);
    
    if (is_new_order)
    {
        std::cout << "[Vda5050Protocol] NEW ORDER detected (orderId changed)" << std::endl;
        
        // Cancel existing order
        if (!current_order_id_.empty())
        {
            std::cout << "[Vda5050Protocol] Cancelling previous order: " << current_order_id_ << std::endl;
            if (amr_)
            {
                amr_->cancelOrder();
            }
        }
        
        // Reset order tracking
        current_order_id_ = new_order_id;
        current_order_update_id_ = new_order_update_id;
        current_zone_set_id_ = order_json.value("zoneSetId", "");
        
        received_nodes_.clear();
        received_edges_.clear();
        node_by_sequence_.clear();
        edge_by_sequence_.clear();
        
        // Parse new order
        parseOrderNodes(order_json["nodes"]);
        parseOrderEdges(order_json["edges"]);
        
        // Validate start node position (strict)
        if (!received_nodes_.empty() && amr_)
        {
            double cur_x = 0.0, cur_y = 0.0, cur_theta = 0.0;
            amr_->getVcu()->getEstimatedPose(cur_x, cur_y, cur_theta);
            
            if (!validateStartNodePosition(received_nodes_[0], cur_x, cur_y, false, error_desc))
            {
                std::cerr << "[Vda5050Protocol] Start node validation failed: " << error_desc << std::endl;
                publishOrderRejectionError("noRouteError", error_desc);
                return false;
            }
        }
        
        // Send to AMR
        if (amr_)
        {
            std::vector<NodeInfo> all_nodes_for_amr = received_nodes_;
            amr_->setOrder(received_nodes_, received_edges_, all_nodes_for_amr, 0.5);
        }
        
        order_active_ = true;
        
        // Publish state immediately
        publishStateMessage(amr_);
        
        std::cout << "[Vda5050Protocol] New order processed successfully" << std::endl;
        return true;
    }
    else if (is_update)
    {
        std::cout << "[Vda5050Protocol] ORDER UPDATE detected" << std::endl;
        std::cout << "  - Merging update " << new_order_update_id 
                  << " into existing order " << current_order_id_ << std::endl;
        
        // Update orderUpdateId IMMEDIATELY (before any processing)
        current_order_update_id_ = new_order_update_id;
        current_zone_set_id_ = order_json.value("zoneSetId", "");
        
        // Publish state immediately to acknowledge update
        publishStateMessage(amr_);
        
        // Parse new nodes and edges from update
        std::vector<NodeInfo> update_nodes;
        std::vector<EdgeInfo> update_edges;
        
        for (const auto& node_json : order_json["nodes"])
        {
            NodeInfo node;
            node.nodeId = node_json.value("nodeId", "");
            node.sequenceId = node_json.value("sequenceId", 0);
            node.nodeDescription = node_json.value("nodeDescription", "");
            node.released = node_json.value("released", true);
            
            if (node_json.contains("nodePosition"))
            {
                node.hasNodePosition = true;
                const auto& pos = node_json["nodePosition"];
                node.nodePosition.x = pos.value("x", 0.0);
                node.nodePosition.y = pos.value("y", 0.0);
                node.x = node.nodePosition.x;
                node.y = node.nodePosition.y;
                
                if (pos.contains("theta"))
                {
                    node.nodePosition.hasTheta = true;
                    node.nodePosition.theta = pos.value("theta", 0.0);
                }
            }
            
            // Parse actions
            if (node_json.contains("actions") && node_json["actions"].is_array())
            {
                for (const auto& action_json : node_json["actions"])
                {
                    Action action;
                    action.actionId = action_json.value("actionId", "");
                    action.actionType = action_json.value("actionType", "");
                    action.actionDescription = action_json.value("actionDescription", "");
                    action.blockingType = action_json.value("blockingType", "NONE");
                    
                    if (action_json.contains("actionParameters") && action_json["actionParameters"].is_array())
                    {
                        for (const auto& param_json : action_json["actionParameters"])
                        {
                            ActionParameter param;
                            param.key = param_json.value("key", "");
                            param.value = param_json["value"];
                            action.actionParameters.push_back(param);
                        }
                    }
                    
                    node.actions.push_back(action);
                }
            }
            
            update_nodes.push_back(node);
        }
        
        for (const auto& edge_json : order_json["edges"])
        {
            EdgeInfo edge;
            edge.edgeId = edge_json.value("edgeId", "");
            edge.sequenceId = edge_json.value("sequenceId", 0);
            edge.edgeDescription = edge_json.value("edgeDescription", "");
            edge.released = edge_json.value("released", true);
            edge.startNodeId = edge_json.value("startNodeId", "");
            edge.endNodeId = edge_json.value("endNodeId", "");
            
            if (edge_json.contains("maxSpeed"))
            {
                edge.hasMaxSpeed = true;
                edge.maxSpeed = edge_json.value("maxSpeed", 1.0);
            }
            else
            {
                edge.maxSpeed = 1.0;
            }
            
            // Parse actions
            if (edge_json.contains("actions") && edge_json["actions"].is_array())
            {
                for (const auto& action_json : edge_json["actions"])
                {
                    Action action;
                    action.actionId = action_json.value("actionId", "");
                    action.actionType = action_json.value("actionType", "");
                    action.actionDescription = action_json.value("actionDescription", "");
                    action.blockingType = action_json.value("blockingType", "NONE");
                    
                    if (action_json.contains("actionParameters") && action_json["actionParameters"].is_array())
                    {
                        for (const auto& param_json : action_json["actionParameters"])
                        {
                            ActionParameter param;
                            param.key = param_json.value("key", "");
                            param.value = param_json["value"];
                            action.actionParameters.push_back(param);
                        }
                    }
                    
                    edge.actions.push_back(action);
                }
            }
            
            update_edges.push_back(edge);
        }
        
        if (update_nodes.empty())
        {
            std::cerr << "[Vda5050Protocol] Update has no nodes" << std::endl;
            return false;
        }
        
        // Find the first sequenceId in the update
        int first_update_seq = update_nodes[0].sequenceId;
        
        std::cout << "[Vda5050Protocol] Update starts at sequenceId: " << first_update_seq << std::endl;
        std::cout << "[Vda5050Protocol] Current path has " << received_nodes_.size() << " nodes" << std::endl;
        
        // Validate that first node of update exists in current path (relaxed validation)
        bool first_node_exists = false;
        for (const auto& existing_node : received_nodes_)
        {
            if (existing_node.sequenceId == first_update_seq)
            {
                first_node_exists = true;
                break;
            }
        }
        
        if (!first_node_exists && amr_)
        {
            // First node doesn't exist - check if it's near current position
            double cur_x = 0.0, cur_y = 0.0, cur_theta = 0.0;
            amr_->getVcu()->getEstimatedPose(cur_x, cur_y, cur_theta);
            
            std::string nearest_node = findNearestNodeInCurrentPath(cur_x, cur_y);
            if (!nearest_node.empty())
            {
                std::cout << "[Vda5050Protocol] First update node not in path, but AGV is near node: " 
                          << nearest_node << ". Accepting update." << std::endl;
            }
            else
            {
                std::cerr << "[Vda5050Protocol] Warning: First update node sequenceId " << first_update_seq
                          << " not found in current path, and AGV not near any known node" << std::endl;
            }
        }
        
        // Remove all nodes/edges with sequenceId >= first_update_seq
        auto node_it = received_nodes_.begin();
        while (node_it != received_nodes_.end())
        {
            if (node_it->sequenceId >= first_update_seq)
            {
                std::cout << "[Vda5050Protocol] Removing old node: sequenceId=" << node_it->sequenceId 
                          << ", nodeId=" << node_it->nodeId << std::endl;
                node_by_sequence_.erase(node_it->sequenceId);
                node_it = received_nodes_.erase(node_it);
            }
            else
            {
                ++node_it;
            }
        }
        
        auto edge_it = received_edges_.begin();
        while (edge_it != received_edges_.end())
        {
            if (edge_it->sequenceId >= first_update_seq)
            {
                std::cout << "[Vda5050Protocol] Removing old edge: sequenceId=" << edge_it->sequenceId 
                          << ", edgeId=" << edge_it->edgeId << std::endl;
                edge_by_sequence_.erase(edge_it->sequenceId);
                edge_it = received_edges_.erase(edge_it);
            }
            else
            {
                ++edge_it;
            }
        }
        
        // Append new nodes and edges
        for (const auto& node : update_nodes)
        {
            received_nodes_.push_back(node);
            node_by_sequence_[node.sequenceId] = node;
            std::cout << "[Vda5050Protocol] Added new node: sequenceId=" << node.sequenceId 
                      << ", nodeId=" << node.nodeId << std::endl;
        }
        
        for (const auto& edge : update_edges)
        {
            received_edges_.push_back(edge);
            edge_by_sequence_[edge.sequenceId] = edge;
            std::cout << "[Vda5050Protocol] Added new edge: sequenceId=" << edge.sequenceId 
                      << ", edgeId=" << edge.edgeId << std::endl;
        }
        
        std::cout << "[Vda5050Protocol] Merged path now has " << received_nodes_.size() << " nodes, "
                  << received_edges_.size() << " edges" << std::endl;
        
        // Send updated order to AMR (use updateOrder for Order Updates)
        if (amr_)
        {
            std::vector<NodeInfo> all_nodes_for_amr = received_nodes_;
            amr_->updateOrder(received_nodes_, received_edges_, all_nodes_for_amr, 0.5);
            std::cout << "[Vda5050Protocol] Called updateOrder() - AGV will continue from current position" << std::endl;
        }
        
        // Publish state again after merge
        publishStateMessage(amr_);
        
        std::cout << "[Vda5050Protocol] Order update merged successfully" << std::endl;
        return true;
    }
    else
    {
        std::cout << "[Vda5050Protocol] Ignoring duplicate or old order" << std::endl;
        std::cout << "  - Same orderId, but orderUpdateId not increasing (" 
                  << new_order_update_id << " <= " << current_order_update_id_ << ")" << std::endl;
        return false;
    }
}

void Vda5050Protocol::parseOrderNodes(const nlohmann::json& nodes_json)
{
    for (const auto& node_json : nodes_json)
    {
        NodeInfo node;
        node.nodeId = node_json.value("nodeId", "");
        node.sequenceId = node_json.value("sequenceId", 0);
        node.nodeDescription = node_json.value("nodeDescription", "");
        node.released = node_json.value("released", true);
        
        if (node_json.contains("nodePosition"))
        {
            node.hasNodePosition = true;
            const auto& pos = node_json["nodePosition"];
            node.nodePosition.x = pos.value("x", 0.0);
            node.nodePosition.y = pos.value("y", 0.0);
            node.x = node.nodePosition.x;
            node.y = node.nodePosition.y;
            
            if (pos.contains("theta"))
            {
                node.nodePosition.hasTheta = true;
                node.nodePosition.theta = pos.value("theta", 0.0);
            }
        }
        
        // Parse actions
        if (node_json.contains("actions") && node_json["actions"].is_array())
        {
            for (const auto& action_json : node_json["actions"])
            {
                Action action;
                action.actionId = action_json.value("actionId", "");
                action.actionType = action_json.value("actionType", "");
                action.actionDescription = action_json.value("actionDescription", "");
                action.blockingType = action_json.value("blockingType", "NONE");
                
                if (action_json.contains("actionParameters") && action_json["actionParameters"].is_array())
                {
                    for (const auto& param_json : action_json["actionParameters"])
                    {
                        ActionParameter param;
                        param.key = param_json.value("key", "");
                        param.value = param_json["value"];
                        action.actionParameters.push_back(param);
                    }
                }
                
                node.actions.push_back(action);
            }
        }
        
        received_nodes_.push_back(node);
        node_by_sequence_[node.sequenceId] = node;
    }
}

void Vda5050Protocol::parseOrderEdges(const nlohmann::json& edges_json)
{
    for (const auto& edge_json : edges_json)
    {
        EdgeInfo edge;
        edge.edgeId = edge_json.value("edgeId", "");
        edge.sequenceId = edge_json.value("sequenceId", 0);
        edge.edgeDescription = edge_json.value("edgeDescription", "");
        edge.released = edge_json.value("released", true);
        edge.startNodeId = edge_json.value("startNodeId", "");
        edge.endNodeId = edge_json.value("endNodeId", "");
        
        if (edge_json.contains("maxSpeed"))
        {
            edge.hasMaxSpeed = true;
            edge.maxSpeed = edge_json.value("maxSpeed", 1.0);
        }
        else
        {
            edge.maxSpeed = 1.0;
        }
        
        // ========================================
        // turnCenter 파싱
        // ========================================
        if (edge_json.contains("turnCenter") && edge_json["turnCenter"].is_object())
        {
            const auto& tc = edge_json["turnCenter"];
            if (tc.contains("x") && tc.contains("y"))
            {
                edge.turnCenter.x = tc["x"].get<double>();
                edge.turnCenter.y = tc["y"].get<double>();
                edge.hasTurnCenter = true;
                
                std::cout << "[Vda5050Protocol] Edge " << edge.edgeId 
                          << " has turnCenter: (" << edge.turnCenter.x 
                          << ", " << edge.turnCenter.y << ")" << std::endl;
            }
        }
        
        
        // Parse actions
        if (edge_json.contains("actions") && edge_json["actions"].is_array())
        {
            for (const auto& action_json : edge_json["actions"])
            {
                Action action;
                action.actionId = action_json.value("actionId", "");
                action.actionType = action_json.value("actionType", "");
                action.actionDescription = action_json.value("actionDescription", "");
                action.blockingType = action_json.value("blockingType", "NONE");
                
                if (action_json.contains("actionParameters") && action_json["actionParameters"].is_array())
                {
                    for (const auto& param_json : action_json["actionParameters"])
                    {
                        ActionParameter param;
                        param.key = param_json.value("key", "");
                        param.value = param_json["value"];
                        action.actionParameters.push_back(param);
                    }
                }
                
                edge.actions.push_back(action);
            }
        }
        
        received_edges_.push_back(edge);
        edge_by_sequence_[edge.sequenceId] = edge;
    }
}

// bool Vda5050Protocol::validateOrder(const nlohmann::json& order_json, std::string& error_type, std::string& error_desc)
// {
//     // Check required fields
//     if (!order_json.contains("orderId"))
//     {
//         error_type = "orderError";
//         error_desc = "Missing required field: orderId";
//         return false;
//     }
    
//     if (!order_json.contains("orderUpdateId"))
//     {
//         error_type = "orderError";
//         error_desc = "Missing required field: orderUpdateId";
//         return false;
//     }
    
//     if (!order_json.contains("nodes") || !order_json["nodes"].is_array())
//     {
//         error_type = "orderError";
//         error_desc = "Missing or invalid nodes array";
//         return false;
//     }
    
//     if (!order_json.contains("edges") || !order_json["edges"].is_array())
//     {
//         error_type = "orderError";
//         error_desc = "Missing or invalid edges array";
//         return false;
//     }
  
//     // VDA5050 표준: Node는 짝수(0,2,4...), Edge는 홀수(1,3,5...)
//     const auto& nodes = order_json["nodes"];
//     const auto& edges = order_json["edges"];
    
//     // Node sequenceId 검증: 짝수여야 함
//     for (size_t i = 0; i < nodes.size(); ++i)
//     {
//         if (!nodes[i].contains("sequenceId"))
//         {
//             error_type = "orderError";
//             error_desc = "Node missing sequenceId";
//             return false;
//         }
        
//         int seq = nodes[i]["sequenceId"].get<int>();
//         if (seq % 2 != 0)  // 홀수이면 에러
//         {
//             error_type = "orderError";
//             error_desc = "Node has odd sequenceId (should be even): " + std::to_string(seq);
//             return false;
//         }
//     }
    
//     // Edge sequenceId 검증: 홀수여야 함
//     for (size_t i = 0; i < edges.size(); ++i)
//     {
//         if (!edges[i].contains("sequenceId"))
//         {
//             error_type = "orderError";
//             error_desc = "Edge missing sequenceId";
//             return false;
//         }
        
//         int seq = edges[i]["sequenceId"].get<int>();
//         if (seq % 2 == 0)  // 짝수이면 에러
//         {
//             error_type = "orderError";
//             error_desc = "Edge has even sequenceId (should be odd): " + std::to_string(seq);
//             return false;
//         }
//     }

//     double wheel_base = 9.3; // todo : from /config/amr_params.yaml
//     double max_steer_angle = 30.0 * M_PI / 180.0; // 라디안 변환
//     double min_radius = wheel_base / std::tan(max_steer_angle);

//     for (const auto& edge : order_json["edges"]) {
//         if (edge.contains("turnCenter")) {
//             double tx = edge["turnCenter"]["x"];
//             double ty = edge["turnCenter"]["y"];
            
//             // 시작/끝 노드 검색
//             std::string start_id = edge["startNodeId"];
//             std::string end_id = edge["endNodeId"];
//             auto nodes_json = order_json["nodes"];
            
//             auto s_it = std::find_if(nodes_json.begin(), nodes_json.end(), [&](const nlohmann::json& n){ return n["nodeId"] == start_id; });
//             auto e_it = std::find_if(nodes_json.begin(), nodes_json.end(), [&](const nlohmann::json& n){ return n["nodeId"] == end_id; });

//             if (s_it != nodes_json.end() && (*s_it).contains("nodePosition")) {
//                 double sx = (*s_it)["nodePosition"]["x"];
//                 double sy = (*s_it)["nodePosition"]["y"];
//                 double stheta = (*s_it)["nodePosition"].value("theta", 0.0);
                
//                 // (A) 최소 반경 검증
//                 double radius = std::hypot(tx - sx, ty - sy);
//                 if (radius < min_radius) {
//                     error_type = "noRouteError";
//                     error_desc = "Radius too small: " + std::to_string(radius) + "m < min " + std::to_string(min_radius) + "m";
//                     return false;
//                 }

//                 // (B) 기하학적 특이점(Singularity) 검증
//                 // 진입 방향 벡터와 중심점 벡터 사이의 각도 계산
//                 double dx = tx - sx;
//                 double dy = ty - sy;
//                 double angle_to_center = std::atan2(dy, dx);
                
//                 // 헤딩 방향과 중심점 방향의 차이가 0도 또는 180도 부근이면 방향 판정 불가
//                 double diff = std::abs(std::remainder(stheta - angle_to_center, M_PI));
//                 if (diff < 0.1 || diff > (M_PI - 0.1)) { // 약 5.7도 임계치
//                     error_type = "noRouteError";
//                     error_desc = "Geometric Singularity: turnCenter is on the heading line. Direction ambiguous.";
//                     return false;
//                 }

//                 // (C) 원호 정합성 검증 (끝 노드 기준 반경도 동일한지 확인)
//                 if (e_it != nodes_json.end() && (*e_it).contains("nodePosition")) {
//                     double ex = (*e_it)["nodePosition"]["x"];
//                     double ey = (*e_it)["nodePosition"]["y"];
//                     double end_radius = std::hypot(tx - ex, ty - ey);
//                     if (std::abs(radius - end_radius) > 0.5) { // 0.5m 오차 허용
//                         error_type = "noRouteError";
//                         error_desc = "Invalid Arc: Start and End nodes have different radii from turnCenter.";
//                         return false;
//                     }
//                 }
//             }
//         }
//     }

//     // for (const auto& edge : order_json["edges"]) {
//     //     if (edge.contains("turnCenter")) {
//     //         double tx = edge["turnCenter"]["x"];
//     //         double ty = edge["turnCenter"]["y"];
            
//     //         // 시작 노드 좌표 가져오기 (반경 계산용)
//     //         std::string start_node_id = edge["startNodeId"];
//     //         auto start_node_it = std::find_if(order_json["nodes"].begin(), order_json["nodes"].end(),
//     //             [&](const nlohmann::json& n) { return n["nodeId"] == start_node_id; });

//     //         if (start_node_it != order_json["nodes"].end()) {
//     //             double sx = (*start_node_it)["nodePosition"]["x"];
//     //             double sy = (*start_node_it)["nodePosition"]["y"];
                
//     //             double calculated_radius = std::hypot(tx - sx, ty - sy);

//     //             if (calculated_radius < min_radius) {
//     //                 error_desc = "Invalid turnCenter: Radius (" + std::to_string(calculated_radius) + 
//     //                                     "m) is smaller than AMR's minimum radius (" + std::to_string(min_radius) + "m)";
//     //                 return false; // 검증 실패
//     //             }
//     //         }
//     //     }
//     // }

//     return true;
// }

bool Vda5050Protocol::validateOrder(const nlohmann::json& order_json, 
                                    std::string& error_type, std::string& error_desc)
{
    // ... 기존 코드 ...
    
    const double wheel_base = 9.3;
    double max_steer_angle = 30.0 * M_PI / 180.0;
    double min_radius = wheel_base / std::tan(max_steer_angle);
    
    // ★ 추가: Pure Pursuit 안정성을 위한 최소 반경
    double min_safe_radius = std::max(min_radius * 1.5, 8.0);  // 최소 8m
    
    for (const auto& edge : order_json["edges"]) {
        if (edge.contains("turnCenter")) {
            double tx = edge["turnCenter"]["x"];
            double ty = edge["turnCenter"]["y"];
            
            std::string start_id = edge["startNodeId"];
            std::string end_id = edge["endNodeId"];
            auto nodes_json = order_json["nodes"];
            
            auto s_it = std::find_if(nodes_json.begin(), nodes_json.end(), 
                [&](const nlohmann::json& n){ return n["nodeId"] == start_id; });
            auto e_it = std::find_if(nodes_json.begin(), nodes_json.end(), 
                [&](const nlohmann::json& n){ return n["nodeId"] == end_id; });

            if (s_it != nodes_json.end() && (*s_it).contains("nodePosition")) {
                double sx = (*s_it)["nodePosition"]["x"];
                double sy = (*s_it)["nodePosition"]["y"];
                double stheta = (*s_it)["nodePosition"].value("theta", 0.0);
                
                double radius = std::hypot(tx - sx, ty - sy);
                
                // (A) 물리적 최소 반경 검증
                if (radius < min_radius) {
                    error_type = "noRouteError";
                    error_desc = "Radius too small: " + std::to_string(radius) + 
                                "m < min " + std::to_string(min_radius) + "m";
                    return false;
                }
                
                // ★ (A-2) Pure Pursuit 안정성 검증
                if (radius < min_safe_radius) {
                    error_type = "noRouteError";
                    error_desc = "Radius too small for stable tracking: " + 
                                std::to_string(radius) + "m < safe min " + 
                                std::to_string(min_safe_radius) + "m";
                    return false;
                }

                // (B) 기하학적 특이점 검증
                double dx = tx - sx;
                double dy = ty - sy;
                double angle_to_center = std::atan2(dy, dx);
                double diff = std::abs(std::remainder(stheta - angle_to_center, M_PI));
                
                if (diff < 0.1 || diff > (M_PI - 0.1)) {
                    error_type = "noRouteError";
                    error_desc = "Geometric Singularity: turnCenter on heading line";
                    return false;
                }

                // (C) 원호 정합성 검증
                if (e_it != nodes_json.end() && (*e_it).contains("nodePosition")) {
                    double ex = (*e_it)["nodePosition"]["x"];
                    double ey = (*e_it)["nodePosition"]["y"];
                    double end_radius = std::hypot(tx - ex, ty - ey);
                    
                    if (std::abs(radius - end_radius) > 0.5) {
                        error_type = "noRouteError";
                        error_desc = "Invalid Arc: Different radii at start/end";
                        return false;
                    }
                }
                
                // ★ (D) 호의 각도 범위 검증 (너무 급격한 회전 방지)
                // if (e_it != nodes_json.end() && (*e_it).contains("nodePosition")) {
                //     double ex = (*e_it)["nodePosition"]["x"];
                //     double ey = (*e_it)["nodePosition"]["y"];
                    
                //     double start_angle = std::atan2(sy - ty, sx - tx);
                //     double end_angle = std::atan2(ey - ty, ex - tx);
                //     double arc_angle = std::abs(std::remainder(end_angle - start_angle, 2*M_PI));
                    
                //     // 180도 이상 회전은 위험
                //     if (arc_angle > M_PI) {
                //         error_type = "noRouteError";
                //         error_desc = "Arc angle too large: " + 
                //                     std::to_string(arc_angle * 180.0 / M_PI) + " deg";
                //         return false;
                //     }
                    
                //     // 작은 반경에서 급격한 각도 변화 방지
                //     double max_allowed_angle = M_PI / 2.0;  // 90도
                //     if (radius < min_safe_radius * 1.5 && arc_angle > max_allowed_angle) {
                //         error_type = "noRouteError";
                //         error_desc = "Sharp turn on small radius: arc=" + 
                //                     std::to_string(arc_angle * 180.0 / M_PI) + 
                //                     " deg, radius=" + std::to_string(radius) + "m";
                //         return false;
                //     }
                // }
            }
        }
    }

    return true;
}

bool Vda5050Protocol::validateStartNodePosition(const NodeInfo& start_node, double current_x, double current_y, 
                                                bool is_update_order, std::string& error_desc)
{
    if (!start_node.hasNodePosition)
    {
        // No position to validate
        return true;
    }
    
    double dx = start_node.nodePosition.x - current_x;
    double dy = start_node.nodePosition.y - current_y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    // For new orders: strict validation
    if (!is_update_order)
    {
        double allowedDeviationXY = start_node.nodePosition.hasAllowedDeviationXY ? 
                                    start_node.nodePosition.allowedDeviationXY : 0.5;
        
        if (distance > allowedDeviationXY)
        {
            error_desc = "noRouteError: START_NODE_TOO_FAR. Distance: " + std::to_string(distance) + 
                        " m, Allowed: " + std::to_string(allowedDeviationXY) + " m";
            return false;
        }
    }
    else
    {
        // For update orders: relaxed validation
        // Check if node is in existing path
        std::string nearest = findNearestNodeInCurrentPath(current_x, current_y);
        if (nearest.empty())
        {
            // Not near any known node - apply strict check
            double allowedDeviationXY = 1.0;  // More lenient for updates
            if (distance > allowedDeviationXY)
            {
                error_desc = "noRouteError: Update start node too far. Distance: " + std::to_string(distance) + " m";
                return false;
            }
        }
        // If near a known node, accept the update
    }
    
    return true;
}

std::string Vda5050Protocol::findNearestNodeInCurrentPath(double current_x, double current_y)
{
    std::string nearest_id;
    double min_distance = 1.0;  // 1m threshold
    
    for (const auto& node : received_nodes_)
    {
        if (!node.hasNodePosition)
            continue;
        
        double dx = node.nodePosition.x - current_x;
        double dy = node.nodePosition.y - current_y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < min_distance)
        {
            min_distance = distance;
            nearest_id = node.nodeId;
        }
    }
    
    return nearest_id;
}

void Vda5050Protocol::handleInstantAction(const nlohmann::json& instant_action_json)
{
    try
    {
        std::string actionType = instant_action_json.at("actionType").get<std::string>();
        std::string actionId = instant_action_json.at("actionId").get<std::string>();
        std::string blockingType = instant_action_json.at("blockingType").get<std::string>();        

        std::cout << "[Vda5050Protocol] Processing instant action: " << actionType 
                  << " (ID: " << actionId << ")" << std::endl;

        if (actionType == "factsheetRequest")
        {
            // Generate and publish factsheet message
            std::string factsheet_msg = makeFactsheetMessage();
            auto msg = mqtt::make_message(factsheet_topic_, factsheet_msg);
            msg->set_qos(1);

            if (mqtt_client_ && mqtt_client_->is_connected())
            {
                mqtt_client_->publish(msg);

                logMessage("OUT", factsheet_topic_, factsheet_msg);

                std::cout << "[Vda5050Protocol] Factsheet published in response to request: " 
                          << actionId << std::endl;
            }
            else
            {
                std::cerr << "[Vda5050Protocol] Cannot publish factsheet: MQTT client disconnected" << std::endl;
            }
        }
        else if (actionType == "cancelOrder")
        {
            std::cout << "[Vda5050Protocol] Cancel order requested. ActionId: " << actionId << std::endl;
            
            // Cancel order and reset to IDLE state
            current_order_id_ = "";
            current_order_update_id_ = 0;
            current_zone_set_id_ = "";
            order_active_ = false;
            
            // Clear order data
            received_nodes_.clear();
            received_edges_.clear();
            
            // Send cancel command to AMR
            if (amr_)
            {
                amr_->cancelOrder();
            }
            
            // Publish state immediately (with empty nodeStates and edgeStates)
            publishStateMessage(amr_);
            std::cout << "[Vda5050Protocol] Order cancelled, switched to IDLE state" << std::endl;
        }
        else
        {
            std::cout << "[Vda5050Protocol] Unknown instant action: " << actionType 
                      << ". ActionId: " << actionId << std::endl;
        }
    }
    catch (const nlohmann::json::exception& e)
    {
        std::cerr << "[Vda5050Protocol] Instant Action Schema/Parsing Error: " << e.what() << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Vda5050Protocol] Exception handling instant action: " << e.what() << std::endl;
    }    
}

void Vda5050Protocol::handleMessage(const std::string& msg, IAmr* amr)
{
    std::cout << "[Vda5050Protocol] handleMessage called" << std::endl;

    if (msg.empty() || !amr) 
    {
        std::cerr << "[Vda5050Protocol] Empty message or null AMR pointer" << std::endl;
        return;
    }

    try 
    {
        auto json_msg = nlohmann::json::parse(msg);

        // Check if this is an InstantActions message
        if (json_msg.contains("actions") && json_msg["actions"].is_array())
        {
            std::cout << "[Vda5050Protocol] InstantActions received" << std::endl;      
                
            for (const auto& action : json_msg["actions"])
            {
                handleInstantAction(action);
            }
            return;
        }

        // Check if this is an Order message
        if (!json_msg.contains("nodes") || !json_msg.contains("edges")) 
        {
            std::cerr << "[Vda5050Protocol] Order missing required nodes or edges fields" << std::endl;
            return;
        }

        // Order Merge 로직 호출
        bool merge_success = mergeOrder(json_msg);
        
        if (!merge_success)
        {
            std::cerr << "[Vda5050Protocol] Order merge failed" << std::endl;
            return;
        }
        
        std::cout << "[Vda5050Protocol] Order successfully processed" << std::endl;
    }
    catch (const nlohmann::json::exception& e)
    {
        std::cerr << "[Vda5050Protocol] JSON parsing error: " << e.what() << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Vda5050Protocol] Exception in handleMessage: " << e.what() << std::endl;
    }
}



void Vda5050Protocol::publishOrderRejectionError(const std::string& error_type, const std::string& error_description)
{
    std::cout << "[Vda5050Protocol] Publishing order rejection error: " << error_type << std::endl;
    
    // 오더 활성화 안 함
    order_active_ = false;
    
    // 오더 정보 초기화
    current_order_id_ = "";
    current_order_update_id_ = 0;
    current_zone_set_id_ = "";
    received_nodes_.clear();
    received_edges_.clear();
    
    // State 메시지 생성 (에러 포함)
    if (amr_)
    {
        // 에러를 임시로 저장
        order_rejection_error_type_ = error_type;
        order_rejection_error_description_ = error_description;
        has_order_rejection_error_ = true;
        
        publishStateMessage(amr_);
        
        // 에러 플래그 리셋
        has_order_rejection_error_ = false;
        order_rejection_error_type_ = "";
        order_rejection_error_description_ = "";
    }
}

// VISUALIZATION MESSAGE
void Vda5050Protocol::publishVisualizationMessage(IAmr* amr)
{
    if(!amr || !mqtt_client_ || !mqtt_client_->is_connected())
    {
        return;
    }

    try
    {
        std::string viz_msg = makeVisualizationMessage(amr);
        if(!viz_msg.empty())
        {
            auto msg = mqtt::make_message(visualization_topic_, viz_msg);
            msg->set_qos(1);
            mqtt_client_->publish(msg);

            // 송신 로그
            logMessage("OUT", visualization_topic_, viz_msg);
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << "[Vda5050Protocol] Visualization publish exception: " << e.what() << std::endl;
    }
}

std::string Vda5050Protocol::makeVisualizationMessage(IAmr* amr)
{
    static int visualization_header_id_ = 0;

    if(!amr)
        return {};

    nlohmann::json viz_json;

    viz_json["headerId"] = visualization_header_id_++;
    viz_json["timestamp"] = getCurrentTimestampISO8601();
    viz_json["version"] = "2.1.0";
    viz_json["manufacturer"] = "ZenixRobotics";
    viz_json["serialNumber"] = agv_id_;

    double x = 0.0, y = 0.0, theta = 0.0;
    amr->getVcu()->getEstimatedPose(x, y, theta);

    viz_json["agvPosition"] = {
        {"x", x},
        {"y", y},
        {"theta", theta},
        {"mapId", "default_map"},
        {"positionInitialized", true}
    };
    viz_json["agvPosition"]["mapDescription"] = "";
    viz_json["agvPosition"]["localizationScore"] = 0.95;
    viz_json["agvPosition"]["deviationRange"] = 0.0;

    viz_json["velocity"] = {
        {"vx", amr->getVcu()->getMotor().getLinearVelocity()},
        {"vy", 0.0},
        {"omega", amr->getVcu()->getMotor().getAngularVelocity()}
    };

    return viz_json.dump();
}

// STATE MESSAGE PUBLISHING
void Vda5050Protocol::publishStateMessage(IAmr* amr)
{
    if (!amr_ || !mqtt_client_ || !mqtt_client_->is_connected()) 
    {
        std::cerr << "[Vda5050Protocol] Cannot publish state: AMR or MQTT not available" << std::endl;
        return;
    }

    //State 발행 직전에 완료 여부를 체크하여 최신 상태 동기화
    checkOrderCompletion(amr);

    try 
    {
        std::string state_msg = makeStateMessage(amr);
        if (!state_msg.empty()) 
        {
            auto msg = mqtt::make_message(state_topic_, state_msg);
            msg->set_qos(1);
            mqtt_client_->publish(msg);

            // 송신 로그
            logMessage("OUT", state_topic_, state_msg);

            // std::cout << "[Vda5050Protocol] State message published successfully" << std::endl;
        }
    } 
    catch (const std::exception& e) 
    {
        std::cerr << "[Vda5050Protocol] State publish exception: " << e.what() << std::endl;
    }
}

// STATE MESSAGE CREATION (VDA5050 2.1 compliant)
std::string Vda5050Protocol::makeStateMessage(IAmr* amr)
{
    if (!amr) 
    {
        std::cerr << "[Vda5050Protocol] Cannot create state message: AMR is null" << std::endl;
        return {};
    }
    
    nlohmann::json state_json;
    
    // 1. Required Header Fields
    state_json["headerId"] = state_header_id_++;
    state_json["timestamp"] = getCurrentTimestampISO8601();
    state_json["version"] = "2.1.0";
    state_json["manufacturer"] = "ZenixRobotics";
    state_json["serialNumber"] = agv_id_;
    
    // 2. Required Order Information
    state_json["orderId"] = current_order_id_.empty() ? "" : current_order_id_;
    state_json["orderUpdateId"] = current_order_update_id_;
    
    if(!current_zone_set_id_.empty())
    {
        state_json["zoneSetId"] = current_zone_set_id_;
    }
    
    // 3. Required Position Information
    state_json["lastNodeId"] = getLastNodeId(amr);
    state_json["lastNodeSequenceId"] = getLastNodeSequenceId(amr);
    
    // 4. Required Driving State
    std::string amr_state = amr->getState();
    bool is_driving = (amr_state.find("MOVING") != std::string::npos ||
                      amr_state.find("DRIVING") != std::string::npos);
    state_json["driving"] = is_driving;
    
    state_json["paused"] = false; 
    state_json["newBaseRequest"] = false; 
    state_json["distanceSinceLastNode"] = getDistanceSinceLastNode(amr);
    
    // 5. Required Operating Mode
    state_json["operatingMode"] = "AUTOMATIC";
    
    // 6. Node States Array (Requirements 1, 2, 3, 4, 5, 6)
    nlohmann::json node_states = nlohmann::json::array();
    
    // IDLE state: No order or order completed (Requirements 1, 2)
    if (!order_active_ || (current_order_id_.empty() && received_nodes_.empty()))
    {
        // nodeStates must be empty array in IDLE state
        std::cout << "[Vda5050Protocol] IDLE state - empty nodeStates" << std::endl;
    }
    else
    {
        // Order execution in progress (Requirements 3, 4, 5, 6)
        auto completed_nodes = amr->getCompletedNodes();
        std::set<std::string> completed_node_ids;
        
        // Build set of completed node IDs
        for (const auto& node : completed_nodes)
        {
            completed_node_ids.insert(node.nodeId);
        }
        
        // Include all nodes from FMS order (Requirement 4)
        // Exclude completed nodes (Requirement 5)
        for (const auto& node : received_nodes_)
        {
            // Skip completed nodes
            if (completed_node_ids.find(node.nodeId) != completed_node_ids.end())
            {
                // std::cout <<"skip : " << node.nodeId << std::endl;
                continue;
            }
            
            nlohmann::json node_state = {
                {"nodeId", node.nodeId},
                {"sequenceId", node.sequenceId},
                {"released", node.released}  // Use original release value from order (Requirement 6)
            };
            
            if (node.hasNodePosition)
            {
                node_state["nodePosition"] = {
                    {"x", node.nodePosition.x},
                    {"y", node.nodePosition.y},
                    {"mapId", node.nodePosition.mapId},
                    {"theta", node.nodePosition.theta},
                    // {"positionInitialized", node.nodePosition.positionInitialized}   //25.12.26, pisces 요청사항
                };
            }
            
            node_states.push_back(node_state);
        }
    }
    // std::cout << "pub_node : " << std::endl;
    // std::cout << node_states << std::endl;
    state_json["nodeStates"] = node_states;
    
    // 7. Edge States Array (Requirements 1, 2, 3, 4, 5, 6)
    nlohmann::json edge_states = nlohmann::json::array();
    
    // IDLE state: No order or order completed (Requirements 1, 2)
    if (!order_active_ || (current_order_id_.empty() && received_edges_.empty()))
    {
        // edgeStates must be empty array in IDLE state
        std::cout << "[Vda5050Protocol] IDLE state - empty edgeStates" << std::endl;
    }
    else
    {
        // Order execution in progress (Requirements 3, 4, 5, 6)
        auto completed_edges = amr->getCompletedEdges();
        std::set<std::string> completed_edge_ids;
        
        // Build set of completed edge IDs
        for (const auto& edge : completed_edges)
        {
            completed_edge_ids.insert(edge.edgeId);
        }
        
        // Include all edges from FMS order (Requirement 4)
        // Exclude completed edges (Requirement 5)
        for (const auto& edge : received_edges_)
        {
            // Skip completed edges
            if (completed_edge_ids.find(edge.edgeId) != completed_edge_ids.end())
            {
                continue;
            }
            
            nlohmann::json edge_state = {
                {"edgeId", edge.edgeId},
                {"sequenceId", edge.sequenceId},
                {"released", edge.released}  // Use original release value from order (Requirement 6)
            };
            
            // Include trajectory information if present
            if (edge.hasTrajectory)
            {
                nlohmann::json trajectory = {
                    {"degree", edge.trajectory.degree},
                    {"knotVector", edge.trajectory.knotVector}
                };
                
                nlohmann::json control_points = nlohmann::json::array();
                for (const auto& cp : edge.trajectory.controlPoints)
                {
                    nlohmann::json point = {
                        {"x", cp.x},
                        {"y", cp.y}
                    };
                    if (cp.hasWeight)
                    {
                        point["weight"] = cp.weight;
                    }
                    control_points.push_back(point);
                }
                trajectory["controlPoints"] = control_points;
                edge_state["trajectory"] = trajectory;
            }
            
            edge_states.push_back(edge_state);
        }
    }
    
    state_json["edgeStates"] = edge_states;
    
    // 8. AGV Position (Required)
    double x = 0.0, y = 0.0, theta = 0.0;
    amr->getVcu()->getEstimatedPose(x, y, theta);
    
    state_json["agvPosition"] = {
        {"x", x},
        {"y", y},
        {"theta", theta},
        {"mapId", "default_map"},
        {"positionInitialized", true}
    };
    state_json["agvPosition"]["mapDescription"] = "";
    state_json["agvPosition"]["localizationScore"] = 0.95;
    state_json["agvPosition"]["deviationRange"] = 0.0;
    
    // 9. Velocity (Required)
    state_json["velocity"] = {
        {"vx", amr->getVcu()->getMotor().getLinearVelocity()},
        {"vy", 0.0},
        {"omega", amr->getVcu()->getMotor().getAngularVelocity()}
    };
    
    // 10. Loads (Required, nullable array)
    state_json["loads"] = nlohmann::json::array();
    
    // 11. Action States Array (Required)
    nlohmann::json action_states = nlohmann::json::array();
    
    auto current_actions = getCurrentActions(amr);
    for (const auto& action : current_actions) 
    {
        nlohmann::json action_state = {
            {"actionId", action.actionId},
            {"actionStatus", action.status}
        };
        
        // Optional fields
        if (!action.actionType.empty())
        {
            action_state["actionType"] = action.actionType;
        }
        if (!action.description.empty())
        {
            action_state["actionDescription"] = action.description;
        }
        if (!action.resultDescription.empty())
        {
            action_state["resultDescription"] = action.resultDescription;
        }
        
        action_states.push_back(action_state);
    }
    
    state_json["actionStates"] = action_states;
    
    // 12. Battery State (Required)
    state_json["batteryState"] = {
        {"batteryCharge", amr->getBatteryPercent()},
        {"charging", isCharging(amr)}
    };
    
    // Optional battery fields
    state_json["batteryState"]["batteryVoltage"] = 0.0;
    state_json["batteryState"]["batteryHealth"] = 100.0;
    state_json["batteryState"]["reach"] = 0.0;
    
    // 13. Errors Array (Required)
    nlohmann::json errors = nlohmann::json::array();
    if (has_order_rejection_error_)
    {
        nlohmann::json rejection_error = {
            {"errorType", order_rejection_error_type_},
            {"errorLevel", "FATAL"},
            {"errorDescription", order_rejection_error_description_}
        };
        rejection_error["errorReferences"] = nlohmann::json::array();
        errors.push_back(rejection_error);
        
        std::cout << "[Vda5050Protocol] Order rejection error included in state: " 
                  << order_rejection_error_type_ << std::endl;
    }    
    
    // Check for low battery
    if (amr->getBatteryPercent() < 20.0) 
    {
        nlohmann::json battery_error = {
            {"errorType", "BATTERY_LOW"},
            {"errorLevel", "WARNING"},
            {"errorDescription", "Battery level is below 20%"}
        };
        battery_error["errorReferences"] = nlohmann::json::array();
        errors.push_back(battery_error);
    }
    
    // Check for system errors
    if (amr_state.find("ERROR") != std::string::npos) 
    {
        nlohmann::json system_error = {
            {"errorType", "SYSTEM_ERROR"},
            {"errorLevel", "FATAL"},
            {"errorDescription", amr_state}
        };
        system_error["errorReferences"] = nlohmann::json::array();
        errors.push_back(system_error);
    }
    
    state_json["errors"] = errors;
    
    // 14. Information Array (Required)
    state_json["information"] = nlohmann::json::array();
    
    // 15. Safety State (Required)
    std::string estop_status = getEmergencyStopStatus(amr) ? "MANUAL" : "NONE";
    state_json["safetyState"] = {
        {"eStop", estop_status},
        {"fieldViolation", getFieldViolationStatus(amr)}
    };
    
    return state_json.dump();
}

void Vda5050Protocol::checkOrderCompletion(IAmr* amr)
{
    if (!order_active_ || !amr || received_edges_.empty())
        return;
    
    auto completed_nodes = amr->getCompletedNodes();
    auto completed_edges = amr->getCompletedEdges();
    
    // 디버깅: completed_nodes 내용 출력
    std::cout << "[checkOrderCompletion] Completed nodes list:" << std::endl;
    for (size_t i = 0; i < completed_nodes.size(); ++i)
    {
        std::cout << "  [" << i << "] " << completed_nodes[i].nodeId << std::endl;
    }
    
    // 순환 경로 감지
    bool is_circular = false;
    if (!completed_nodes.empty() && !ordered_nodes_.empty())
    {
        const std::string& start_node_id = completed_nodes.front().nodeId;
        const std::string& last_ordered_node_id = ordered_nodes_.back().nodeId;
        
        if (start_node_id == last_ordered_node_id)
        {
            is_circular = true;
            std::cout << "[Vda5050Protocol] Circular path detected: start='" << start_node_id 
                      << "' == end='" << last_ordered_node_id << "'" << std::endl;
        }
    }
    
    // 수정: 순환 경로(시작점과 끝점이 같은 경로)일 때 고유 노드 개수 계산
    // NE4(시작)->NS1->NE1->NS2->NE2->NS3->NE3->NS4->NE4(끝)
    size_t unique_node_count = completed_nodes.size();
    
    if (is_circular)
    {
        // 순환 경로: 시작 노드와 끝 노드가 같으므로 중복 카운트
        // 고유 노드 개수 = completed_nodes.size() (중복이 이미 제거되어야 함)
        // 하지만 실제로는 중복이 있을 수 있으므로 다시 계산
        std::set<std::string> unique_nodes;
        for (const auto& node : completed_nodes)
        {
            unique_nodes.insert(node.nodeId);
        }
        unique_node_count = unique_nodes.size();
        
        std::cout << "[Vda5050Protocol] Unique node count (set): " << unique_node_count 
                  << " (original: " << completed_nodes.size() << ")" << std::endl;
    }
    
    bool all_nodes_completed;
    if (is_circular)
    {
        // 순환: 고유 노드 개수 == ordered_nodes_ 크기
        all_nodes_completed = (unique_node_count == ordered_nodes_.size());
    }
    else
    {
        all_nodes_completed = (completed_nodes.size() == ordered_nodes_.size() + 1);
    }
    
    bool all_edges_completed = (completed_edges.size() >= received_edges_.size());

    // std::cout << "[Vda5050Protocol] Order progress: Nodes(" << completed_nodes.size() 
    //           << "/" << (is_circular ? ordered_nodes_.size() : ordered_nodes_.size() + 1) 
    //           << ", unique: " << unique_node_count
    //           << "), Edges(" << completed_edges.size() << "/" << received_edges_.size() 
    //           << ") [" << (is_circular ? "CIRCULAR" : "LINEAR") << "]" << std::endl;    
    
    // if (all_nodes_completed && all_edges_completed && !ordered_nodes_.empty())
    if (!received_edges_.empty() && all_edges_completed)
    {
        std::cout << "[Vda5050Protocol] Order completed - switching to IDLE state" << std::endl;
        order_active_ = false;

        // current_order_id_ = "";
        // current_order_update_id_ = 0;
        // current_zone_set_id_ = "";    
        
        received_nodes_.clear();
        received_edges_.clear();
        
        publishStateMessage(amr);

        std::cout << "[Vda5050Protocol] IDLE state confirmed. Ready for next order." << std::endl;
    }
}

// HELPER METHODS
std::string Vda5050Protocol::getCurrentNodeId(IAmr* amr)
{
    auto current_nodes = amr->getCurrentNodes();
    if (!current_nodes.empty())
    {
        return current_nodes[0].nodeId;
    }
    return "";
}

std::string Vda5050Protocol::getCurrentEdgeId(IAmr* amr)
{
    auto current_edges = amr->getCurrentEdges();
    if (!current_edges.empty())
    {
        return current_edges[0].edgeId;
    }
    return "";
}

std::vector<NodeInfo> Vda5050Protocol::getUpcomingNodes(IAmr* amr)
{
    return amr->getCurrentNodes();
}

std::vector<EdgeInfo> Vda5050Protocol::getUpcomingEdges(IAmr* amr)
{
    return amr->getCurrentEdges();
}


// ============================================================================
// Helper Functions
// ============================================================================

ActionInfo Vda5050Protocol::convertActionToActionInfo(const Action& action, const std::string& status)
{
    ActionInfo info;
    info.actionId = action.actionId;
    info.actionType = action.actionType;
    info.description = action.actionDescription;
    info.status = status;
    info.resultDescription = "";
    
    // actionParameters 변환
    info.actionParameters = nlohmann::json::array();
    for (const auto& param : action.actionParameters)
    {
        nlohmann::json param_json;
        param_json["key"] = param.key;
        param_json["value"] = param.value;
        info.actionParameters.push_back(param_json);
    }
    
    return info;
}

std::vector<ActionInfo> Vda5050Protocol::getCurrentActions(IAmr* amr)
{
    std::vector<ActionInfo> actions;
    
    if (!order_active_)
    {
        // In IDLE state, only report charging action if applicable
        if (isCharging(amr)) 
        {
            ActionInfo charging_action;
            charging_action.actionId = "charge_001";
            charging_action.actionType = "charge";
            charging_action.description = "Battery charging";
            charging_action.status = "RUNNING";
            charging_action.resultDescription = "Charging in progress";
            actions.push_back(charging_action);
        }
        return actions;
    }
    
    // During order execution: report actions from uncompleted nodes/edges
    auto completed_nodes = amr->getCompletedNodes();
    std::set<std::string> completed_node_ids;
    for (const auto& node : completed_nodes)
    {
        completed_node_ids.insert(node.nodeId);
    }
    
    for (const auto& node : received_nodes_)
    {
        if (completed_node_ids.find(node.nodeId) == completed_node_ids.end())
        {
            for (const auto& action : node.actions)
            {
                ActionInfo action_info;
                action_info.actionId = action.actionId;
                action_info.actionType = action.actionType;
                action_info.description = action.actionDescription;  // 필드명 수정
                
                // TODO: Get actual action status from AMR
                // 현재는 간단히 WAITING으로 설정
                action_info.status = "WAITING";  
                
                // actionParameters 복사
                // action.actionParameters는 vector<ActionParameter>
                // action_info.actionParameters는 nlohmann::json
                nlohmann::json params_json = nlohmann::json::array();
                for (const auto& param : action.actionParameters)
                {
                    nlohmann::json param_json;
                    param_json["key"] = param.key;
                    param_json["value"] = param.value;
                    params_json.push_back(param_json);
                }
                action_info.actionParameters = params_json;
                
                actions.push_back(action_info);
            }
        }
    }  
    
    auto completed_edges = amr->getCompletedEdges();
    std::set<std::string> completed_edge_ids;
    for (const auto& edge : completed_edges)
    {
        completed_edge_ids.insert(edge.edgeId);
    }
    
    for (const auto& edge : received_edges_)
    {
        if (completed_edge_ids.find(edge.edgeId) == completed_edge_ids.end())
        {
            for (const auto& action : edge.actions)
            {
                ActionInfo action_info;
                action_info.actionId = action.actionId;
                action_info.actionType = action.actionType;
                action_info.description = action.actionDescription;  // 필드명 수정
                
                // TODO: Get actual action status from AMR
                action_info.status = "WAITING";
                
                // actionParameters 복사
                nlohmann::json params_json = nlohmann::json::array();
                for (const auto& param : action.actionParameters)
                {
                    nlohmann::json param_json;
                    param_json["key"] = param.key;
                    param_json["value"] = param.value;
                    params_json.push_back(param_json);
                }
                action_info.actionParameters = params_json;
                
                actions.push_back(action_info);
            }
        }
    }
    
    return actions;
}

std::vector<ErrorInfo> Vda5050Protocol::getSystemErrors(IAmr* amr)
{
    std::vector<ErrorInfo> errors;
    
    // Battery warning
    if (amr->getBatteryPercent() < 20.0)
    {
        ErrorInfo error;
        error.errorType = "BATTERY_LOW";
        error.errorLevel = "WARNING";
        error.description = "Battery level is below 20%";
        errors.push_back(error);
    }
    
    // System errors based on AMR state
    std::string state = amr->getState();
    if (state.find("ERROR") != std::string::npos)
    {
        ErrorInfo error;
        error.errorType = "SYSTEM_ERROR";
        error.errorLevel = "FATAL";
        error.description = state;
        errors.push_back(error);
    }
    
    return errors;
}

bool Vda5050Protocol::getEmergencyStopStatus(IAmr* amr)
{
    // TODO: Implement actual E-Stop status check
    return false;
}

bool Vda5050Protocol::getFieldViolationStatus(IAmr* amr)
{
    // TODO: Implement actual safety zone violation check
    return false;
}

bool Vda5050Protocol::isCharging(IAmr* amr)
{
    std::string state = amr->getState();
    return state.find("CHARGING") != std::string::npos;
}

std::string Vda5050Protocol::getLastNodeId(IAmr* amr)
{
    return amr->getLastNodeId();
}

int Vda5050Protocol::getLastNodeSequenceId(IAmr* amr)
{
    return amr->getLastNodeSequenceId();
}

nlohmann::json Vda5050Protocol::getCurrentNodePosition(IAmr* amr)
{
    double x, y, theta;
    amr->getVcu()->getEstimatedPose(x, y, theta);
    
    return {
        {"x", x},
        {"y", y},
        {"theta", theta},
        {"mapId", "default_map"},
        {"positionInitialized", true}
    };
}

double Vda5050Protocol::getDistanceSinceLastNode(IAmr* amr)
{
    if (!amr)
        return 0.0;
    
    double current_x = 0.0, current_y = 0.0, current_theta = 0.0;
    amr->getVcu()->getEstimatedPose(current_x, current_y, current_theta);
    
    auto completed_nodes = amr->getCompletedNodes();
    if (completed_nodes.empty())
        return 0.0;
    
    const auto& last_node = completed_nodes.back();
    double dx, dy;
    
    if (!last_node.hasNodePosition)
    {
        // Use x, y if nodePosition is not available
        dx = current_x - last_node.x;
        dy = current_y - last_node.y;
    }
    else
    {
        dx = current_x - last_node.nodePosition.x;
        dy = current_y - last_node.nodePosition.y;
    }
    
    return std::hypot(dx, dy);
}

void Vda5050Protocol::enableLogging(const std::string &path)
{
    if (log_file_.is_open()) {
        log_file_.close();
    }
    log_file_.open(path, std::ios::out | std::ios::app);
    log_enabled_ = log_file_.is_open();

    if (!log_enabled_) {
        std::cerr << "[Vda5050Protocol] Failed to open log file: "
                  << path << std::endl;
    } else {
        std::cout << "[Vda5050Protocol] VDA5050 logging enabled: "
                  << path << std::endl;
    }
}

void Vda5050Protocol::disableLogging()
{
    if (log_file_.is_open()) {
        log_file_.close();
    }
    log_enabled_ = false;
    std::cout << "[Vda5050Protocol] VDA5050 logging disabled" << std::endl;
}

void Vda5050Protocol::logMessage(const std::string &direction,
                                 const std::string &topic,
                                 const std::string &payload)
{
    if (!log_enabled_ || !log_file_.is_open()) {
        return;
    }

    // ISO8601 타임스탬프는 기존 헬퍼 재사용
    const std::string ts = getCurrentTimestampISO8601();

    // 심플 JSON 라인 형태로 남김 (파싱하기 쉽도록)
    log_file_ << "{"
              << "\"timestamp\":\"" << ts << "\","
              << "\"direction\":\"" << direction << "\","
              << "\"topic\":\"" << topic << "\","
              << "\"payload\":" << payload   // payload 자체가 JSON 문자열이라고 가정
              << "}"
              << std::endl;
}