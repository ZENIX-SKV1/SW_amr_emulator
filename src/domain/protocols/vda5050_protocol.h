// vda5050_protocol.h - VDA5050 2.1 Schema compliant with Order Merge support
#ifndef VDA5050_PROTOCOL_H
#define VDA5050_PROTOCOL_H

#include <string>
#include <memory>
#include <thread>
#include <vector>
#include <set>
#include <map>
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include "iprotocol.h"
#include "yaml_config.h"
#include "node_edge_info.h"
#include <fstream>

// Forward declarations
class IAmr;
struct AmrConfig;

class Vda5050Protocol : public IProtocol 
{
public:
    explicit Vda5050Protocol(const AmrConfig& config);
    virtual ~Vda5050Protocol();

    // IProtocol interface implementation
    void setAmr(IAmr* amr) override;
    void setAgvId(const std::string& agv_id) override;
    void useDefaultConfig(const std::string& server_address) override;
    void start() override;
    void stop() override;
    
    // Message publishing (outbound to FMS)
    void publishStateMessage(IAmr* amr) override;
    void publishVisualizationMessage(IAmr* amr) override;
    
    // Message handling (inbound from FMS)
    void handleMessage(const std::string& msg, IAmr* amr) override;
    
    // Message creation (for testing/debugging)
    std::string makeStateMessage(IAmr* amr) override;
    
    // Protocol type
    std::string getProtocolType() const override { return "VDA5050"; }
    
    // Order completion check
    void checkOrderCompletion(IAmr* amr) override;

    void enableLogging(const std::string &path);
    void disableLogging();

private:
    // MQTT callback handler
    class Vda5050MqttCallback : public mqtt::callback 
    {
    public:
        explicit Vda5050MqttCallback(Vda5050Protocol* proto) : proto_(proto) {}
        void message_arrived(mqtt::const_message_ptr msg) override;
    private:
        Vda5050Protocol* proto_;
    };

    // MQTT connection members
    std::string mqtt_server_uri_;
    std::unique_ptr<mqtt::async_client> mqtt_client_;
    std::shared_ptr<Vda5050MqttCallback> mqtt_callback_;
    mqtt::connect_options conn_opts_;
    
    // VDA5050 Topic names
    std::string state_topic_;
    std::string order_topic_;
    std::string instant_actions_topic;
    std::string visualization_topic_;
    std::string connection_topic_;
    std::string factsheet_topic_;
    
    // AMR configuration
    IAmr* amr_;
    AmrConfig config_;
    std::string agv_id_;
    bool running_;
    std::thread publish_thread_;
    
    // VDA5050 Header ID counters (incremental)
    int state_header_id_;
    int factsheet_header_id_;
    int connection_header_id_;
    
    // Current order tracking (VDA5050 standard)
    std::string current_order_id_;
    int current_order_update_id_;
    std::string current_zone_set_id_;
    bool order_active_;  // true when order is being executed, false when IDLE
    
    // Original order data from FMS (for state reporting)
    std::vector<NodeInfo> received_nodes_;  // All nodes from FMS order (complete path)
    std::vector<EdgeInfo> received_edges_;  // All edges from FMS order (complete path)
    std::vector<NodeInfo> ordered_nodes_; 

    bool has_order_rejection_error_;
    std::string order_rejection_error_type_;
    std::string order_rejection_error_description_;
    
    // Order Merge support - sequenceId 기반 빠른 검색
    std::map<int, NodeInfo> node_by_sequence_;   // sequenceId -> NodeInfo
    std::map<int, EdgeInfo> edge_by_sequence_;   // sequenceId -> EdgeInfo
    
    // Message creation functions
    std::string makeVisualizationMessage(IAmr* amr);
    std::string makeFactsheetMessage();
    std::string makeConnectMessage();
    
    // Instant action handling
    void handleInstantAction(const nlohmann::json& instant_action_json);

    void publishOrderRejectionError(const std::string& error_type, const std::string& error_description);
    
    // Order Merge 핵심 로직
    bool mergeOrder(const nlohmann::json& order_json);
    void parseOrderNodes(const nlohmann::json& nodes_json);
    void parseOrderEdges(const nlohmann::json& edges_json);
    bool validateOrder(const nlohmann::json& order_json, std::string& error_type, std::string& error_desc);
    bool validateStartNodePosition(const NodeInfo& start_node, double current_x, double current_y, 
                                   bool is_update_order, std::string& error_desc);
    std::string findNearestNodeInCurrentPath(double current_x, double current_y);
    
    // Utility functions
    std::string getCurrentTimestampISO8601();
    std::string detectConnection();
    
    // State query helper functions
    std::string getCurrentNodeId(IAmr* amr);
    std::string getCurrentEdgeId(IAmr* amr);
    std::string getLastNodeId(IAmr* amr);
    int getLastNodeSequenceId(IAmr* amr);
    nlohmann::json getCurrentNodePosition(IAmr* amr);
    double getDistanceSinceLastNode(IAmr* amr);
    
    std::vector<NodeInfo> getUpcomingNodes(IAmr* amr);
    std::vector<EdgeInfo> getUpcomingEdges(IAmr* amr);
    std::vector<ActionInfo> getCurrentActions(IAmr* amr);
    std::vector<ErrorInfo> getSystemErrors(IAmr* amr);

    std::ofstream log_file_;
    bool log_enabled_ = false;

    void logMessage(const std::string &direction,
                    const std::string &topic,
                    const std::string &payload);
    
    // Safety status functions
    bool getEmergencyStopStatus(IAmr* amr);
    bool getFieldViolationStatus(IAmr* amr);
    bool isCharging(IAmr* amr);
    
    // Helper function for Action/ActionInfo conversion
    ActionInfo convertActionToActionInfo(const Action& action, const std::string& status);
};

#endif // VDA5050_PROTOCOL_H