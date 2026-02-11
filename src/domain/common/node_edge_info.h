#ifndef NODE_EDGE_INFO_H
#define NODE_EDGE_INFO_H

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

// Action 구조체 (nodes, edges, instantActions)
struct ActionParameter 
{
    std::string key;
    nlohmann::json value;  // array, boolean, number, string 모두 가능
};

struct Action 
{
    std::string actionType;           // Required
    std::string actionId;             // Required
    std::string actionDescription;    // Nullable
    std::string blockingType;         // Required
    std::vector<ActionParameter> actionParameters;  // Nullable
};

// ActionInfo (State 메시지에서 사용 - 실행 상태 포함)
struct ActionInfo 
{
    std::string actionId;
    std::string actionType;
    std::string description;
    std::string status;  // WAITING, INITIALIZING, RUNNING, PAUSED, FINISHED, FAILED
    std::string resultDescription;
    nlohmann::json actionParameters;
};

// NodePosition 구조체
struct NodePosition 
{
    double x;                         // Required
    double y;                         // Required
    double theta;                     // Nullable 
    double allowedDeviationXY;        // Nullable 
    double allowedDeviationTheta;     // Nullable 
    std::string mapId;                // Required
    std::string mapDescription;       // Nullable
    bool positionInitialized = true;
    
    bool hasTheta = false;
    bool hasAllowedDeviationXY = false;
    bool hasAllowedDeviationTheta = false;
};

// Node 구조체 (Order Schema)
struct NodeInfo 
{
    std::string nodeId;               // Required
    int sequenceId;                   // Required 
    std::string nodeDescription;      // Nullable
    bool released;                    // Required
    NodePosition nodePosition;        // Nullable
    std::vector<Action> actions;      // Required 
    
    bool hasNodePosition = false;
    
    // 기존 코드 호환성을 위한 간단한 위치 정보
    double x = 0.0;
    double y = 0.0;
};

// Trajectory 구조체 (NURBS)
struct ControlPoint 
{
    double x;                         // Required
    double y;                         // Required
    double weight;                    // Nullable   
    bool hasWeight = false;
};

struct Trajectory 
{
    int degree;                       // Optional
    std::vector<double> knotVector;   // Required 
    std::vector<ControlPoint> controlPoints;  // Required
    
    bool hasDegree = false;
};

// Edge 구조체 (Order Schema)
struct EdgeInfo 
{
    std::string edgeId;               // Required
    int sequenceId;                   // Required 
    std::string edgeDescription;      // Nullable
    bool released;                    // Required
    std::string startNodeId;          // Required
    std::string endNodeId;            // Required
    double maxSpeed;                  // Nullable
    double maxHeight;                 // Nullable
    double minHeight;                 // Nullable
    double orientation;               // Nullable 
    std::string orientationType;      // Nullable
    std::string direction;            // Nullable
    bool rotationAllowed;             // Nullable
    double maxRotationSpeed;          // Nullable
    double length;                    // Nullable
    Trajectory trajectory;            // Nullable
    std::vector<Action> actions;      // Required 
    
    // Nullable 필드 플래그
    bool hasMaxSpeed = false;
    bool hasMaxHeight = false;
    bool hasMinHeight = false;
    bool hasOrientation = false;
    bool hasOrientationType = false;
    bool hasDirection = false;
    bool hasRotationAllowed = false;
    bool hasMaxRotationSpeed = false;
    bool hasLength = false;
    bool hasTrajectory = false;

    struct TurnCenter 
    {
        double x;
        double y;
    }turnCenter;
    
    bool hasTurnCenter = false;
    
    // 기존 코드 호환성
    // std::string centerNodeId;
    // bool has_turn_center = false;
};

// Order 전체 구조체
struct OrderInfo 
{
    // Header
    int headerId;                     // Required
    std::string timestamp;            // Required 
    std::string version;              // Required
    std::string manufacturer;         // Required
    std::string serialNumber;         // Required
    
    // Order specific
    std::string orderId;              // Required
    int orderUpdateId;                // Required 
    std::string zoneSetId;            // Nullable
    
    std::vector<NodeInfo> nodes;      // Required
    std::vector<EdgeInfo> edges;      // Required
    
    bool hasZoneSetId = false;
};

// InstantActions 구조체
struct InstantActionsMessage 
{
    // Header
    int headerId;                     // Required
    std::string timestamp;            // Required 
    std::string version;              // Required
    std::string manufacturer;         // Required
    std::string serialNumber;         // Required
    
    std::vector<Action> actions;      // Required
};

// ========== Error 구조체 ==========

struct ErrorInfo 
{
    std::string errorType;
    std::string errorLevel;  // WARNING, FATAL
    std::string description;
    std::string hint;
    std::vector<std::string> errorReferences;
};

#endif // NODE_EDGE_INFO_H
