#include <iostream>
#include <string>
#include <mqtt/async_client.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using json = nlohmann::json;

const std::string SERVER_ADDRESS("tcp://localhost:1883");
const std::string CLIENT_ID("fms_client");
const std::string ORDER_TOPIC("vda5050/agvs/amr_0/order");
const std::string STATE_TOPIC("vda5050/agvs/amr_0/state");
const std::string VISUALIZATION_TOPIC("vda5050/agvs/amr_0/visualization");
const int QOS = 1;

const std::string FACTSHEET_REQUEST_INSTANT_ACTION = R"({
    "headerId": "factsheet_request_1",
    "timestamp": 1650000000,
    "version": "2.1.0",
    "manufacturer": "FMSManufacturer",
    "serialNumber": "fms_001",
    "actions": [
        {
            "actionType": "factsheetRequest",
            "actionId": "req_001"
        }
    ]
})";

// MQTT 콜백 핸들러 클래스 정의
class Callback : public virtual mqtt::callback
{
public:
    Callback(rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher,
             rclcpp::Node::SharedPtr node)
        : pose_pub_(publisher), node_(node) {}

    void message_arrived(mqtt::const_message_ptr msg) override
    {
        std::cout << "[MQTT] Message arrived on topic: " << msg->get_topic() << std::endl;
        std::cout << "[MQTT] Payload:\n" << msg->to_string() << std::endl << std::endl;

        if (msg->get_topic() == VISUALIZATION_TOPIC)
        {
            std::cout << "[MQTT] Visualization message arrived\n";
            std::string payload = msg->to_string();
            try
            {
                auto j = json::parse(payload);
                auto pose_json = j.at("pose");
                double x = pose_json.at("x").get<double>();
                double y = pose_json.at("y").get<double>();
                double theta = pose_json.at("theta").get<double>();

                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp = node_->get_clock()->now();
                pose.header.frame_id = "map";
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = 0.0;

                tf2::Quaternion q;
                q.setRPY(0, 0, theta);
                pose.pose.orientation = tf2::toMsg(q);

                pose_pub_->publish(pose);
                std::cout << "[ROS2] Published pose to RViz" << std::endl;
            }
            catch (...)
            {
                std::cerr << "Failed to parse visualization json" << std::endl;
            }
        }
    }

    void connection_lost(const std::string &cause) override
    {
        std::cout << "[MQTT] Connection lost";
        if (!cause.empty())
            std::cout << ", cause: " << cause;
        std::cout << std::endl;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Node::SharedPtr node_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("fms_mqtt_publish");

    auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);

    mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);
    mqtt::connect_options connOpts;
    connOpts.set_clean_session(true);

    Callback cb(pose_pub, node);
    client.set_callback(cb);

    try
    {
        std::cout << "Connecting to the MQTT server..." << std::endl;
        mqtt::token_ptr conntok = client.connect(connOpts);
        conntok->wait();
        std::cout << "Connected!" << std::endl;

        client.subscribe("vda5050/agvs/amr_0/#", QOS)->wait();

        // Factsheet 요청 instantAction 메시지 발행
        {
            std::cout << "Publishing factsheet request instantAction message..." << std::endl;
            auto instant_msg = mqtt::make_message("vda5050/agvs/amr_0/instantActions", FACTSHEET_REQUEST_INSTANT_ACTION);
            instant_msg->set_qos(QOS);
            client.publish(instant_msg)->wait();
            std::cout << "Factsheet request message published!" << std::endl;
        }

        // Order 메시지 발행
        std::string payload = R"({
              "headerId": "header_test",
              "timestamp": 1650000000,
              "orderId": "order_test_1",
              "nodes": [
                {"nodeId": "N1", "sequenceId": 0, "nodePosition": {"x":0.0, "y":0.0, "theta":0.0}},
                {"nodeId": "N2", "sequenceId": 1, "nodePosition": {"x":3.0, "y":0.0, "theta":1.57}},
                {"nodeId": "N3", "sequenceId": 2, "nodePosition": {"x":3.0, "y":3.0, "theta":3.14}},
                {"nodeId": "N4", "sequenceId": 3, "nodePosition": {"x":0.0, "y":3.0, "theta":4.71}},
                {"nodeId": "N5", "sequenceId": 4, "nodePosition": {"x":0.0, "y":0.0, "theta":0.0}}
              ],
              "edges": [
                {"edgeId": "E1", "sequenceId": 0, "startNodeId": "N1", "endNodeId": "N2"},
                {"edgeId": "E2", "sequenceId": 1, "startNodeId": "N2", "endNodeId": "N3"},
                {"edgeId": "E3", "sequenceId": 2, "startNodeId": "N3", "endNodeId": "N4"},
                {"edgeId": "E4", "sequenceId": 3, "startNodeId": "N4", "endNodeId": "N5"}
              ]
            })";

        auto msg = mqtt::make_message(ORDER_TOPIC, payload);
        msg->set_qos(QOS);

        std::cout << "Publishing message to topic: " << ORDER_TOPIC << std::endl;
        client.publish(msg)->wait_for(std::chrono::seconds(10));
        std::cout << "Message published!" << std::endl;

        std::cout << "Waiting for state messages for 10 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(10));

        // ROS 2 spin
        rclcpp::spin(node);

        // 연결 종료
        client.unsubscribe("vda5050/agvs/amr_0/#")->wait();
        client.disconnect()->wait();
        std::cout << "Disconnected." << std::endl;
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << "Error: " << exc.what() << std::endl;
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}