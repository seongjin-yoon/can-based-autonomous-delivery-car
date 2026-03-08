#include <iostream>
#include <mosquittopp.h>
#include <sqlite3.h>
#include <nlohmann/json.hpp>
#include <thread>
#include <map>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <algorithm>

using json = nlohmann::json;

class VehicleOnboard : public mosqpp::mosquittopp {
private:
    sqlite3* db = nullptr;
    bool connected = false;
    std::string vehicle_id;

    // ✅ MQTT 설정 멤버 변수
    std::string broker_host;
    int broker_port;

    // ✅ Keep-Alive 및 주기적 업데이트를 위한 스레드
    std::thread heartbeat_thread;
    volatile bool running = true;

    // ✅ [NEW] 현재 할당된 destination 정보
    std::string current_destination = "";

public:
    // ✅ 개선된 생성자: 호스트/포트를 매개변수로 받음
    VehicleOnboard(const std::string& vid,
        const std::string& host = "10.42.0.1",
        int port = 1883)
        : mosqpp::mosquittopp("rpi1-vehicle"),
        vehicle_id(vid),
        broker_host(host),
        broker_port(port) {

        // ✅ Mosquitto 인증 정보 설정
        username_pw_set("hoji", "1234");
        std::cout << "✓ MQTT Client created (Vehicle: " << vehicle_id << ")" << std::endl;
        std::cout << "  Broker: " << broker_host << ":" << broker_port << std::endl;
    }

    ~VehicleOnboard() {
        running = false;
        if (heartbeat_thread.joinable()) {
            heartbeat_thread.join();
        }
    }

    bool start() {
        mosqpp::lib_init();

        const char* env_host = std::getenv("MQTT_BROKER_HOST");
        if (env_host) {
            broker_host = env_host;
            std::cout << "📝 Using MQTT broker from env: " << broker_host << std::endl;
        }

        std::cout << "🔗 Connecting to MQTT broker " << broker_host
            << ":" << broker_port << "..." << std::endl;

        int rc = connect(broker_host.c_str(), broker_port, 60);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "✗ MQTT connection failed (rc=" << rc << ")" << std::endl;
            print_error_code(rc);
            return false;
        }

        if (loop_start() != MOSQ_ERR_SUCCESS) {
            std::cerr << "✗ Loop start failed" << std::endl;
            return false;
        }

        // ✅ 연결 대기 (최대 5초)
        for (int i = 0; i < 50; i++) {
            if (connected) {
                std::cout << "✓ Connected to MQTT broker" << std::endl;

                // ✅ Keep-Alive 스레드 시작
                start_heartbeat();
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cerr << "✗ Connection timeout (waited 5 seconds)" << std::endl;
        return false;
    }

    // ✅ Keep-Alive 메시지 발행 (주기적으로 상태 전송)
    void start_heartbeat() {
        heartbeat_thread = std::thread([this]() {
            int heartbeat_interval = 10;  // 10초마다 heartbeat
            int counter = 0;

            while (running) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                counter++;

                // 10초마다 heartbeat 전송
                if (counter >= heartbeat_interval) {
                    if (connected) {
                        publish_heartbeat();
                    }
                    counter = 0;
                }
            }
            });
        heartbeat_thread.detach();
    }

    // ✅ Heartbeat 메시지 발행
    void publish_heartbeat() {
        json heartbeat = {
            {"vehicle_id", vehicle_id},
            {"timestamp", get_current_timestamp()},
            {"status", "active"},
            {"heartbeat", true}
        };
        publish_message(heartbeat);
    }

    // ✅ MQTT 메시지 발행 (delivery/vehicle/{vehicle_id})
    void publish_message(const json& data) {
        if (!connected) {
            std::cerr << "⚠️  Not connected to MQTT broker" << std::endl;
            return;
        }

        std::string topic = "delivery/vehicle/" + vehicle_id;
        std::string payload = data.dump();
        int ret = publish(nullptr, topic.c_str(), payload.length(),
            (const void*)payload.c_str(), 1, false);

        if (ret == MOSQ_ERR_SUCCESS) {
            std::cout << "  ✓ Published to " << topic << std::endl;
        }
        else {
            std::cerr << "✗ Publish failed to " << topic << std::endl;
        }
    }

    // ✅ 차량 상태 업데이트 메시지 발행 (외부에서 호출)
    void publish_status(const std::string& status_type, const json& data) {
        if (!connected) {
            std::cerr << "⚠️  Not connected to MQTT broker" << std::endl;
            return;
        }

        json status_msg = {
            {"vehicle_id", vehicle_id},
            {"timestamp", get_current_timestamp()},
            {"status_type", status_type},
            {"data", data}
        };
        publish_message(status_msg);
    }

    // ✅ MQTT 메시지 수신 (delivery/vehicle/{vehicle_id})
    void on_message(const struct mosquitto_message* msg) override {
        std::string topic(msg->topic);
        std::string payload(static_cast<char*>(msg->payload), msg->payloadlen);

        std::cout << "\n[📨] " << topic << std::endl;

        try {
            auto data = json::parse(payload);

            // ✅ [NEW] Destination 정보 수신 처리 (delivery/vehicle/{vehicle_id}/destination)
            if (topic.find("delivery/vehicle/") != std::string::npos &&
                topic.find("/destination") != std::string::npos) {
                std::cout << "  ✓ Destination 정보 수신" << std::endl;
                handle_destination(data);
            }
            // ✅ ACK 응답 처리
            else if (data.contains("status") && data["status"] == "acknowledged") {
                std::cout << "  ✓ Message acknowledged by server" << std::endl;
            }
        }
        catch (const std::exception& e) {
            std::cerr << "✗ JSON parse error: " << e.what() << std::endl;
        }
    }

protected:
    void on_connect(int rc) override {
        if (rc == 0) {
            connected = true;
            std::cout << "✓ Connected to MQTT broker" << std::endl;

            // ✅ MQTT Topic 구독 (delivery/vehicle/{vehicle_id})
            std::string topic = "delivery/vehicle/" + vehicle_id;
            subscribe(nullptr, topic.c_str(), 1);  // QoS 1

            // ✅ [NEW] Destination 토픽 구독 (delivery/vehicle/{vehicle_id}/destination)
            std::string destination_topic = "delivery/vehicle/" + vehicle_id + "/destination";
            subscribe(nullptr, destination_topic.c_str(), 1);  // QoS 1

            std::cout << "✓ Topics subscribed:" << std::endl;
            std::cout << "  - " << topic << std::endl;
            std::cout << "  - " << destination_topic << std::endl;
        }
        else {
            std::cerr << "✗ Connection failed: " << rc << std::endl;
            print_error_code(rc);
        }
    }

    void on_disconnect(int rc) override {
        connected = false;
        std::cout << "⚠️  Disconnected (rc: " << rc << ")" << std::endl;

        // ✅ 연결 끊김 이유 출력
        switch (rc) {
        case 0:
            std::cout << "  Reason: Client initiated disconnect" << std::endl;
            break;
        case 1:
            std::cout << "  Reason: Unexpected disconnect" << std::endl;
            break;
        case 7:
            std::cout << "  Reason: Connection lost (MOSQ_ERR_CONN_LOST)" << std::endl;
            std::cout << "  Solution: Keep-Alive mechanism or server timeout" << std::endl;
            break;
        default:
            std::cout << "  Reason: Unknown (rc=" << rc << ")" << std::endl;
        }
    }

private:
    // ✅ [NEW] Destination 정보 처리 (RPi3에서 수신)
    void handle_destination(const json& data) {
        try {
            // destination 추출 (문자: A, B, C, D)
            std::string destination = data.at("destination").get<std::string>();
            std::string order_id = data.value("order_id", "unknown");

            // 현재 할당된 destination 업데이트
            current_destination = destination;

            std::cout << "════════════════════════════════════════" << std::endl;
            std::cout << "🎯 Destination Assignment Received" << std::endl;
            std::cout << "════════════════════════════════════════" << std::endl;
            std::cout << "  Order ID: " << order_id << std::endl;
            std::cout << "  Destination: " << destination << std::endl;
            std::cout << "  Vehicle: " << vehicle_id << std::endl;
            std::cout << "  Timestamp: " << data.value("timestamp", "N/A") << std::endl;
            std::cout << "════════════════════════════════════════" << std::endl;

            // ✅ 수신 확인 메시지를 RPi3로 전송
            send_destination_acknowledgement(order_id, destination);

            // ✅ 시뮬레이션: 목적지로 이동 시작
            std::cout << "\n🚗 차량이 목적지 " << destination << "로 이동 중..." << std::endl;

        }
        catch (const std::exception& e) {
            std::cerr << "✗ handle_destination 오류: " << e.what() << std::endl;
        }
    }

    // ✅ [NEW] RPi3로 Destination 수신 확인 전송
    void send_destination_acknowledgement(const std::string& order_id, const std::string& destination) {
        if (!connected) {
            std::cerr << "⚠️  Not connected to MQTT broker" << std::endl;
            return;
        }

        try {
            json ack_msg = {
                {"order_id", order_id},
                {"vehicle_id", vehicle_id},
                {"destination", destination},
                {"timestamp", get_current_timestamp()},
                {"status", "destination_acknowledged"},
                {"message", "차량이 목적지 정보를 수신했습니다"}
            };

            // RPi3로 ACK 전송 (delivery/vehicle/{vehicle_id}/ack)
            std::string ack_topic = "delivery/vehicle/" + vehicle_id + "/ack";
            std::string payload = ack_msg.dump();

            int ret = publish(nullptr, ack_topic.c_str(), payload.length(),
                (const void*)payload.c_str(), 1, false);

            if (ret == MOSQ_ERR_SUCCESS) {
                std::cout << "\n✓ Destination ACK sent to RPi3" << std::endl;
            }
            else {
                std::cerr << "✗ Destination ACK send failed" << std::endl;
            }
        }
        catch (const std::exception& e) {
            std::cerr << "✗ send_destination_acknowledgement 오류: " << e.what() << std::endl;
        }
    }

    // ✅ 오류 코드 출력
    void print_error_code(int rc) {
        switch (rc) {
        case 1: std::cerr << "  Error: Unacceptable protocol version" << std::endl; break;
        case 2: std::cerr << "  Error: Identifier rejected" << std::endl; break;
        case 3: std::cerr << "  Error: Server unavailable" << std::endl; break;
        case 4: std::cerr << "  Error: Bad username or password" << std::endl; break;
        case 5: std::cerr << "  Error: Not authorised" << std::endl; break;
        default: std::cerr << "  Error code: " << rc << std::endl;
        }
    }

    // ✅ 현재 타임스탐프 반환
    std::string get_current_timestamp() {
        auto now = std::time(nullptr);
        auto tm = *std::localtime(&now);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
        return oss.str();
    }
};


int main(int argc, char* argv[]) {
    std::cout << "╔════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  RPi1 Vehicle Onboard System (MQTT)            ║" << std::endl;
    std::cout << "║  ✅ RPi3에서 Destination 정보 수신             ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════╝" << std::endl;

    std::string vehicle_id = "vehicle_001";
    std::string broker_host = "10.42.0.1";

    if (argc > 1) {
        vehicle_id = argv[1];
        std::cout << "📝 Using vehicle ID from argument: " << vehicle_id << std::endl;
    }

    if (argc > 2) {
        broker_host = argv[2];
        std::cout << "📝 Using broker from argument: " << broker_host << std::endl;
    }

    const char* env_vehicle = std::getenv("VEHICLE_ID");
    if (env_vehicle) {
        vehicle_id = env_vehicle;
        std::cout << "📝 Using vehicle ID from env: " << vehicle_id << std::endl;
    }

    const char* env_host = std::getenv("MQTT_BROKER_HOST");
    if (env_host) {
        broker_host = env_host;
        std::cout << "📝 Using broker from env: " << broker_host << std::endl;
    }

    VehicleOnboard vehicle(vehicle_id, broker_host, 1883);
    if (!vehicle.start()) {
        return 1;
    }

    std::cout << "\n✓ Vehicle system running (Ctrl+C to stop)\n" << std::endl;

    // ✅ 메인 루프 (시뮬레이션 예제)
    int simulation_counter = 0;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // ✅ 30초마다 위치 업데이트 시뮬레이션
        simulation_counter++;
        if (simulation_counter % 30 == 0) {
            json location_data = {
                {"latitude", 37.5 + (std::rand() % 100) / 10000.0},
                {"longitude", 127.0 + (std::rand() % 100) / 10000.0},
                {"speed", 30 + (std::rand() % 50)}
            };
            vehicle.publish_status("location_update", location_data);
        }
    }

    return 0;
}