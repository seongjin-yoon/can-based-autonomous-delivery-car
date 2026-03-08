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
    std::string current_delivery_id;
    std::string vehicle_status;         // idle, ready, in_transit, arrived

    // ✅ MQTT 설정 멤버 변수
    std::string broker_host;
    int broker_port;

    // PIN 관련 (하나만 사용)
    std::string pin;                    // 통합 PIN
    int pin_attempt_count = 0;
    const int MAX_PIN_ATTEMPT = 5;

public:
    // ✅ 개선된 생성자: 호스트/포트를 매개변수로 받음
    VehicleOnboard(const std::string& vid,
        const std::string& host = "10.42.0.1",
        int port = 1883)
        : mosqpp::mosquittopp("rpi1-vehicle"),
        vehicle_id(vid),
        broker_host(host),
        broker_port(port),
        vehicle_status("idle") {

        // ✅ 데이터베이스 초기화
        std::string db_path = "/home/pi/catnip_vehicle/database/vehicle.db";
        
        sqlite3_open(db_path.c_str(), &db);
        if (!db) {
            std::cerr << "✗ Database open failed: " << db_path << std::endl;
        }
        else {
            std::cout << "✓ Database opened: " << db_path << std::endl;
            initialize_database();
        }

        // ✅ Mosquitto 인증 정보 설정
        username_pw_set("hoji", "1234");
        std::cout << "✓ MQTT Client created (Vehicle: " << vehicle_id << ")" << std::endl;
        std::cout << "  Broker: " << broker_host << ":" << broker_port << std::endl;
    }

    ~VehicleOnboard() {
        if (db) sqlite3_close(db);
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
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cerr << "✗ Connection timeout (waited 5 seconds)" << std::endl;
        return false;
    }

    // ✅ MQTT 메시지 발행
    void publish_message(const std::string& topic, const json& data, int qos) {
        if (!connected) {
            std::cerr << "⚠️  Not connected to MQTT broker" << std::endl;
            return;
        }

        std::string payload = data.dump();
        int ret = publish(nullptr, topic.c_str(), payload.length(),
            (const void*)payload.c_str(), qos, false);

        if (ret == MOSQ_ERR_SUCCESS) {
            std::cout << "  ✓ Published to " << topic << std::endl;
        }
        else {
            std::cerr << "✗ Publish failed to " << topic << std::endl;
        }
    }

    // ✅ 주기적으로 호출: 차량 상태 전송 (주기: 500ms, QoS: 0)
    void send_heartbeat() {
        if (!connected) return;

        json msg = {
            {"vehicle_id", vehicle_id},
            {"delivery_id", current_delivery_id},
            {"status", vehicle_status},
            {"timestamp", get_current_timestamp()}
        };

        publish_message("delivery/vehicle/" + vehicle_id + "/status", msg, 0);
    }

    // ✅ MQTT 메시지 수신
    void on_message(const struct mosquitto_message* msg) override {
        std::string topic(msg->topic);
        std::string payload(static_cast<char*>(msg->payload), msg->payloadlen);

        std::cout << "\n[📨] " << topic << std::endl;

        try {
            auto data = json::parse(payload);

            // ✅ PIN 정보 수신 (delivery/pin/{id}/3to1, QoS 2) ✅ CRITICAL FIX!
            if (topic.find("delivery/pin/") != std::string::npos &&
                topic.find("3to1") != std::string::npos) {
                handle_pin_from_server(data);
            }
        }
        catch (const std::exception& e) {
            std::cerr << "✗ JSON parse error: " << e.what() << std::endl;
        }
    }

    // ============================================================================
    // ✅ RPi3에서 온 PIN 정보 처리 - NEWLY IMPLEMENTED
    // ============================================================================
    void handle_pin_from_server(const json& data) {
        try {
            // 1️⃣ 데이터 추출
            current_delivery_id = data.at("delivery_id").get<std::string>();
            std::string vehicle_id_msg = data.at("vehicle_id").get<std::string>();
            std::string destination = data.at("destination").get<std::string>();
            std::string receiver = data.at("receiver").get<std::string>();
            pin = data.at("pin").get<std::string>();
            
            std::cout << "\n✓ PIN Information received from server" << std::endl;
            std::cout << "  Delivery ID: " << current_delivery_id << std::endl;
            std::cout << "  Destination: " << destination << std::endl;
            std::cout << "  Receiver: " << receiver << std::endl;
            std::cout << "  PIN: " << pin << std::endl;

            // 2️⃣ PIN을 DB에 저장 (로컬 백업) - 매개변수화 쿼리
            std::string pin_query =
                "INSERT INTO password_table "
                "(vehicle_id, delivery_id, pin_code, pin_type, expire_time) "
                "VALUES (?, ?, ?, 'single', datetime('now', '+30 minutes'))";
            
            if (execute_prepared_query(pin_query, {vehicle_id, current_delivery_id, pin})) {
                std::cout << "  ✓ PIN saved to local database" << std::endl;
            } else {
                std::cerr << "  ✗ Failed to save PIN to database" << std::endl;
            }

            // 3️⃣ 배달 정보를 DB에 저장 - 매개변수화 쿼리
            std::string delivery_query =
                "INSERT INTO delivery_table "
                "(delivery_id, vehicle_id, destination, receiver, status) "
                "VALUES (?, ?, ?, ?, 'ordered')";
            
            if (execute_prepared_query(delivery_query, {current_delivery_id, vehicle_id, destination, receiver})) {
                std::cout << "  ✓ Delivery info saved to local database" << std::endl;
            }

            // 4️⃣ 차량 상태 변경
            vehicle_status = "ready";
            pin_attempt_count = 0;
            
            // 5️⃣ event_log 기록
            log_event("pin_received", current_delivery_id, "info");
            
            std::cout << "✓ Vehicle is ready for delivery dispatch\n" << std::endl;
            
        } catch (const std::exception& e) {
            std::cerr << "✗ Error in handle_pin_from_server: " << e.what() << std::endl;
            log_event("pin_processing_error", "", "critical");
        }
    }

    // ============================================================================
    // 배달 관리 함수들
    // ============================================================================

    // ✅ 배달 시작 (운전자 버튼 입력 시 호출)
    void start_delivery() {
        if (vehicle_status != "ready") {
            std::cerr << "⚠️  Vehicle is not ready (status: " << vehicle_status << ")" << std::endl;
            return;
        }

        vehicle_status = "in_transit";

        json msg = {
            {"vehicle_id", vehicle_id},
            {"delivery_id", current_delivery_id},
            {"status", "in_transit"},
            {"timestamp", get_current_timestamp()}
        };

        publish_message("delivery/start/" + vehicle_id + "/1to3", msg, 1);
        log_event("delivery_started", current_delivery_id, "info");
        update_delivery_status("in_transit");

        std::cout << "✓ Delivery started: " << current_delivery_id << std::endl;
    }

    // ✅ 목적지 도착
    void notify_arrival() {
        if (vehicle_status != "in_transit") {
            std::cerr << "⚠️  Vehicle is not in transit" << std::endl;
            return;
        }

        vehicle_status = "arrived";

        json msg = {
            {"vehicle_id", vehicle_id},
            {"delivery_id", current_delivery_id},
            {"status", "arrived"},
            {"timestamp", get_current_timestamp()}
        };

        publish_message("delivery/arrived/" + vehicle_id + "/1to3", msg, 1);
        log_event("delivery_arrived", current_delivery_id, "info");
        update_delivery_status("arrived");

        std::cout << "✓ Delivery arrived: " << current_delivery_id << std::endl;
    }

    // ✅ 배달 완료
    void complete_delivery() {
        json msg = {
            {"vehicle_id", vehicle_id},
            {"delivery_id", current_delivery_id},
            {"status", "completed"},
            {"timestamp", get_current_timestamp()}
        };

        publish_message("delivery/complete/" + current_delivery_id + "/1to3", msg, 1);
        log_event("delivery_completed", current_delivery_id, "info");
        update_delivery_status("completed");

        // 배달 완료 후 상태 초기화
        current_delivery_id = "";
        pin = "";
        vehicle_status = "idle";
        pin_attempt_count = 0;

        std::cout << "✓ Delivery completed and reset" << std::endl;
    }

protected:
    void on_connect(int rc) override {
        if (rc == 0) {
            connected = true;
            std::cout << "✓ Connected to MQTT broker" << std::endl;

            // ✅ PIN 정보 구독 (RPi3 → RPi1)
            subscribe(nullptr, "delivery/pin/+/3to1", 2);  // QoS 2: 정확히 1회

            std::cout << "✓ Topics subscribed (1 topic)" << std::endl;
        }
        else {
            std::cerr << "✗ Connection failed: " << rc << std::endl;
            print_error_code(rc);
        }
    }

    void on_disconnect(int rc) override {
        connected = false;
        std::cout << "⚠️  Disconnected (rc: " << rc << ")" << std::endl;
    }

private:
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

    // ✅ 매개변수화 쿼리 실행 (SQL Injection 방지)
    bool execute_prepared_query(const std::string& query,
                                const std::vector<std::string>& params) {
        if (!db) {
            std::cerr << "✗ Database not initialized" << std::endl;
            return false;
        }

        sqlite3_stmt* stmt = nullptr;

        int rc = sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, nullptr);
        if (rc != SQLITE_OK) {
            std::cerr << "✗ Prepare failed: " << sqlite3_errmsg(db) << std::endl;
            return false;
        }

        for (size_t i = 0; i < params.size(); i++) {
            sqlite3_bind_text(stmt, i + 1, params[i].c_str(), -1, SQLITE_STATIC);
        }

        rc = sqlite3_step(stmt);
        if (rc != SQLITE_DONE) {
            std::cerr << "✗ Execute failed: " << sqlite3_errmsg(db) << std::endl;
            sqlite3_finalize(stmt);
            return false;
        }

        sqlite3_finalize(stmt);
        return true;
    }

    bool execute_query(const std::string& query) {
        char* err = nullptr;
        if (sqlite3_exec(db, query.c_str(), nullptr, nullptr, &err) != SQLITE_OK) {
            std::cerr << "  ✗ DB Error: " << err << std::endl;
            sqlite3_free(err);
            return false;
        }
        return true;
    }

    void log_event(const std::string& event_type, const std::string& delivery_id,
                   const std::string& severity) {
        std::string query =
            "INSERT INTO event_log (vehicle_id, delivery_id, event_type, severity) "
            "VALUES (?, ?, ?, ?)";
        execute_prepared_query(query, {vehicle_id, delivery_id, event_type, severity});
    }

    void update_delivery_status(const std::string& status) {
        std::string query =
            "UPDATE delivery_table SET status=? WHERE delivery_id=?";
        execute_prepared_query(query, {status, current_delivery_id});
    }

    // ============================================================================
    // ✅ 데이터베이스 초기화
    // ============================================================================
    void initialize_database() {
        // delivery_table
        execute_query(
            "CREATE TABLE IF NOT EXISTS delivery_table ("
            "id INTEGER PRIMARY KEY AUTOINCREMENT,"
            "delivery_id TEXT NOT NULL UNIQUE,"
            "vehicle_id TEXT NOT NULL,"
            "destination TEXT NOT NULL,"
            "receiver TEXT NOT NULL,"
            "order_time DATETIME DEFAULT CURRENT_TIMESTAMP,"
            "start_time DATETIME,"
            "arrive_time DATETIME,"
            "complete_time DATETIME,"
            "status TEXT DEFAULT 'ordered'"
            ");"
        );

        // password_table (PIN - 하나만)
        execute_query(
            "CREATE TABLE IF NOT EXISTS password_table ("
            "id INTEGER PRIMARY KEY AUTOINCREMENT,"
            "vehicle_id TEXT NOT NULL,"
            "delivery_id TEXT NOT NULL UNIQUE,"
            "pin_code TEXT NOT NULL UNIQUE,"
            "pin_type TEXT NOT NULL,"
            "created_time DATETIME DEFAULT CURRENT_TIMESTAMP,"
            "expire_time DATETIME,"
            "used INTEGER DEFAULT 0,"
            "attempt_count INTEGER DEFAULT 0"
            ");"
        );

        // event_log
        execute_query(
            "CREATE TABLE IF NOT EXISTS event_log ("
            "id INTEGER PRIMARY KEY AUTOINCREMENT,"
            "timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,"
            "vehicle_id TEXT,"
            "delivery_id TEXT,"
            "event_type TEXT NOT NULL,"
            "detail TEXT,"
            "severity TEXT DEFAULT 'info'"
            ");"
        );

        // 인덱스
        execute_query("CREATE INDEX IF NOT EXISTS idx_delivery_vehicle ON delivery_table(vehicle_id);");
        execute_query("CREATE INDEX IF NOT EXISTS idx_delivery_id ON delivery_table(delivery_id);");
        execute_query("CREATE INDEX IF NOT EXISTS idx_password_delivery ON password_table(delivery_id);");
        execute_query("CREATE INDEX IF NOT EXISTS idx_event_timestamp ON event_log(timestamp);");
        execute_query("CREATE INDEX IF NOT EXISTS idx_event_delivery_id ON event_log(delivery_id);");

        // PRAGMA
        execute_query("PRAGMA journal_mode = WAL;");
        execute_query("PRAGMA synchronous = NORMAL;");
        execute_query("PRAGMA cache_size = 10000;");

        std::cout << "✓ Database initialized successfully" << std::endl;
    }

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
    std::cout << "║  ✅ PIN 통일 (single PIN)                      ║" << std::endl;
    std::cout << "║  ✅ SQL Injection 방지                          ║" << std::endl;
    std::cout << "║  ✅ PIN 수신 처리 완성                          ║" << std::endl;
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

    // ✅ Heartbeat 스레드
    std::thread heartbeat_thread([&vehicle]() {
        while (true) {
            vehicle.send_heartbeat();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });
    heartbeat_thread.detach();

    // ✅ 메인 루프
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}
