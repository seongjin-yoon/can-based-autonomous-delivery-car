#include <iostream>
#include <mosquittopp.h>
#include <sqlite3.h>
#include <nlohmann/json.hpp>
#include <thread>
#include <map>
#include <chrono>
#include <cstdlib>
#include <random>
#include <sstream>
#include <iomanip>
#include <filesystem>

using json = nlohmann::json;
namespace fs = std::filesystem;

class DeliveryServer : public mosqpp::mosquittopp {
private:
    sqlite3* db = nullptr;
    bool connected = false;
    std::string broker_host;
    int broker_port;
    std::string db_path;

public:
    // 생성자: MQTT 브로커 설정 초기화
    DeliveryServer(const std::string& host = "10.42.0.1",
        int port = 1883,
        const std::string& db_file = "/home/pi/catnip/database/delivery_system.db")
        : mosqpp::mosquittopp("rpi3-server"),
        broker_host(host),
        broker_port(port),
        db_path(db_file) {

        // MQTT 인증 정보 설정
        username_pw_set("hoji", "1234");
        std::cout << "✓ MQTT Client created with broker: " << broker_host
            << ":" << broker_port << std::endl;
    }

    ~DeliveryServer() {
        if (db) {
            sqlite3_close(db);
            std::cout << "\n✓ Database closed" << std::endl;
        }
    }

    bool start() {
        mosqpp::lib_init();

        // MQTT 연결 전에 데이터베이스 초기화
        std::cout << "\n🗄️  Initializing SQLite Database..." << std::endl;
        if (!init_database()) {
            std::cerr << "✗ Database initialization failed" << std::endl;
            return false;
        }
        std::cout << "✓ Database initialized successfully\n" << std::endl;

        // 환경 변수에서 브로커 주소 확인
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

        // 연결 대기 (최대 5초)
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

protected:
    void on_connect(int rc) override {
        if (rc == 0) {
            connected = true;
            std::cout << "✓ Connected to MQTT broker" << std::endl;

            // RPi4에서 주문 메시지 수신 (destination 포함: delivery/order/{order_id}/{destination})
            subscribe(nullptr, "delivery/order/+/+", 1);

            // RPi1에서 차량 정보 메시지 수신
            subscribe(nullptr, "delivery/vehicle/+", 1);

            std::cout << "✓ Topics subscribed (2 topics)" << std::endl;
        }
        else {
            std::cerr << "✗ Connection failed: " << rc << std::endl;
            print_error_code(rc);
        }
    }

    void on_message(const struct mosquitto_message* msg) override {
        std::string topic(msg->topic);
        std::string payload(static_cast<char*>(msg->payload), msg->payloadlen);

        std::cout << "\n[📨 메시지 수신]" << std::endl;
        std::cout << "  토픽: [" << topic << "]" << std::endl;

        try {
            auto data = json::parse(payload);

            if (topic.find("delivery/order/") != std::string::npos) {
                // ACK 응답 무시
                if (topic.find("/3to4") != std::string::npos) {
                    std::cout << "  → ACK 응답 (무시)" << std::endl;
                    return;
                }

                std::cout << "  → Order 메시지" << std::endl;
                
                // ✅ [FIX] 토픽에서 직접 목적지 추출 (JSON 무시)
                // 토픽 형식: delivery/order/{order_id}/{destination}
                // 예: delivery/order/Order_1/A
                
                std::string destination = "";
                
                // "delivery/order/" 다음 위치
                size_t pos = topic.find("delivery/order/");
                if (pos != std::string::npos) {
                    pos += strlen("delivery/order/");
                    
                    // order_id 다음 "/" 찾기
                    size_t slash = topic.find("/", pos);
                    if (slash != std::string::npos) {
                        // "/" 다음부터 끝까지 (또는 다음 "/"까지)
                        size_t next_slash = topic.find("/", slash + 1);
                        if (next_slash == std::string::npos) {
                            destination = topic.substr(slash + 1);
                        } else {
                            destination = topic.substr(slash + 1, next_slash - slash - 1);
                        }
                        
                        // 첫 글자만 (A, B, C, D)
                        if (!destination.empty()) {
                            destination = destination.substr(0, 1);
                        }
                    }
                }
                
                std::cout << "  ✅ 토픽에서 추출한 목적지: '" << destination << "'" << std::endl;
                
                // ✅ JSON의 destination을 토픽 값으로 강제 설정
                if (!destination.empty()) {
                    data["destination"] = destination;
                    std::cout << "  ✅ JSON 업데이트: destination = '" << destination << "'" << std::endl;
                }
                
                handle_order(data);
            }
            else if (topic.find("delivery/vehicle/") != std::string::npos) {
                std::cout << "  → Vehicle 메시지" << std::endl;

                if (data.contains("heartbeat") && data["heartbeat"] == true) {
                    handle_heartbeat(data);
                } else {
                    handle_vehicle(data);
                }
            }
        }
        catch (const std::exception& e) {
            std::cerr << "✗ JSON 파싱 오류: " << e.what() << std::endl;
        }
    }

    // ✅ 토픽 파싱은 on_message에서 직접 처리 (더 간단함)

    void on_disconnect(int rc) override {
        connected = false;
        std::cout << "⚠️  연결 해제됨 (rc: " << rc << ")" << std::endl;
    }

private:
    // MQTT 오류 코드 출력
    void print_error_code(int rc) {
        switch (rc) {
        case 1: std::cerr << "  오류: 허용되지 않는 프로토콜 버전" << std::endl; break;
        case 2: std::cerr << "  오류: 식별자 거부됨" << std::endl; break;
        case 3: std::cerr << "  오류: 서버 사용 불가" << std::endl; break;
        case 4: std::cerr << "  오류: 잘못된 사용자명 또는 비밀번호" << std::endl; break;
        case 5: std::cerr << "  오류: 권한 없음" << std::endl; break;
        default: std::cerr << "  오류 코드: " << rc << std::endl;
        }
    }

    // 데이터베이스 초기화 및 테이블 생성
    bool init_database() {
        // 디렉토리 존재 확인 및 생성
        std::string dir_path = db_path.substr(0, db_path.find_last_of("/\\"));
        if (!fs::exists(dir_path)) {
            try {
                fs::create_directories(dir_path);
                std::cout << "  ✓ 데이터베이스 디렉토리 생성됨: " << dir_path << std::endl;
            }
            catch (const std::exception& e) {
                std::cerr << "✗ 디렉토리 생성 실패: " << e.what() << std::endl;
                return false;
            }
        }

        // 데이터베이스 파일 열기 또는 생성
        int rc = sqlite3_open(db_path.c_str(), &db);
        if (rc != SQLITE_OK) {
            std::cerr << "✗ 데이터베이스 열기 실패: " << sqlite3_errmsg(db) << std::endl;
            return false;
        }

        std::cout << "  ✓ 데이터베이스 파일 열림: " << db_path << std::endl;

        // 테이블 생성 (이미 존재하면 스킵)
        const char* create_tables = R"(
            CREATE TABLE IF NOT EXISTS vehicle_table (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                vehicle_id TEXT NOT NULL
            );

            CREATE TABLE IF NOT EXISTS order_table (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                order_id TEXT NOT NULL,
                destination TEXT NOT NULL,
                pin TEXT NOT NULL,
                menus TEXT,
                vehicle_id TEXT,
                status TEXT DEFAULT 'pending'
            );

            CREATE TABLE IF NOT EXISTS heartbeat_table (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                vehicle_id TEXT NOT NULL,
                status TEXT DEFAULT 'active'
            );

            CREATE INDEX IF NOT EXISTS idx_vehicle_id ON vehicle_table(vehicle_id);
            CREATE INDEX IF NOT EXISTS idx_vehicle_ts ON vehicle_table(timestamp);
            CREATE INDEX IF NOT EXISTS idx_order_id ON order_table(order_id);
            CREATE INDEX IF NOT EXISTS idx_order_ts ON order_table(timestamp);
            CREATE INDEX IF NOT EXISTS idx_order_dest ON order_table(destination);
            CREATE INDEX IF NOT EXISTS idx_order_status ON order_table(status);
            CREATE INDEX IF NOT EXISTS idx_heartbeat_vehicle_id ON heartbeat_table(vehicle_id);
            CREATE INDEX IF NOT EXISTS idx_heartbeat_ts ON heartbeat_table(timestamp);
        )";

        char* err = nullptr;
        rc = sqlite3_exec(db, create_tables, nullptr, nullptr, &err);
        if (rc != SQLITE_OK) {
            std::cerr << "✗ 테이블 생성 실패: " << err << std::endl;
            sqlite3_free(err);
            return false;
        }

        std::cout << "  ✓ 테이블 생성 완료" << std::endl;
        return true;
    }

    // Prepared Statement를 사용한 쿼리 실행
    bool execute_prepared_query(const std::string& query,
        const std::vector<std::string>& params) {
        if (!db) {
            std::cerr << "✗ 데이터베이스가 초기화되지 않음" << std::endl;
            return false;
        }

        sqlite3_stmt* stmt = nullptr;
        int rc = sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, nullptr);

        if (rc != SQLITE_OK) {
            std::cerr << "✗ Prepare 실패: " << sqlite3_errmsg(db) << std::endl;
            return false;
        }

        // 파라미터 바인딩
        for (size_t i = 0; i < params.size(); ++i) {
            sqlite3_bind_text(stmt, i + 1, params[i].c_str(), -1, SQLITE_STATIC);
        }

        // 쿼리 실행
        rc = sqlite3_step(stmt);
        sqlite3_finalize(stmt);

        return (rc == SQLITE_DONE);
    }

    // 주문 메시지 처리 (RPi4에서 수신)
    void handle_order(const json& data) {
        try {
            // 필수 필드 추출
            std::string order_id = data.at("order_id").get<std::string>();
            std::string destination = data.at("destination").get<std::string>();
            std::string pin = data.at("pin").get<std::string>();
            std::string vehicle_id = data.value("vehicle_id", "vehicle_001");

            // ✅ PIN 검증 (4자리 확인)
            if (pin.empty() || pin.length() != 4) {
                throw std::runtime_error("PIN 형식 오류: 4자리여야 합니다 (입력값: " + pin + ")");
            }

            std::cout << "✓ 주문 수신 성공" << std::endl;
            std::cout << "  주문 ID: " << order_id << std::endl;
            std::cout << "  목적지: " << destination << std::endl;
            std::cout << "  PIN: " << pin << " (길이: " << pin.length() << " 자리)" << std::endl;

            // 메뉴 배열을 문자열로 변환
            std::string menus_str;
            if (data.contains("menus") && data["menus"].is_array()) {
                auto menus_arr = data["menus"];
                for (size_t i = 0; i < menus_arr.size(); ++i) {
                    if (i > 0) menus_str += ", ";
                    menus_str += menus_arr[i].get<std::string>();
                }
            }

            // ✅ 데이터베이스에 주문 정보 저장 (PIN 포함)
            std::string insert_query =
                "INSERT INTO order_table (order_id, destination, pin, menus, vehicle_id, status) "
                "VALUES (?, ?, ?, ?, ?, 'pending')";

            std::cout << "\n📝 데이터베이스 저장 시도:" << std::endl;
            std::cout << "  쿼리: INSERT INTO order_table" << std::endl;
            std::cout << "  order_id: " << order_id << std::endl;
            std::cout << "  destination: " << destination << std::endl;
            std::cout << "  pin: " << pin << std::endl;
            std::cout << "  menus: " << menus_str << std::endl;
            std::cout << "  vehicle_id: " << vehicle_id << std::endl;

            if (!execute_prepared_query(insert_query,
                { order_id, destination, pin, menus_str, vehicle_id })) {
                throw std::runtime_error("주문 레코드 삽입 실패 (DB 오류)");
            }

            std::cout << "✅ 주문이 데이터베이스에 저장됨" << std::endl;

            // ✅ 저장된 데이터 검증 (SELECT로 확인)
            verify_order_in_database(order_id, pin);

            // ✅ [NEW] 주문 정보(PIN 포함)를 RPi1(배송 로봇)으로 전송
            send_order_to_vehicle(order_id, destination, pin, menus_str, vehicle_id);

            // ACK 응답 전송
            json ack = {
                {"order_id", order_id},
                {"timestamp", get_current_timestamp()},
                {"status", "acknowledged"},
                {"message", "주문이 수락되었습니다"}
            };

            std::string ack_topic = "delivery/order/" + order_id + "/3to4";
            publish_message(ack_topic, ack, 1);

            std::cout << "  ✓ ACK 응답 전송 완료" << std::endl;

        }
        catch (const std::exception& e) {
            std::cerr << "✗ handle_order 오류: " << e.what() << std::endl;

            try {
                std::string order_id = data.at("order_id").get<std::string>();

                // 에러 응답 전송
                json error_ack = {
                    {"order_id", order_id},
                    {"timestamp", get_current_timestamp()},
                    {"status", "error"},
                    {"message", e.what()}
                };
                std::string ack_topic = "delivery/order/" + order_id + "/3to4";
                publish_message(ack_topic, error_ack, 1);
            }
            catch (...) {
                std::cerr << "  ✗ 오류 응답 전송 실패" << std::endl;
            }
        }
    }

    // ✅ [NEW] 데이터베이스에 저장된 주문 검증 함수
    void verify_order_in_database(const std::string& order_id, const std::string& expected_pin) {
        if (!db) {
            std::cerr << "✗ 데이터베이스 초기화되지 않음" << std::endl;
            return;
        }

        sqlite3_stmt* stmt = nullptr;
        const char* query = "SELECT order_id, destination, pin, menus FROM order_table WHERE order_id = ? LIMIT 1";

        int rc = sqlite3_prepare_v2(db, query, -1, &stmt, nullptr);
        if (rc != SQLITE_OK) {
            std::cerr << "✗ Prepare 실패: " << sqlite3_errmsg(db) << std::endl;
            return;
        }

        sqlite3_bind_text(stmt, 1, order_id.c_str(), -1, SQLITE_STATIC);

        if (sqlite3_step(stmt) == SQLITE_ROW) {
            // 저장된 데이터 추출
            const char* db_order_id = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
            const char* db_destination = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
            const char* db_pin = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
            const char* db_menus = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));

            std::cout << "\n📊 데이터베이스 검증 결과:" << std::endl;
            std::cout << "  ✓ 주문 ID: " << db_order_id << std::endl;
            std::cout << "  ✓ 목적지: " << db_destination << std::endl;
            std::cout << "  ✓ PIN: " << db_pin << " (일치: "
                << (std::string(db_pin) == expected_pin ? "YES ✅" : "NO ❌") << ")" << std::endl;
            std::cout << "  ✓ 메뉴: " << db_menus << std::endl;

            if (std::string(db_pin) == expected_pin) {
                std::cout << "  ✅ PIN 저장 성공!" << std::endl;
            }
            else {
                std::cerr << "  ✗ PIN 불일치 오류!" << std::endl;
                std::cerr << "    예상: " << expected_pin << " / 저장됨: " << db_pin << std::endl;
            }
        }
        else {
            std::cerr << "✗ 데이터베이스에서 주문을 찾을 수 없음: " << order_id << std::endl;
        }

        sqlite3_finalize(stmt);
    }

    // ✅ [NEW] RPi1으로 주문 정보 전송 (PIN 포함)
    void send_order_to_vehicle(const std::string& order_id,
        const std::string& destination,
        const std::string& pin,
        const std::string& menus,
        const std::string& vehicle_id) {
        try {
            // ✅ PIN을 포함한 주문 메시지 구성
            json order_msg = {
                {"order_id", order_id},
                {"vehicle_id", vehicle_id},
                {"destination", destination},
                {"pin", pin},
                {"menus", menus},
                {"timestamp", get_current_timestamp()},
                {"message_type", "order_assignment"}
            };

            // RPi1(배송 로봇)으로 주문 정보 전송
            // 토픽: delivery/vehicle/{vehicle_id}/order
            std::string order_topic = "delivery/vehicle/" + vehicle_id + "/order";
            publish_message(order_topic, order_msg, 1);

            std::cout << "  ✓ 주문 정보가 RPi1로 전송됨 (주문ID: " << order_id
                << ", 목적지: " << destination
                << ", PIN: " << pin << ")" << std::endl;

        }
        catch (const std::exception& e) {
            std::cerr << "✗ send_order_to_vehicle 오류: " << e.what() << std::endl;
        }
    }

    // RPi1에서 수신한 차량 정보 메시지 처리
    void handle_vehicle(const json& data) {
        try {
            // vehicle_id 추출 및 검증
            std::string vehicle_id = data.at("vehicle_id").get<std::string>();
            std::cout << "✓ 차량 정보 수신됨: " << vehicle_id << std::endl;

            // 데이터베이스에 차량 레코드 저장
            std::string insert_query =
                "INSERT INTO vehicle_table (vehicle_id) VALUES (?)";

            if (!execute_prepared_query(insert_query, { vehicle_id })) {
                throw std::runtime_error("차량 레코드 삽입 실패");
            }
            std::cout << "  ✓ 차량 레코드가 데이터베이스에 저장됨" << std::endl;

            // RPi1에 확인 응답 전송
            json ack = {
                {"vehicle_id", vehicle_id},
                {"timestamp", get_current_timestamp()},
                {"status", "acknowledged"},
                {"message", "차량 정보가 수신되었습니다"}
            };

            std::string ack_topic = "delivery/vehicle/" + vehicle_id + "/3to1";
            publish_message(ack_topic, ack, 1);

            std::cout << "  ✓ 확인 응답이 RPi1로 전송됨" << std::endl;

        }
        catch (const std::exception& e) {
            std::cerr << "✗ handle_vehicle 오류: " << e.what() << std::endl;
        }
    }

    // ✅ 차량 Heartbeat 메시지 처리
    void handle_heartbeat(const json& data) {
        try {
            // vehicle_id 추출 및 검증
            std::string vehicle_id = data.at("vehicle_id").get<std::string>();
            std::string status = data.value("status", "active");

            std::cout << "✓ Heartbeat 수신됨: " << vehicle_id << " (상태: " << status << ")" << std::endl;

            // 데이터베이스에 heartbeat 레코드 저장
            std::string insert_query =
                "INSERT INTO heartbeat_table (vehicle_id, status) VALUES (?, ?)";

            if (!execute_prepared_query(insert_query, { vehicle_id, status })) {
                throw std::runtime_error("Heartbeat 레코드 삽입 실패");
            }
            std::cout << "  ✓ Heartbeat이 데이터베이스에 저장됨" << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "✗ handle_heartbeat 오류: " << e.what() << std::endl;
        }
    }

    // ✅ 차량의 마지막 heartbeat 시간 조회
    std::string get_vehicle_last_heartbeat(const std::string& vehicle_id) {
        if (!db) {
            std::cerr << "✗ 데이터베이스 초기화되지 않음" << std::endl;
            return "";
        }

        sqlite3_stmt* stmt = nullptr;
        std::string last_heartbeat = "미수신";

        const char* query =
            "SELECT timestamp FROM heartbeat_table "
            "WHERE vehicle_id = ? "
            "ORDER BY timestamp DESC LIMIT 1";

        int rc = sqlite3_prepare_v2(db, query, -1, &stmt, nullptr);
        if (rc != SQLITE_OK) {
            std::cerr << "✗ Prepare 실패: " << sqlite3_errmsg(db) << std::endl;
            return "";
        }

        sqlite3_bind_text(stmt, 1, vehicle_id.c_str(), -1, SQLITE_STATIC);

        if (sqlite3_step(stmt) == SQLITE_ROW) {
            last_heartbeat = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
        }

        sqlite3_finalize(stmt);
        return last_heartbeat;
    }

    // ✅ 모든 활성 차량 목록 조회 및 상태 출력
    void print_vehicle_status() {
        if (!db) {
            std::cerr << "✗ 데이터베이스 초기화되지 않음" << std::endl;
            return;
        }

        sqlite3_stmt* stmt = nullptr;
        const char* query =
            "SELECT DISTINCT vehicle_id FROM heartbeat_table "
            "ORDER BY vehicle_id";

        int rc = sqlite3_prepare_v2(db, query, -1, &stmt, nullptr);
        if (rc != SQLITE_OK) {
            std::cerr << "✗ Prepare 실패: " << sqlite3_errmsg(db) << std::endl;
            return;
        }

        std::cout << "\n📊 차량 활동 상태:" << std::endl;
        std::cout << "─────────────────────────────────────" << std::endl;

        bool has_vehicles = false;
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            has_vehicles = true;
            std::string vehicle_id = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
            std::string last_hb = get_vehicle_last_heartbeat(vehicle_id);

            std::cout << "  🚗 " << vehicle_id << ": " << last_hb << std::endl;
        }

        if (!has_vehicles) {
            std::cout << "  (차량 데이터 없음)" << std::endl;
        }

        std::cout << "─────────────────────────────────────\n" << std::endl;
        sqlite3_finalize(stmt);
    }

    // 현재 타임스탬프를 YYYY-MM-DD HH:MM:SS 형식으로 반환
    std::string get_current_timestamp() {
        auto now = std::time(nullptr);
        auto tm = *std::localtime(&now);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
        return oss.str();
    }

    // MQTT 메시지 발행
    void publish_message(const std::string& topic, const json& data, int qos) {
        try {
            std::string payload = data.dump();
            int rc = publish(nullptr, topic.c_str(), payload.length(),
                payload.c_str(), qos, false);

            if (rc != MOSQ_ERR_SUCCESS) {
                std::cerr << "✗ 메시지 발행 실패 (rc=" << rc << ")" << std::endl;
            }
        }
        catch (const std::exception& e) {
            std::cerr << "✗ publish_message 오류: " << e.what() << std::endl;
        }
    }
};


int main(int argc, char* argv[]) {
    std::cout << "╔════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  RPi3 배송 서버 (MQTT 통신 + PIN 포함)         ║" << std::endl;
    std::cout << "║  ✅ 주문 수신 후 PIN을 포함하여 RPi1로 전송   ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════╝" << std::endl;

    std::string broker_host = "10.42.0.1";
    int broker_port = 1883;
    std::string db_file = "/home/pi/catnip/database/delivery_system.db";

    // 명령줄 인자 파싱
    if (argc > 1) {
        broker_host = argv[1];
        std::cout << "📝 명령줄 인자에서 브로커 설정: " << broker_host << std::endl;
    }
    else {
        const char* env_host = std::getenv("MQTT_BROKER_HOST");
        if (env_host) {
            broker_host = env_host;
            std::cout << "📝 환경 변수에서 브로커 설정: " << broker_host << std::endl;
        }
        else {
            std::cout << "📝 기본 브로커 사용: " << broker_host << std::endl;
        }
    }

    // 데이터베이스 경로 환경 변수 확인
    const char* env_db = std::getenv("DELIVERY_DB_PATH");
    if (env_db) {
        db_file = env_db;
        std::cout << "📝 환경 변수에서 데이터베이스 경로 설정: " << db_file << std::endl;
    }

    DeliveryServer server(broker_host, broker_port, db_file);
    if (!server.start()) {
        return 1;
    }

    std::cout << "\n✓ 서버 실행 중 (Ctrl+C로 종료)\n" << std::endl;

    // ✅ 주기적으로 차량 상태 출력 (30초마다)
    int status_check_counter = 0;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        status_check_counter++;
        if (status_check_counter % 60 == 0) {  // 30초마다 (500ms * 60 = 30s)
            // server.print_vehicle_status();  // ← 활성화하려면 public 메서드로 변경
            status_check_counter = 0;
        }
    }

    return 0;
}