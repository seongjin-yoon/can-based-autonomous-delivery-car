#include "mqtt_client.h"
#include <QJsonDocument>
#include <QJsonArray>
#include <QDateTime>
#include <QDebug>
#include <QTimer>
#include <thread>
#include <chrono>
#include <mqtt/async_client.h>
#include <cstdlib>
#include <cstdio>

// =============================================================================
// MQTTCallback 구현
// =============================================================================

MQTTClient::MQTTCallback::MQTTCallback(MQTTClient* parent)
    : m_parent(parent) {
}

void MQTTClient::MQTTCallback::connected(const std::string& /*cause*/) {
    qDebug() << "\n════════════════════════════════════════════";
    qDebug() << "✅ MQTT 브로커 연결 성공";
    qDebug() << "════════════════════════════════════════════";

    if (m_parent) {
        m_parent->m_connected = true;
        m_parent->subscribe_topics();

        qDebug() << "✓ 토픽 구독 요청 전송";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        qDebug() << "✓ 구독 준비 완료";

        emit m_parent->connection_changed(true);
    }
}

void MQTTClient::MQTTCallback::connection_lost(const std::string& /*cause*/) {
    qWarning() << "\n⚠️  MQTT 연결 끊김";

    if (m_parent) {
        m_parent->m_connected = false;
        emit m_parent->connection_changed(false);
    }
}

void MQTTClient::MQTTCallback::delivery_complete(mqtt::delivery_token_ptr /*token*/) {
    qDebug() << "✓ 메시지 전송 완료";
}

// =============================================================================
// MQTTClient 구현
// =============================================================================

MQTTClient::MQTTClient(const QString& broker_address,
    const QString& username,
    const QString& password,
    QObject* parent)
    : QObject(parent),
    m_broker_address(broker_address),
    m_username(username),
    m_password(password) {

    std::string client_id = "rpi4_client_qt6";

    try {
        std::string broker_str = broker_address.toStdString();

        // "tcp://" 프로토콜 자동 추가
        if (broker_str.find("://") == std::string::npos) {
            broker_str = "tcp://" + broker_str;
        }

        qDebug() << "════════════════════════════════════════════";
        qDebug() << "✓ MQTT 클라이언트 초기화 중";
        qDebug() << "════════════════════════════════════════════";

        // Paho MQTT 클라이언트 생성
        m_mqtt_client = std::make_unique<mqtt::client>(broker_str, client_id);
        m_callback = std::make_unique<MQTTCallback>(this);
        m_mqtt_client->set_callback(*m_callback);

        qDebug() << "✓ MQTT 클라이언트 객체 생성 완료";
    }
    catch (const std::exception& e) {
        qCritical() << "✗ MQTT 클라이언트 생성 실패:" << e.what();
        emit error_occurred(QString("MQTT 클라이언트 생성 실패: %1").arg(e.what()));
    }
}

MQTTClient::~MQTTClient() {
    if (m_connected) {
        try {
            disconnect();
        }
        catch (...) {
            // 소멸자에서의 예외 억제
        }
    }
}

void MQTTClient::connect() {
    if (!m_mqtt_client) {
        qCritical() << "✗ MQTT 클라이언트가 초기화되지 않음";
        return;
    }

    // 별도 스레드에서 연결 시도
    std::thread connection_thread([this]() {
        try {
            qDebug() << "\n🔗 MQTT 브로커 연결 시도...";

            // 연결 옵션 설정
            mqtt::connect_options conn_opts;
            conn_opts.set_clean_session(true);           // 클린 세션
            conn_opts.set_automatic_reconnect(1, 30);    // 자동 재연결 (1~30초)
            conn_opts.set_connect_timeout(5);            // 연결 타임아웃 5초

            // 인증 정보 설정
            conn_opts.set_user_name(m_username.toStdString());
            conn_opts.set_password(m_password.toStdString());

            // MQTT 브로커에 연결
            m_mqtt_client->connect(conn_opts);

            qDebug() << "✓ 연결 요청 전송 완료";
        }
        catch (const std::exception& e) {
            qCritical() << "✗ MQTT 연결 실패:" << e.what();
            emit error_occurred(QString("MQTT 연결 실패: %1").arg(e.what()));
        }
        });

    connection_thread.detach();
}

void MQTTClient::disconnect() {
    if (!m_mqtt_client) {
        return;
    }

    try {
        if (m_connected) {
            m_mqtt_client->disconnect();
            m_connected = false;
            qDebug() << "✓ MQTT 연결 해제 완료";
        }
    }
    catch (const std::exception& e) {
        qWarning() << "✗ MQTT 연결 해제 오류:" << e.what();
    }
}

bool MQTTClient::is_connected() const {
    return m_connected;
}

void MQTTClient::subscribe_topics() {
    if (!m_mqtt_client || !m_connected) {
        qWarning() << "✗ 구독 불가능: MQTT 미연결";
        return;
    }

    try {
        qDebug() << "📡 MQTT 토픽 구독 시작";
        qDebug() << "토픽: delivery/order/+/3to4";

        // "delivery/order/+/3to4" 토픽 구독 (QoS 1)
        m_mqtt_client->subscribe("delivery/order/+/3to4", 1);

        qDebug() << "✓ 토픽 구독 완료";
    }
    catch (const std::exception& e) {
        qCritical() << "✗ 토픽 구독 실패:" << e.what();
        emit error_occurred(QString("토픽 구독 실패: %1").arg(e.what()));
    }
}

void MQTTClient::publish_order(const QString& order_id,
    const QStringList& menus,
    int destination,
    const QString& pin) {
    if (!m_connected) {
        emit error_occurred("MQTT 브로커에 연결되지 않았습니다");
        return;
    }

    // 별도 스레드에서 메시지 발행 및 ACK 대기
    std::thread publish_thread([order_id, menus, destination, pin, this]() {
        try {
            // destination을 문자(A-D)로 변환
            QString destination_str;
            switch (destination) {
            case 1: destination_str = "A"; break;
            case 2: destination_str = "B"; break;
            case 3: destination_str = "C"; break;
            case 4: destination_str = "D"; break;
            default: destination_str = QString::number(destination); break;
            }

            // [1] 주문 JSON 객체 생성
            QJsonObject order_obj;
            order_obj["order_id"] = order_id;
            order_obj["vehicle_id"] = "vehicle_001";
            order_obj["timestamp"] = QDateTime::currentDateTime().toString(Qt::ISODate);
            order_obj["destination"] = destination_str;  // ✅ 문자형으로 저장
            order_obj["receiver"] = "Customer";

            // 메뉴를 JSON 배열로 변환
            QJsonArray menus_array;
            for (const QString& menu : menus) {
                menus_array.append(menu);
            }
            order_obj["menus"] = menus_array;
            order_obj["pin"] = pin;

            // JSON을 문자열로 변환 (컴팩트 형식)
            QJsonDocument doc(order_obj);
            QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));

            QString topic = QString("delivery/order/%1/%2").arg(order_id).arg(destination_str);

            qDebug() << "\n════════════════════════════════════════";
            qDebug() << "📤 MQTT 메시지 발행";
            qDebug() << "════════════════════════════════════════";
            qDebug() << "토픽: " << topic;
            qDebug() << "목적지: " << destination_str;
            qDebug() << "메뉴: " << menus;
            qDebug() << "════════════════════════════════════════";

            // [2] mosquitto_pub 명령어로 메시지 발행
            // JSON 페이로드에서 특수 문자 이스케이프 처리
            QString escaped_payload = payload;
            escaped_payload.replace("\\", "\\\\");
            escaped_payload.replace("\"", "\\\"");

            QString cmd = QString("mosquitto_pub -h 10.42.0.1 -u hoji -P 1234 "
                "-t %1 -m \"%2\"")
                .arg(topic)
                .arg(escaped_payload);

            int result = system(cmd.toStdString().c_str());

            if (result != 0) {
                qWarning() << "⚠️  mosquitto_pub 명령 실패";
                emit error_occurred("메시지 발행 실패");
                return;
            }

            qDebug() << "✓ 메시지 발행 완료";

            // [3] ACK 응답 대기 (최대 10초)
            qDebug() << "\n⏳ ACK 응답 대기 중 (타임아웃: 10초)...";

            QString ack_topic = QString("delivery/order/%1/3to4").arg(order_id);

            // mosquitto_sub로 ACK 수신 (timeout 10초)
            QString ack_cmd = QString("timeout 10 mosquitto_sub -h 10.42.0.1 -u hoji -P 1234 "
                "-t %1 -C 1 2>&1")
                .arg(ack_topic);

            FILE* pipe = popen(ack_cmd.toStdString().c_str(), "r");
            if (!pipe) {
                qDebug() << "[❌ ACK 실패] popen 호출 실패";
                return;
            }

            // ACK 메시지 읽기
            char buffer[256];
            bool received = false;
            while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                QString line = QString::fromStdString(std::string(buffer));
                if (!line.isEmpty()) {
                    received = true;
                    qDebug() << "📨 ACK 수신: " << line.trimmed();
                }
            }

            int status = pclose(pipe);

            // ACK 수신 여부 판단
            if (received || status == 0) {
                qDebug() << "════════════════════════════════════════";
                qDebug() << "✓ ACK 수신 성공";
                qDebug() << "주문 ID: " << order_id;
                qDebug() << "════════════════════════════════════════";

                emit order_acknowledged(order_id);
            }
            else {
                qDebug() << "════════════════════════════════════════";
                qDebug() << "⚠️  ACK 타임아웃";
                qDebug() << "10초 이내에 응답이 없습니다";
                qDebug() << "════════════════════════════════════════";
            }
        }
        catch (const std::exception& e) {
            qCritical() << "✗ 주문 발행 실패:" << e.what();
            emit error_occurred(QString("주문 발행 실패: %1").arg(e.what()));
        }
        });

    publish_thread.detach();
}