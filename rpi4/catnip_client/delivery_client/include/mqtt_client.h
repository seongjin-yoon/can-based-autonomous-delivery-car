#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <QString>
#include <QObject>
#include <QJsonObject>
#include <memory>
#include <mqtt/client.h>

/**
 * @class MQTTClient
 * @brief MQTT 브로커와의 비동기 통신을 관리하는 클래스 (Qt6 기반)
 *
 * Paho C++ MQTT 라이브러리를 사용하여 주문 발행 및 ACK 수신
 */
class MQTTClient : public QObject {
    Q_OBJECT

public:
    explicit MQTTClient(const QString& broker_address = "10.42.0.1:1883",
        const QString& username = "hoji",
        const QString& password = "1234",
        QObject* parent = nullptr);
    ~MQTTClient();

    void connect();
    void disconnect();
    [[nodiscard]] bool is_connected() const;

    void publish_order(const QString& order_id,
        const QStringList& menus,
        int destination,
        const QString& pin);

    void subscribe_topics();

signals:
    void connection_changed(bool connected);
    void order_acknowledged(const QString& order_id);
    void error_occurred(const QString& error_message);
    void shop_loaded(const QJsonObject& shop_data);

private:
    class MQTTCallback : public virtual mqtt::callback {
    public:
        explicit MQTTCallback(MQTTClient* parent);
        void connected(const std::string& cause) override;
        void connection_lost(const std::string& cause) override;
        void delivery_complete(mqtt::delivery_token_ptr token) override;

    private:
        MQTTClient* m_parent;
    };

    QString m_broker_address;
    QString m_username;
    QString m_password;

    std::unique_ptr<mqtt::client> m_mqtt_client;
    std::unique_ptr<MQTTCallback> m_callback;
    bool m_connected = false;
};

#endif // MQTT_CLIENT_H
