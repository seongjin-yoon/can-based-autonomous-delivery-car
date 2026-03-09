#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QCheckBox>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFrame>
#include <QStyle>
#include <memory>
#include "numpaddialog.h"
#include "monitorwindow.h"
#include "mqtt_client.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // UI 이벤트
    void onOrderClicked();
    void updateTotal();
    
    // MQTT 신호 처리
    void onMqttConnectionChanged(bool connected);
    void onMqttError(const QString& error_message);
    void onOrderAcknowledged(const QString& order_id);

private:
    // UI 초기화
    void initUI();
    void applyStyles();
    
    // UI 컴포넌트 - 메뉴
    QCheckBox *cb_bulgogi;
    QCheckBox *cb_jeyuk;
    QCheckBox *cb_haejangguk;
    QCheckBox *cb_drink;

    // UI 컴포넌트 - 목적지
    QPushButton *btn_dest[4];
    int selected_dest = -1;

    // UI 컴포넌트 - PIN
    QLineEdit *le_pin;
    
    // UI 컴포넌트 - 버튼
    QPushButton *btn_order;
    
    // UI 컴포넌트 - 정보
    QLabel *lbl_total;
    QLabel *lbl_status;
    
    // MQTT 클라이언트
    std::unique_ptr<MQTTClient> m_mqtt_client;
    
    // 주문 관리
    int m_order_counter = 1;
    QString m_last_order_id;
    MonitorWindow *m_monitor_window = nullptr;
};

#endif