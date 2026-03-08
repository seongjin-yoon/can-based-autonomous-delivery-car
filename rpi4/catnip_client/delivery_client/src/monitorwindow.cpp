#include "monitorwindow.h"

// ── RouteWidget ──
RouteWidget::RouteWidget(QWidget *parent) : QWidget(parent)
{
    setFixedHeight(120);
}

void RouteWidget::setProgress(float progress)
{
    _progress = qBound(0.0f, progress, 1.0f);
    update();
}

void RouteWidget::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    int margin = 40;
    int cy = height() / 2;
    int x1 = margin;
    int x2 = width() - margin;

    // 배경 선
    p.setPen(QPen(QColor("#C8DFC8"), 4));
    p.drawLine(x1, cy, x2, cy);

    // 진행 선
    int xProgress = x1 + (int)((x2 - x1) * _progress);
    p.setPen(QPen(QColor("#5C8A5A"), 4));
    p.drawLine(x1, cy, xProgress, cy);

    // 출발지 원
    p.setBrush(QColor("#5C8A5A"));
    p.setPen(Qt::NoPen);
    p.drawEllipse(QPoint(x1, cy), 10, 10);

    // 목적지 원
    p.setBrush(QColor("#F7FAF7"));
    p.setPen(QPen(QColor("#5C8A5A"), 2));
    p.drawEllipse(QPoint(x2, cy), 10, 10);

    // 배달카 점
    p.setBrush(QColor("#3D6B3D"));
    p.setPen(Qt::NoPen);
    p.drawEllipse(QPoint(xProgress, cy), 14, 14);

    // 라벨
    p.setPen(QColor("#5C8A5A"));
    p.setFont(QFont("sans-serif", 11, QFont::Bold));
    p.drawText(x1 - 15, cy + 30, "출발");
    p.drawText(x2 - 15, cy + 30, "도착");
}

// ── MonitorWindow ──
MonitorWindow::MonitorWindow(const QString &menus, int dest, QWidget *parent)
    : QWidget(parent)
{
    setFixedSize(480, 800);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(24, 30, 24, 30);
    layout->setSpacing(20);

    // 타이틀
    QLabel *lbl_title = new QLabel("배달 현황");
    lbl_title->setAlignment(Qt::AlignCenter);
    lbl_title->setObjectName("monitorTitle");
    layout->addWidget(lbl_title);

    // 주문 요약
    lbl_order = new QLabel("주문: " + menus);
    lbl_order->setObjectName("orderInfo");
    lbl_order->setWordWrap(true);
    layout->addWidget(lbl_order);

    lbl_dest = new QLabel(QString("목적지: %1번").arg(dest));
    lbl_dest->setObjectName("orderInfo");
    layout->addWidget(lbl_dest);

    QFrame *line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setObjectName("divider");
    layout->addWidget(line);

    // 상태 텍스트
    lbl_state = new QLabel("주행 중...");
    lbl_state->setAlignment(Qt::AlignCenter);
    lbl_state->setObjectName("stateLabel");
    layout->addWidget(lbl_state);

    // 루트 위젯
    routeWidget = new RouteWidget();
    layout->addWidget(routeWidget);

    // 진행 바
    progressBar = new QProgressBar();
    progressBar->setRange(0, 100);
    progressBar->setValue(0);
    progressBar->setTextVisible(false);
    progressBar->setFixedHeight(20);
    progressBar->setObjectName("deliveryProgress");
    layout->addWidget(progressBar);

    // 카운트다운 라벨 (도착 시에만 표시)
    lbl_countdown = new QLabel("");
    lbl_countdown->setAlignment(Qt::AlignCenter);
    lbl_countdown->setObjectName("countdownLabel");
    lbl_countdown->setVisible(false);
    layout->addWidget(lbl_countdown);

    countdownTimer = new QTimer(this);
    connect(countdownTimer, &QTimer::timeout, this, [this](){
        countdown_sec--;
        lbl_countdown->setText(QString("수령까지 남은 시간: %1초").arg(countdown_sec));
        if (countdown_sec <= 0) {
            countdownTimer->stop();
            updateState(DeliveryState::RETURNING, 1.0f);
        }
    });

    // 수령 완료 버튼
    btn_collect = new QPushButton("수령 완료 (테스트)");
    btn_collect->setObjectName("collectBtn");
    btn_collect->setFixedHeight(55);
    btn_collect->setVisible(false);
    connect(btn_collect, &QPushButton::clicked, this, [this](){
        updateState(DeliveryState::AUTH_SUCCESS, 1.0f);
    });
    layout->addWidget(btn_collect);

    layout->addStretch();

    applyStyles();
}

void MonitorWindow::updateState(DeliveryState state, float progress)
{
    lbl_state->setText(stateToString(state));
    routeWidget->setProgress(progress);
    progressBar->setValue((int)(progress * 100));

    if (state == DeliveryState::ARRIVED) {
        startCountdown();
        btn_collect->setVisible(true);
    } else if (state == DeliveryState::AUTH_SUCCESS) {
        countdownTimer->stop();
        lbl_countdown->setVisible(false);
        btn_collect->setVisible(true);
    } else if (state == DeliveryState::RETURNING || state == DeliveryState::DONE) {
        countdownTimer->stop();
        lbl_countdown->setVisible(false);
        btn_collect->setVisible(true);
    }
}

QString MonitorWindow::stateToString(DeliveryState state)
{
    switch (state) {
        case DeliveryState::DRIVING:      return "주행 중...";
        case DeliveryState::ARRIVED:      return "도착! PIN을 입력하여 수령하세요";
        case DeliveryState::WAITING_AUTH: return "PIN 인증 대기 중";
        case DeliveryState::AUTH_SUCCESS: return "배달이 완료되었습니다!";
        case DeliveryState::RETURNING:    return "미수령으로 복귀합니다";
        case DeliveryState::DONE:         return "배달 완료!";
        default:                          return "대기 중";
    }
}

void MonitorWindow::startCountdown()
{
    countdown_sec = 30;
    lbl_countdown->setVisible(true);
    lbl_countdown->setText("수령까지 남은 시간: 30초");
    countdownTimer->start(1000);
}

void MonitorWindow::applyStyles()
{
    setStyleSheet(R"(
        QWidget {
            background-color: #FFFFFF;
            font-family: 'Noto Sans KR', sans-serif;
        }
        #monitorTitle {
            font-size: 26px;
            font-weight: bold;
            color: #3D6B3D;
            padding: 10px 0;
        }
        #orderInfo {
            font-size: 15px;
            color: #555555;
        }
        #divider {
            color: #C8DFC8;
            background-color: #C8DFC8;
            max-height: 1px;
        }
        #stateLabel {
            font-size: 22px;
            font-weight: bold;
            color: #3D6B3D;
            padding: 10px;
        }
        #countdownLabel {
            font-size: 18px;
            color: #E05555;
            font-weight: bold;
            padding: 6px;
        }
        QProgressBar#deliveryProgress {
            border: 2px solid #C8DFC8;
            border-radius: 10px;
            background-color: #F7FAF7;
        }
        QProgressBar#deliveryProgress::chunk {
            background-color: #5C8A5A;
            border-radius: 8px;
        }
        #collectBtn {
            font-size: 18px;
            font-weight: bold;
            color: #FFFFFF;
            background-color: #3D6B3D;
            border: none;
            border-radius: 10px;
        }
        #collectBtn:pressed {
            background-color: #2A4F2A;
        }
    )");
}
