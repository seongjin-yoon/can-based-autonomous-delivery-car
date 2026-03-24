#include "mainwindow.h"
#include <QMessageBox>
#include <QStyle>
#include <QLocale>
#include <QFrame>
#include <QProcess>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setFixedSize(480, 800);
    setWindowTitle("락감마당");

    QWidget *central = new QWidget(this);
    setCentralWidget(central);

    QVBoxLayout *mainLayout = new QVBoxLayout(central);
    mainLayout->setContentsMargins(20, 20, 20, 20);
    mainLayout->setSpacing(16);

    // 가게 이름
    QLabel *lbl_shop = new QLabel("락감마당");
    lbl_shop->setAlignment(Qt::AlignCenter);
    lbl_shop->setObjectName("shopName");
    mainLayout->addWidget(lbl_shop);

    QFrame *line1 = new QFrame();
    line1->setFrameShape(QFrame::HLine);
    line1->setObjectName("divider");
    mainLayout->addWidget(line1);

    // 메뉴 선택
    QLabel *lbl_menu = new QLabel("메뉴 선택");
    lbl_menu->setObjectName("sectionLabel");
    mainLayout->addWidget(lbl_menu);

    auto makeMenuRow = [](QCheckBox *cb, const QString &name, const QString &price) -> QWidget* {
        QWidget *row = new QWidget();
        QHBoxLayout *hl = new QHBoxLayout(row);
        hl->setContentsMargins(10, 8, 10, 8);
        cb->setText(name);
        cb->setObjectName("menuCheck");
        QLabel *lbl_price = new QLabel(price);
        lbl_price->setObjectName("priceLabel");
        lbl_price->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        hl->addWidget(cb);
        hl->addWidget(lbl_price);
        row->setObjectName("menuRow");
        return row;
    };

    cb_bulgogi    = new QCheckBox(); mainLayout->addWidget(makeMenuRow(cb_bulgogi,    "불고기백반", "₩10,000"));
    cb_jeyuk      = new QCheckBox(); mainLayout->addWidget(makeMenuRow(cb_jeyuk,      "제육볶음",   "₩9,000"));
    cb_haejangguk = new QCheckBox(); mainLayout->addWidget(makeMenuRow(cb_haejangguk, "뼈해장국",   "₩9,000"));
    cb_drink      = new QCheckBox(); mainLayout->addWidget(makeMenuRow(cb_drink,      "음료수",     "₩2,000"));

    connect(cb_bulgogi,    &QCheckBox::checkStateChanged, this, [this](Qt::CheckState){ updateTotal(); });
    connect(cb_jeyuk,      &QCheckBox::checkStateChanged, this, [this](Qt::CheckState){ updateTotal(); });
    connect(cb_haejangguk, &QCheckBox::checkStateChanged, this, [this](Qt::CheckState){ updateTotal(); });
    connect(cb_drink,      &QCheckBox::checkStateChanged, this, [this](Qt::CheckState){ updateTotal(); });

    QFrame *line2 = new QFrame();
    line2->setFrameShape(QFrame::HLine);
    line2->setObjectName("divider");
    mainLayout->addWidget(line2);

    // 목적지
    QLabel *lbl_dest = new QLabel("목적지 선택");
    lbl_dest->setObjectName("sectionLabel");
    mainLayout->addWidget(lbl_dest);

    QHBoxLayout *destLayout = new QHBoxLayout();
    destLayout->setSpacing(10);
    for (int i = 0; i < 4; i++) {
        btn_dest[i] = new QPushButton(QString::number(i + 1));
        btn_dest[i]->setObjectName("destBtn");
        btn_dest[i]->setFixedSize(90, 60);
        connect(btn_dest[i], &QPushButton::clicked, this, [this, i]() {
            selected_dest = i + 1;
            for (int j = 0; j < 4; j++) {
                btn_dest[j]->setProperty("selected", j == i);
                btn_dest[j]->style()->unpolish(btn_dest[j]);
                btn_dest[j]->style()->polish(btn_dest[j]);
            }
        });
        destLayout->addWidget(btn_dest[i]);
    }
    mainLayout->addLayout(destLayout);

    QFrame *line3 = new QFrame();
    line3->setFrameShape(QFrame::HLine);
    line3->setObjectName("divider");
    mainLayout->addWidget(line3);

    // PIN
    QLabel *lbl_pin = new QLabel("PIN 설정 (4자리)");
    lbl_pin->setObjectName("sectionLabel");
    mainLayout->addWidget(lbl_pin);

    le_pin = new QLineEdit();
    le_pin->setPlaceholderText("터치하여 PIN 입력");
    le_pin->setMaxLength(4);
    le_pin->setEchoMode(QLineEdit::Password);
    le_pin->setObjectName("pinInput");
    le_pin->setFixedHeight(55);
    le_pin->setReadOnly(true);
    mainLayout->addWidget(le_pin);

    connect(le_pin, &QLineEdit::selectionChanged, this, [this](){
        NumPadDialog dlg(this);
        if (dlg.exec() == QDialog::Accepted) {
            le_pin->setText(dlg.getPin());
        }
    });

    // 총 금액
    lbl_total = new QLabel("총 금액: ₩0");
    lbl_total->setAlignment(Qt::AlignRight);
    lbl_total->setObjectName("totalLabel");
    mainLayout->addWidget(lbl_total);

    mainLayout->addStretch();

    // 주문 버튼
    btn_order = new QPushButton("주문하기");
    btn_order->setObjectName("orderBtn");
    btn_order->setFixedHeight(65);
    connect(btn_order, &QPushButton::clicked, this, &MainWindow::onOrderClicked);
    mainLayout->addWidget(btn_order);

    applyStyles();
}

void MainWindow::updateTotal()
{
    int total = 0;
    if (cb_bulgogi->isChecked())    total += 10000;
    if (cb_jeyuk->isChecked())      total += 9000;
    if (cb_haejangguk->isChecked()) total += 9000;
    if (cb_drink->isChecked())      total += 2000;

    lbl_total->setText(QString("총 금액: ₩%1").arg(
        QLocale(QLocale::Korean).toString(total)));
}

void MainWindow::onOrderClicked()
{
    if (!cb_bulgogi->isChecked() && !cb_jeyuk->isChecked() &&
        !cb_haejangguk->isChecked() && !cb_drink->isChecked()) {
        QMessageBox::warning(this, "알림", "메뉴를 선택해주세요.");
        return;
    }
    if (selected_dest == -1) {
        QMessageBox::warning(this, "알림", "목적지를 선택해주세요.");
        return;
    }
    if (le_pin->text().length() != 4) {
        QMessageBox::warning(this, "알림", "PIN 4자리를 입력해주세요.");
        return;
    }

    QStringList menus;
    if (cb_bulgogi->isChecked())    menus << "불고기백반";
    if (cb_jeyuk->isChecked())      menus << "제육볶음";
    if (cb_haejangguk->isChecked()) menus << "뼈해장국";
    if (cb_drink->isChecked())      menus << "음료수";

    // TODO: MQTT 전송 (팀원이 채울 부분)
    // mqttClient->publish("delivery/order/001", orderJson);
    // mqttClient->publish("delivery/pin/001/4to3", pinJson);

    MonitorWindow *monitor = new MonitorWindow(menus.join(", "), selected_dest);
    monitor->setWindowFlags(Qt::Window);
    monitor->show();
    this->hide();
}

void MainWindow::applyStyles()
{
    setStyleSheet(R"(
        QWidget {
            background-color: #FFFFFF;
            font-family: 'Noto Sans KR', sans-serif;
        }
        #shopName {
            font-size: 28px;
            font-weight: bold;
            color: #3D6B3D;
            padding: 10px 0;
            letter-spacing: 2px;
        }
        #sectionLabel {
            font-size: 15px;
            font-weight: bold;
            color: #5C8A5A;
            padding: 4px 0;
        }
        #divider {
            color: #C8DFC8;
            background-color: #C8DFC8;
            max-height: 1px;
        }
        #menuRow {
            background-color: #F7FAF7;
            border: 1px solid #D0E8D0;
            border-radius: 10px;
        }
        #menuCheck {
            font-size: 16px;
            color: #2D2D2D;
            spacing: 10px;
        }
        #menuCheck::indicator {
            width: 24px;
            height: 24px;
            border: 2px solid #5C8A5A;
            border-radius: 5px;
        }
        #menuCheck::indicator:checked {
            background-color: #5C8A5A;
        }
        #priceLabel {
            font-size: 15px;
            color: #5C8A5A;
            font-weight: bold;
        }
        #destBtn {
            font-size: 20px;
            font-weight: bold;
            color: #5C8A5A;
            background-color: #F7FAF7;
            border: 2px solid #5C8A5A;
            border-radius: 10px;
        }
        #destBtn[selected="true"] {
            background-color: #5C8A5A;
            color: #FFFFFF;
        }
        #pinInput {
            font-size: 18px;
            border: 2px solid #5C8A5A;
            border-radius: 10px;
            padding: 8px 14px;
            color: #2D2D2D;
        }
        #pinInput:focus {
            border-color: #3D6B3D;
        }
        #totalLabel {
            font-size: 17px;
            font-weight: bold;
            color: #3D6B3D;
            padding: 4px 0;
        }
        #orderBtn {
            font-size: 20px;
            font-weight: bold;
            color: #FFFFFF;
            background-color: #5C8A5A;
            border: none;
            border-radius: 14px;
        }
        #orderBtn:pressed {
            background-color: #3D6B3D;
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

MainWindow::~MainWindow() {}
