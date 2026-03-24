#include "numpaddialog.h"

NumPadDialog::NumPadDialog(QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle("PIN 입력");
    setFixedSize(360, 480);
    setModal(true);

    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(20, 20, 20, 20);
    mainLayout->setSpacing(12);

    QLabel *lbl = new QLabel("PIN 4자리 입력");
    lbl->setAlignment(Qt::AlignCenter);
    lbl->setObjectName("padTitle");
    mainLayout->addWidget(lbl);

    le_input = new QLineEdit();
    le_input->setReadOnly(true);
    le_input->setEchoMode(QLineEdit::Password);
    le_input->setMaxLength(4);
    le_input->setAlignment(Qt::AlignCenter);
    le_input->setObjectName("padInput");
    le_input->setFixedHeight(60);
    mainLayout->addWidget(le_input);

    QGridLayout *grid = new QGridLayout();
    grid->setSpacing(10);

    const QStringList keys = {"1","2","3","4","5","6","7","8","9","←","0","✓"};
    for (int i = 0; i < keys.size(); i++) {
        QPushButton *btn = new QPushButton(keys[i]);
        btn->setFixedSize(90, 70);

        if (keys[i] == "✓") {
            btn->setObjectName("confirmBtn");
            connect(btn, &QPushButton::clicked, this, [this](){
                if (le_input->text().length() == 4)
                    accept();
            });
        } else if (keys[i] == "←") {
            btn->setObjectName("deleteBtn");
            connect(btn, &QPushButton::clicked, this, [this](){
                QString t = le_input->text();
                t.chop(1);
                le_input->setText(t);
            });
        } else {
            btn->setObjectName("numBtn");
            QString key = keys[i];
            connect(btn, &QPushButton::clicked, this, [this, key](){
                if (le_input->text().length() < 4)
                    le_input->setText(le_input->text() + key);
            });
        }
        grid->addWidget(btn, i / 3, i % 3);
    }
    mainLayout->addLayout(grid);

    applyStyles();
}

void NumPadDialog::applyStyles()
{
    setStyleSheet(R"(
        QDialog {
            background-color: #FFFFFF;
        }
        #padTitle {
            font-size: 18px;
            font-weight: bold;
            color: #3D6B3D;
        }
        #padInput {
            font-size: 28px;
            border: 2px solid #5C8A5A;
            border-radius: 10px;
            padding: 8px;
        }
        #numBtn {
            font-size: 22px;
            font-weight: bold;
            color: #2D2D2D;
            background-color: #F7FAF7;
            border: 2px solid #C8DFC8;
            border-radius: 10px;
        }
        #numBtn:pressed {
            background-color: #C8DFC8;
        }
        #deleteBtn {
            font-size: 22px;
            background-color: #FFF0F0;
            border: 2px solid #FFCCCC;
            border-radius: 10px;
        }
        #deleteBtn:pressed {
            background-color: #FFCCCC;
        }
        #confirmBtn {
            font-size: 22px;
            font-weight: bold;
            color: #FFFFFF;
            background-color: #5C8A5A;
            border: none;
            border-radius: 10px;
        }
        #confirmBtn:pressed {
            background-color: #3D6B3D;
        }
    )");
}
