#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QCheckBox>
#include <QButtonGroup>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFrame>
#include <QTimer>
#include "numpaddialog.h"
#include "monitorwindow.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onOrderClicked();
    void updateTotal();

private:
    QCheckBox *cb_bulgogi;
    QCheckBox *cb_jeyuk;
    QCheckBox *cb_haejangguk;
    QCheckBox *cb_drink;

    QPushButton *btn_dest[4];
    int selected_dest = -1;

    QLineEdit *le_pin;
    QPushButton *btn_order;
    QLabel *lbl_total;

    void applyStyles();
};

#endif
