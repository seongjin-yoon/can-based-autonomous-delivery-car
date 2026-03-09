#ifndef MONITORWINDOW_H
#define MONITORWINDOW_H

#include <QWidget>
#include <QLabel>
#include <QProgressBar>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFrame>
#include <QPainter>
#include <QTimer>
#include <QPushButton>

enum class DeliveryState {
    IDLE,
    DRIVING,
    ARRIVED,
    WAITING_AUTH,
    AUTH_SUCCESS,
    RETURNING,
    DONE
};

class RouteWidget : public QWidget
{
    Q_OBJECT
public:
    RouteWidget(QWidget *parent = nullptr);
    void setProgress(float progress);

protected:
    void paintEvent(QPaintEvent *) override;

private:
    float _progress = 0.0f;
};

class MonitorWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MonitorWindow(const QString &menus, int dest, QWidget *parent = nullptr);

public slots:
    void updateState(DeliveryState state, float progress);

private:
    RouteWidget  *routeWidget;
    QProgressBar *progressBar;
    QLabel       *lbl_state;
    QLabel       *lbl_order;
    QLabel       *lbl_dest;
    QLabel *lbl_countdown;
    QTimer *countdownTimer;
    QPushButton *btn_collect;

    int countdown_sec = 30;

    void startCountdown();
    void applyStyles();
    QString stateToString(DeliveryState state);
};

#endif
