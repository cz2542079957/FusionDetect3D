#ifndef CARCONTROLLER_H
#define CARCONTROLLER_H
#include "common.h"

// 运动状态
typedef enum
{
    MOTION_STOP,
    MOTION_FORWARD,
    MOTION_BACKWARD,
    MOTION_LEFT,
    MOTION_RIGHT,
    MOTION_FRONT_RIGHT,
    MOTION_FRONT_LEFT,
    MOTION_BACK_RIGHT,
    MOTION_BACK_LEFT,
    MOTION_TURN_LEFT,
    MOTION_TURN_RIGHT,
    MOTION_TURN_FRONT_RIGHT,
    MOTION_TURN_FRONT_LEFT,
    MOTION_TURN_BACK_RIGHT,
    MOTION_TURN_BACK_LEFT,
} MotionState;

class CarController: public QObject
{
    Q_OBJECT
public:
    CarController();
    ~CarController();

    int getMode();
    void setMode(int _mode);

    void keypressActionHandler(QKeyEvent *event);
    void keyreleaseActionHandler(QKeyEvent *event);

    //清空按键
    void clearKeys();

private:
    //模式： 0静止     1扫描模式    2运动模式
    int mode = 0;
    int speed = 30;

    //定时处理
    QTimer timer;
    //响应帧间隔(ms)
    int responseInterval =  10;
    //键盘按下按键列表
    QList<int> keys;

signals:
    void sendControlSignal(int state, int speed);

public slots:
    void handler();

    void carSpeedSlot(int value);

};

#endif // CARCONTROLLER_H
