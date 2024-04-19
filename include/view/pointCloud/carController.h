#ifndef CARCONTROLLER_H
#define CARCONTROLLER_H
#include "pointCloudBase.h"

class CarController: public QObject
{
    Q_OBJECT
public:
    CarController();
    ~CarController();

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
