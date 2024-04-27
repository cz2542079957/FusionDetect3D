#include "carController.h"


CarController::CarController()
{
    connect(&timer, SIGNAL(timeout()), this, SLOT(handler()));
}

CarController::~CarController()
{

}

int CarController::getMode()
{
    return mode;
}

void CarController::setMode(int _mode)
{
    mode = _mode;
}

void CarController::keypressActionHandler(QKeyEvent *event)
{
    if (!event->isAutoRepeat()) //判断如果不是长按时自动触发的按下,就将key值加入容器
    {
        keys.append(event->key());
    }
    if (!timer.isActive()) //如果定时器不在运行，就启动一下
    {
        timer.start(responseInterval);
    }
}

void CarController::keyreleaseActionHandler(QKeyEvent *event)
{
    if (!event->isAutoRepeat()) //判断如果不是长按时自动触发的释放,就将key值从容器中删除
    {
        keys.removeAll(event->key());
    }
    if (keys.isEmpty()) //容器空了，关闭定时器
    {
        timer.stop();
    }
}

void CarController::clearKeys()
{
    keys.clear();
    timer.stop();
}

void CarController::handler()
{
    int state = MOTION_STOP;
    //只允许单键
    if (keys.size() >= 2)
    {
        return;
    }
    if (keys.size() == 0)
    {
        state = MOTION_STOP;
        emit sendControlSignal(state, speed);
        return;
    }

    int key = keys.first();
    //扫描模式
    if (mode == 1)
    {
        switch (key)
        {
            case Qt::Key_W:
                state = MOTION_FORWARD;
                break;
            case Qt::Key_S:
                state = MOTION_BACKWARD;
                break;
            case Qt::Key_A:
                state = MOTION_TURN_LEFT;
                break;
            case Qt::Key_D:
                state = MOTION_TURN_RIGHT;
                break;
        }
    }
    else if (mode == 2)
    {
        //运动模式
        switch (key)
        {
            case Qt::Key_W:
                state = MOTION_FORWARD;
                break;
            case Qt::Key_S:
                state = MOTION_BACKWARD;
                break;
            case Qt::Key_A:
                state = MOTION_TURN_LEFT;
                break;
            case Qt::Key_D:
                state = MOTION_TURN_RIGHT;
                break;
            case Qt::Key_Q:
                state = MOTION_LEFT;
                break;
            case Qt::Key_E:
                state = MOTION_RIGHT;
                break;
            case Qt::Key_7:
                state = MOTION_FRONT_LEFT;
                break;
            case Qt::Key_9:
                state = MOTION_FRONT_RIGHT;
                break;
            case Qt::Key_1:
                state = MOTION_BACK_LEFT;
                break;
            case Qt::Key_3:
                state = MOTION_BACK_RIGHT;
                break;
        }
    }
    // qDebug() << state ;
    emit sendControlSignal(state, speed);
}

void CarController::carSpeedSlot(int value)
{
    speed = value;
}
