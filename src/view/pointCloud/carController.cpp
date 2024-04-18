#include "carController.h"


CarController::CarController()
{
    connect(&timer, SIGNAL(timeout()), this, SLOT(handler()));
}

CarController::~CarController()
{

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
    int state = 0, speed = 20;
    if (keys.size() >= 2)
    {
        return;
    }
    if (keys.size() == 0)
    {
        state = 0;
    }

    // if (!(keys.contains(Qt::Key_W) && keys.contains(Qt::Key_S)))
    // {
    //     qDebug() << keys;
    // }
    if (keys.contains(Qt::Key_W))
    {
        state = 1;
    }
    else if (keys.contains(Qt::Key_S))
    {
        state = 2;
    }
    else if (keys.contains(Qt::Key_A))
    {
        state = 3;
    }
    else if (keys.contains(Qt::Key_D))
    {
        state = 4;
    }
    // qDebug() << state ;
    emit sendControlSignal(state, speed);
}
