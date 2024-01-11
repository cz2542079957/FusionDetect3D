#include "cameracontroller.h"

CameraController::CameraController()
{
    baseVector = QVector3D(baseDirection - basePos);
    baseVector.normalize();
    cameraRight  =  QVector3D::crossProduct(baseUp, baseVector);
    cameraRight.normalize();
    cameraUp  = QVector3D::crossProduct(baseVector, cameraRight);
    cameraUp.normalize();

    connect(&timer, SIGNAL(timeout()), this, SLOT(handler()));
}

CameraController::~CameraController()
{
}

const QVector3D &CameraController::getBasePos() const
{
    return basePos;
}

void CameraController::setBasePos(const QVector3D &newBasePos)
{
    basePos = newBasePos;
}

void CameraController::basePosAdd(const QVector3D &delta)
{
    basePos += delta;
}

const QVector3D &CameraController::getBaseUp() const
{
    return baseUp;
}

void CameraController::setBaseUp(const QVector3D &newBaseUp)
{
    baseUp = newBaseUp;
}

const QVector3D &CameraController::getCameraRight() const
{
    return cameraRight;
}

void CameraController::setCameraRight(const QVector3D &newCameraRight)
{
    cameraRight = newCameraRight;
}

const QVector3D &CameraController::getCameraUp() const
{
    return cameraUp;
}

void CameraController::setCameraUp(const QVector3D &newCameraUp)
{
    cameraUp = newCameraUp;
}

const QVector3D &CameraController::getBaseDirection() const
{
    return baseDirection;
}

void CameraController::setBaseDirection(const QVector3D &newBaseDirection)
{
    baseDirection = newBaseDirection;
}

const QVector3D &CameraController::getBaseVector() const
{
    return baseVector;
}

void CameraController::setBaseVector(const QVector3D &newBaseVector)
{
    baseVector = newBaseVector;
}

float CameraController::getSpeed() const
{
    return moveSpeed;
}

void CameraController::setSpeed(float newSpeed)
{
    moveSpeed = newSpeed;
}

void CameraController::keypressActionHandler(QKeyEvent *event)
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

void CameraController::keyreleaseActionHandler(QKeyEvent *event)
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

void CameraController::handler()
{
    foreach (auto k, keys)
    {
        switch (k)
        {
            //前
            case Qt::Key_W:
                basePosAdd(moveSpeed * baseVector);
                break;
            //后
            case Qt::Key_S:
                basePosAdd(-moveSpeed * baseVector);
                break;
            //左
            case Qt::Key_A:
                basePosAdd(moveSpeed * cameraRight);
                break;
            //右
            case Qt::Key_D:
                basePosAdd(-moveSpeed * cameraRight);
                break;
            //向右滚筒
            case Qt::Key_E:
                cameraUp = rotateAboutAxis(cameraUp,  baseVector, M_PI * rollSpeed);
                cameraRight = rotateAboutAxis(cameraRight,  baseVector, M_PI * rollSpeed);
                break;
            //向左滚筒
            case Qt::Key_Q:
                cameraUp = rotateAboutAxis(cameraUp,  baseVector, -M_PI * rollSpeed);
                cameraRight = rotateAboutAxis(cameraRight,  baseVector, -M_PI * rollSpeed);
                break;
            //上升
            case Qt::Key_Shift:
                basePosAdd(moveSpeed * cameraUp);
                break;
            //下降
            case Qt::Key_Control:
                basePosAdd(-moveSpeed * cameraUp);
                break;
            //功能键
            case Qt::Key_V:
                break;
        }
    }
    emit updateGraph();

}

void CameraController::wheelActionHandler(QWheelEvent *event)
{
    QPoint p = event->angleDelta();
    int x =  p.x();
    int y = p.y();
    fov -=  y * wheelRate;
    if (fov < 20)
    {
        fov = 20;
    }
    else if (fov > 90)
    {
        fov = 90;
    }
    emit updateGraph();
}

float CameraController::getFov() const
{
    return fov;
}

void CameraController::mousepressActionHandler(QMouseEvent *event)
{
    lastMousePos = event->pos();
}

void CameraController::mousereleaseActionHandler(QMouseEvent *event)
{
    lastMousePos = event->pos();
}

void CameraController::mousemoveActionHandler(QMouseEvent *event)
{
    currentMousePos = event->pos();
    if (pointsDistance(currentMousePos, lastMousePos)  <= 2)
    {
        //优化剪枝
        return;
    }
    int rightDelta = currentMousePos.x() - lastMousePos.x();
    int upDelta = lastMousePos.y()  - currentMousePos.y();

    if (rightDelta != 0)
    {
        baseVector = rotateAboutAxis(baseVector, cameraUp, rightDelta *  rotateSpeed);
        cameraRight = QVector3D::crossProduct(cameraUp, baseVector);
        cameraRight.normalize();
    }
    if (upDelta != 0)
    {
        baseVector = rotateAboutAxis(baseVector,  cameraRight, upDelta *  rotateSpeed);
        cameraUp  = QVector3D::crossProduct(baseVector, cameraRight);
        cameraUp.normalize();
    }
//    qDebug() << lastMousePos << " " << currentMousePos;
//    qDebug() << baseVector << " " <<  cameraUp << " " << cameraRight;
    lastMousePos = currentMousePos;
    emit updateGraph();
}

