#include "cameraController.h"

using namespace NSPointCloud;

CameraController::CameraController()
{
    baseVector = (baseDirection - basePos).normalized();
    cameraRight = QVector3D::crossProduct(baseUp, baseVector).normalized();
    cameraUp = QVector3D::crossProduct(baseVector, cameraRight).normalized();

    connect(&timer, SIGNAL(timeout()), this, SLOT(handler()));
    connect(&animationTimer, SIGNAL(timeout()), this, SLOT(animationHandler()));
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



float CameraController::getFov() const
{
    return fov;
}

float CameraController::getBaseSpeed() const
{
    return baseSpeed;
}

void CameraController::setBaseSpeed(float newBaseSpeed)
{
    baseSpeed = newBaseSpeed;
}

float CameraController::getMoveSpeed() const
{
    return moveSpeed * baseSpeed;
}

void CameraController::setMoveSpeed(float newMoveSpeed)
{
    moveSpeed = newMoveSpeed;
}

float CameraController::getRollSpeed() const
{
    return rollSpeed * baseSpeed;
}

void CameraController::setRollSpeed(float newRollSpeed)
{
    rollSpeed = newRollSpeed;
}

float CameraController::getWheelRate() const
{
    return wheelRate * baseSpeed;
}

void CameraController::setWheelRate(float newWheelRate)
{
    wheelRate = newWheelRate;
}

float CameraController::getRotateSpeed() const
{
    return rotateSpeed * baseSpeed;
}

void CameraController::setRotateSpeed(float newRotateSpeed)
{
    rotateSpeed = newRotateSpeed;
}


void CameraController::handler()
{
    if (animationStart)
    {
        return ;
    }
    // qDebug() << keys;
    //加速 减速 效果
    bool speedUp = false, speedDown = false;
    float speedMagnification = 1.0;
    if (keys.contains(Qt::Key_Shift))
    {
        speedUp = true;
        speedMagnification = moveSpeedUpMagnification;
    }
    if (keys.contains(Qt::Key_Control))
    {
        speedDown = true;
        speedMagnification = moveSpeedDownMagnification;
    }
    if (speedUp && speedDown)
    {
        //同时按下加速 减速则无效
        speedMagnification = 1;
    }
    foreach (auto k, keys)
    {
        switch (k)
        {
            //前
            case Qt::Key_W :
                basePosAdd(getMoveSpeed() * baseVector * speedMagnification);
                break;
            //后
            case Qt::Key_S:
                basePosAdd(-getMoveSpeed() * baseVector * speedMagnification);
                break;
            //左
            case Qt::Key_A:
                basePosAdd(getMoveSpeed() * cameraRight * speedMagnification);
                break;
            //右
            case Qt::Key_D:
                basePosAdd(-getMoveSpeed() * cameraRight * speedMagnification);
                break;
            //向右滚筒
            case Qt::Key_E:
                cameraUp = rotateAboutAxis(cameraUp,  baseVector, M_PI * getRollSpeed());
                cameraRight = rotateAboutAxis(cameraRight,  baseVector, M_PI * getRollSpeed());
                break;
            //向左滚筒
            case Qt::Key_Q:
                cameraUp = rotateAboutAxis(cameraUp,  baseVector, -M_PI * getRollSpeed());
                cameraRight = rotateAboutAxis(cameraRight,  baseVector, -M_PI * getRollSpeed());
                break;
            //上升
            case Qt::Key_X:
                basePosAdd(getMoveSpeed() * cameraUp);
                break;
            //下降
            case Qt::Key_Z:
                basePosAdd(-getMoveSpeed() * cameraUp);
                break;
            //重置视角
            case Qt::Key_R:
                resetAnimation();
                break;
            //功能键
            case Qt::Key_V:
                break;
        }
    }
    emit updateGraph();
}

void CameraController::animationHandler()
{
    if (!animationStart)
    {
        return ;
    }
    double posDeltaLengthSquared = (targetPos - basePos).lengthSquared();
    double vectorDeltaLengthSquared = (targetVector - baseVector).lengthSquared();
    double cameraRightDeltaLengthSquared = (targetCameraRight - cameraRight).lengthSquared();
    if (posDeltaLengthSquared < animationAccuracy && vectorDeltaLengthSquared < animationAccuracy && cameraRightDeltaLengthSquared < animationAccuracy)
    {
        animationStart = false;
        animationTimer.stop();
        return;
    }
    // qDebug() << posDeltaLengthSquared << " "  <<  vectorDeltaLengthSquared;
    if (posDeltaLengthSquared > animationAccuracy)
    {
        basePos += deltaPos / animationStep;
    }
    if (vectorDeltaLengthSquared > animationAccuracy)
    {
        baseVector = (baseVector + deltaVector / animationStep).normalized();
    }
    if (cameraRightDeltaLengthSquared > animationAccuracy)
    {
        cameraRight += deltaCameraRight / animationStep;
        cameraUp = QVector3D::crossProduct(baseVector, cameraRight).normalized();
    }
    emit updateGraph();
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

void CameraController::wheelActionHandler(QWheelEvent *event)
{
    QPoint p = event->angleDelta();
    // int x =  p.x();
    int y = p.y();
    fov -=  y * getWheelRate();
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

void CameraController::mousepressActionHandler(QMouseEvent *event)
{
    if (animationStart)
    {
        return ;
    }
    lastMousePos = event->pos();
}

void CameraController::mousereleaseActionHandler(QMouseEvent *event)
{
    if (animationStart)
    {
        return ;
    }
    lastMousePos = event->pos();
}

void CameraController::mousemoveActionHandler(QMouseEvent *event)
{
    if (animationStart)
    {
        return ;
    }
    currentMousePos = event->pos();
    if (pointsDistance(currentMousePos, lastMousePos)  <= 2)
    {
        //优化剪枝
        return;
    }

    QQuaternion horizontalRotation = QQuaternion::fromAxisAndAngle(cameraRight, (lastMousePos.y()  - currentMousePos.y()) * getRotateSpeed());
    QQuaternion verticalRotation = QQuaternion::fromAxisAndAngle(cameraUp, (currentMousePos.x() - lastMousePos.x()) * getRotateSpeed());
    QQuaternion cumulativeRotation =  (horizontalRotation * verticalRotation).normalized();

    baseVector = cumulativeRotation.rotatedVector(baseVector);
    cameraUp = cumulativeRotation.rotatedVector(cameraUp);
    cameraRight = QVector3D::crossProduct(cameraUp, baseVector).normalized();

    // if (rightDelta != 0)
    // {
    //     baseVector = rotateAboutAxis(baseVector, cameraUp, rightDelta *  getRotateSpeed());
    //     cameraRight = QVector3D::crossProduct(cameraUp, baseVector).normalized();
    // }
    // if (upDelta != 0)
    // {
    //     baseVector = rotateAboutAxis(baseVector,  cameraRight, upDelta *  getRotateSpeed());
    //     cameraUp = QVector3D::crossProduct(baseVector, cameraRight).normalized();
    // }
    // qDebug() << cameraUp;
    // qDebug() << lastMousePos << " " << currentMousePos;
    // qDebug() << baseVector << " " <<  cameraUp << " " << cameraRight;
    lastMousePos = currentMousePos;
    emit updateGraph();
}


void CameraController::resetAnimation()
{
    if (animationStart)
    {
        return;
    }
    oldPos = basePos;
    targetPos = QVector3D(3, 3, 3);
    deltaPos =  targetPos - oldPos;

    targetVector = (QVector3D(0, 0, 0) - targetPos).normalized();
    targetCameraRight = QVector3D::crossProduct(baseUp, targetVector).normalized();
    targetCameraUp = QVector3D::crossProduct(targetVector, targetCameraRight).normalized();
    deltaVector = targetVector -  baseVector;
    // deltaUp = targetCameraUp - baseUp;
    deltaCameraRight = targetCameraRight - cameraRight;

    // 如果变化过小则不改变
    if (deltaPos.lengthSquared() <  animationAccuracy && deltaVector.lengthSquared() < animationAccuracy && deltaCameraRight.lengthSquared() < animationAccuracy)
    {
        return;
    }
    animationStart = true;
    animationTimer.start(animationInterval);
}



