#include "cameraController.h"

using namespace NSPointCloud;

CameraController::CameraController()
{
    //初始方向设置为中心点
    baseDirection = baseCenter;
    //计算相机方位
    baseVector = (baseDirection - basePos).normalized();
    cameraRight = QVector3D::crossProduct(baseUp, baseVector).normalized();
    cameraUp = QVector3D::crossProduct(baseVector, cameraRight).normalized();

    connect(&timer, SIGNAL(timeout()), this, SLOT(handler()));
    connect(&animationTimer, SIGNAL(timeout()), this, SLOT(animationHandler()));
}

CameraController::~CameraController()
{
}


void CameraController::basePosAdd(const QVector3D &delta)
{
    basePos += delta;
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

    lastMousePos = currentMousePos;
    emit updateGraph();
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
    foreach (auto key, keys)
    {
        switch (key)
        {
            case Qt::Key_W:
            case Qt::Key_S:
            case Qt::Key_A:
            case Qt::Key_D:
                moveHandler(key, speedMagnification);
                break;
            case Qt::Key_Q:
            case Qt::Key_E:
                rollHandler(key);
                break;
            case Qt::Key_C:
            case Qt::Key_Z:
                upDownHandler(key, speedMagnification);
                break;
            //重置视角
            case Qt::Key_R:
                // resetAnimation();
                lookAtCenterAnimation();
                break;
            //功能键
            case Qt::Key_V:
                break;
        }
    }
    emit updateGraph();
}

void CameraController::moveHandler(int key, float speedMagnification = 1.0)
{
    //自由视角（默认）
    if (mode == 0)
    {
        switch (key)
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
        }
    }
    //全环绕视角
    else if (mode == 1)
    {
        //获得当前位置与中心点的距离
        double distance = (basePos  - baseCenter).length();
        switch (key)
        {
            //拉近
            case Qt::Key_W :
                if (distance <= 0.1)
                {
                    // 距离太近，不能继续拉近
                    return ;
                }
                basePosAdd(getMoveSpeed() * baseVector * speedMagnification);
                break;
            //远离
            case Qt::Key_S:
                basePosAdd(-getMoveSpeed() * baseVector * speedMagnification);
                break;
            //左转
            case Qt::Key_A:
                baseVector = rotateAboutAxis(baseVector, cameraUp,  -M_PI * getRollSpeed() *  speedMagnification).normalized();
                cameraRight = QVector3D::crossProduct(cameraUp, baseVector).normalized();
                basePos =  - baseVector * distance;
                break;
            //右转
            case Qt::Key_D:
                baseVector =  rotateAboutAxis(baseVector, cameraUp,  M_PI * getRollSpeed() *  speedMagnification).normalized();
                cameraRight = QVector3D::crossProduct(cameraUp, baseVector).normalized();
                basePos = - baseVector * distance;
                break;
        }
    }
}

void CameraController::rollHandler(int key)
{
    switch (key)
    {
        //顺时针滚筒
        case Qt::Key_E:
            cameraUp = rotateAboutAxis(cameraUp,  baseVector, M_PI * getRollSpeed());
            cameraRight = rotateAboutAxis(cameraRight,  baseVector, M_PI * getRollSpeed());
            break;
        //逆时针滚筒
        case Qt::Key_Q:
            cameraUp = rotateAboutAxis(cameraUp,  baseVector, -M_PI * getRollSpeed());
            cameraRight = rotateAboutAxis(cameraRight,  baseVector, -M_PI * getRollSpeed());
            break;
    }
}

void CameraController::upDownHandler(int key, float speedMagnification)
{
    //自由视角（默认）
    if (mode == 0)
    {
        switch (key)
        {
            //上升
            case Qt::Key_C:
                basePosAdd(getMoveSpeed() * cameraUp *  speedMagnification);
            //下降
            case Qt::Key_Z:
                basePosAdd(-getMoveSpeed() * cameraUp *  speedMagnification);
                break;
        }
    }
    //全环绕视角
    else if (mode == 1)
    {
        //获得当前位置与中心点的距离
        double distance = (basePos  - baseCenter).length();
        switch (key)
        {
            //上升环绕
            case Qt::Key_C:
                baseVector = rotateAboutAxis(baseVector, cameraRight,  M_PI * getRollSpeed() *  speedMagnification).normalized();
                cameraUp = - QVector3D::crossProduct(cameraRight, baseVector).normalized();
                basePos =  - baseVector * distance;
                break;
            //下降环绕
            case Qt::Key_Z:
                baseVector = rotateAboutAxis(baseVector, cameraRight,  -M_PI * getRollSpeed() *  speedMagnification).normalized();
                cameraUp = - QVector3D::crossProduct(cameraRight, baseVector).normalized();
                basePos =  - baseVector * distance;
                break;
        }
    }
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

void CameraController::resetAnimation()
{
    if (animationStart)
    {
        return;
    }
    //计算动画先后位置变化向量
    oldPos = basePos;
    targetPos = QVector3D(3, 3, 3);
    deltaPos =  targetPos - oldPos;

    //计算目标位置的相机方位
    targetVector = (QVector3D(0, 0, 0) - targetPos).normalized();
    targetCameraRight = QVector3D::crossProduct(baseUp, targetVector).normalized();
    targetCameraUp = QVector3D::crossProduct(targetVector, targetCameraRight).normalized();
    deltaVector = targetVector -  baseVector;
    deltaCameraRight = targetCameraRight - cameraRight;

    // 如果变化过小则不改变
    if (deltaPos.lengthSquared() <  animationAccuracy && deltaVector.lengthSquared() < animationAccuracy && deltaCameraRight.lengthSquared() < animationAccuracy)
    {
        return;
    }
    animationStart = true;
    animationTimer.start(animationInterval);
}

void CameraController::lookAtCenterAnimation()
{
    if (animationStart)
    {
        return;
    }
    //计算动画先后位置变化向量
    oldPos = basePos;
    targetPos = basePos;
    deltaPos =  targetPos - oldPos;

    //计算目标位置的相机方位
    targetVector = (baseCenter - targetPos).normalized();
    targetCameraRight = QVector3D::crossProduct(baseUp, targetVector).normalized();
    targetCameraUp = QVector3D::crossProduct(targetVector, targetCameraRight).normalized();
    deltaVector = targetVector -  baseVector;
    deltaCameraRight = targetCameraRight - cameraRight;

    // 如果变化过小则不改变
    if (deltaPos.lengthSquared() <  animationAccuracy && deltaVector.lengthSquared() < animationAccuracy && deltaCameraRight.lengthSquared() < animationAccuracy)
    {
        return;
    }
    animationStart = true;
    animationTimer.start(animationInterval);
}



