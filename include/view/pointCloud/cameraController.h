#ifndef CAMERACONTROLLER_H
#define CAMERACONTROLLER_H

#include "pointCloudBase.h"
#include <QPointF>
#include "QKeyEvent"
#include "QWidget"
#include "QTimer"


namespace  NSPointCloud
{
    class CameraController: public QWidget
    {
        Q_OBJECT
    public:


    public:
        CameraController();
        ~CameraController();


        const QVector3D &getBasePos() const;
        void setBasePos(const QVector3D &newBasePos);
        void basePosAdd(const QVector3D &delta);
        const QVector3D &getBaseDirection() const;
        void setBaseDirection(const QVector3D &newBaseDirection);
        const QVector3D &getBaseVector() const;
        void setBaseVector(const QVector3D &newBaseVector);
        const QVector3D &getBaseUp() const;
        void setBaseUp(const QVector3D &newBaseUp);
        const QVector3D &getCameraRight() const;
        void setCameraRight(const QVector3D &newCameraRight);
        const QVector3D &getCameraUp() const;
        void setCameraUp(const QVector3D &newCameraUp);

        float getBaseSpeed() const;
        void setBaseSpeed(float newBaseSpeed);
        float getMoveSpeed() const;
        void setMoveSpeed(float newMoveSpeed);
        float getRollSpeed() const;
        void setRollSpeed(float newRollSpeed);
        float getWheelRate() const;
        void setWheelRate(float newWheelRate);
        float getRotateSpeed() const;
        void setRotateSpeed(float newRotateSpeed);
        float getFov() const;


        void keypressActionHandler(QKeyEvent *event);
        void keyreleaseActionHandler(QKeyEvent *event);
        void wheelActionHandler(QWheelEvent *event);
        void mousepressActionHandler(QMouseEvent *event);
        void mousereleaseActionHandler(QMouseEvent *event);
        void mousemoveActionHandler(QMouseEvent *event);


    private :

        //原坐标系相机位置
        QVector3D basePos = QVector3D(3, 3, 3);
        //原坐标系相机朝向的位置
        QVector3D baseDirection =  QVector3D(0, 0, 0);
        //原坐标系相机朝向 方向向量
        QVector3D baseVector;
        //原坐标系相机的上方向
        QVector3D baseUp = QVector3D(0, 0, 1);
        //相机坐标系的相机右方向
        QVector3D cameraRight;
        //相机坐标系的相机上方向
        QVector3D cameraUp;


        //定时处理
        QTimer timer;
        //响应时间间隔(ms)
        int responseInterval =  10;
        //键盘按下按键列表
        QList<int> keys;
        //基础速度
        float baseSpeed = 1.0f;
        //移动速度
        float moveSpeed = 0.0150f;
        //滚筒旋转速度
        float rollSpeed =  0.20f;
        //视场角 \ 缩放
        float fov = 45;
        //滚轮视场角缩放速度
        float wheelRate = 0.010f;


        //鼠标拖动操作
        QPoint lastMousePos;
        QPoint currentMousePos;
        //拖动灵敏度
        float rotateSpeed = 0.05f;


    signals:
        void updateGraph();


    private slots:
        //键盘事件处理程序
        void handler();
    };
}


#endif // CAMERACONTROLLER_H
