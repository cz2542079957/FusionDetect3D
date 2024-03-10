#ifndef POINTCLOUDBASE_H
#define POINTCLOUDBASE_H

#include "common.h"


//转换rgb数值到0-1小数
inline float RGBNormalized(float rgb)
{
    return  rgb / 255.0f;
}

//计算originalVector绕正交向量axis旋转angle后的向量
inline QVector3D rotateAboutAxis(const QVector3D &originalVector, const QVector3D &axis, double angle)
{
    QQuaternion rotationQuaternion = QQuaternion::fromAxisAndAngle(axis.normalized(), angle);
    QQuaternion originalAsQuaternion = QQuaternion(0.0f, originalVector.x(), originalVector.y(), originalVector.z());
    QQuaternion rotatedQuaternion = rotationQuaternion * originalAsQuaternion * rotationQuaternion.conjugated();
    return rotatedQuaternion.vector();
}

//计算两点距离
inline float pointsDistance(const QPoint &a,  const QPoint  &b)
{
    return sqrt(pow(a.x() - b.x(), 2) + pow(a.y()  - b.y(), 2));
}

#endif // POINTCLOUDBASE_H
