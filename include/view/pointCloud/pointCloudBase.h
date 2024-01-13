#ifndef POINTCLOUDBASE_H
#define POINTCLOUDBASE_H
#include <QVector3D>

namespace NSPointCloud
{
    //转换rgb数值到0-1小数
    inline float RGBNormalized(float rgb)
    {
        return  rgb / 255.0f;
    }

    //计算originalVector绕正交向量axis旋转angle后的向量
    inline QVector3D rotateAboutAxis(const QVector3D &originalVector, const QVector3D &axis, float angle)
    {
        QVector3D normalizedAxis = axis.normalized();
        QVector3D rotated = (cos(angle) * originalVector) + (sin(angle) * QVector3D::crossProduct(normalizedAxis, originalVector));
        return rotated;
    }

    //计算两点距离
    inline float pointsDistance(const QPoint &a,  const QPoint  &b)
    {
        return sqrt(pow(a.x() - b.x(), 2) + pow(a.y()  - b.y(), 2));
    }
}

#endif // POINTCLOUDBASE_H
