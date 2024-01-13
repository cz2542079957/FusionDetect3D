#ifndef POINTCLOUDDATAMANAGER_H
#define POINTCLOUDDATAMANAGER_H
#include  "vector"
#include  "pointCloudBase.h"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace NSPointCloud
{

#define DEFAUTL_MAX_CACHE_SIZE 6 * 1024 * 50

    class PointCloudDataManager
    {
    public:
        PointCloudDataManager(int _maxSize = DEFAUTL_MAX_CACHE_SIZE);

        //接受不带颜色的节点
        bool addPoint(std::vector<float> _newData);
        //接受坐标
        bool addPoint(float _x, float _y, float _z);
        //最大缓存
        unsigned long getMaxCacheSize() const;
        void setMaxCacheSize(unsigned long newMaxCacheSize);
        //数据
        std::vector<float> getData() const;
        //获取当前数据大小
        unsigned long getCurrentCacheSize() const;

        unsigned long getLastDataSize() const;

    private:
        //最大Cache大小
        unsigned long maxCacheSize = DEFAUTL_MAX_CACHE_SIZE;
        //当前数据(x,y,z,r,b,g);
        std::vector<float> data;
        //上次更新数据前的数据大小
        unsigned long lastDataSize = 0;

        bool extendBuffer();

    };
}

#endif // POINTCLOUDDATAMANAGER_H
