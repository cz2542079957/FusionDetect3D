#ifndef POINTCLOUDDATAMANAGER_H
#define POINTCLOUDDATAMANAGER_H
#include  "vector"
#include "QObject"
#include "rclcpp/rclcpp.hpp"
#include  "pointCloudBase.h"
#include  "sensor_msgs/msg/laser_scan.hpp"
#include "message/msg/imu_data.hpp"

namespace NSPointCloud
{

#define DEFAUTL_MAX_CACHE_SIZE 6 * 1024 * 50

    class PointCloudDataManager: public QObject
    {
        Q_OBJECT
    public:
        PointCloudDataManager(int _maxSize = DEFAUTL_MAX_CACHE_SIZE);

        // todo 整合位置信息
        // bool addPoint();
        //接受不带颜色的节点
        bool addPoint(std::vector<float> _newData);
        //最大缓存
        unsigned long getMaxCacheSize() const;
        void setMaxCacheSize(unsigned long newMaxCacheSize);
        //数据
        std::vector<float> getData() const;
        //获取当前数据大小
        unsigned long getCurrentCacheSize() const;
        //更新前数据大小
        unsigned long getLastDataSize() const;
        //颜色层次
        int getColorLevel() const;
        //获取有多少点需要绘制(优化机制)
        unsigned long getPointNeedPaintNumber();

    private:
        //最大Cache大小
        unsigned long maxCacheSize = DEFAUTL_MAX_CACHE_SIZE;
        //当前数据(x,y,z,r,b,g);
        std::vector<float> data;
        //上次更新数据前的数据大小人
        unsigned long lastDataSize = 0;
        //每一批数据的大小
        std::vector<unsigned long>  batchSize;


        //处理点的颜色
        void handlePointsColor();
        //点颜色的层次数
        int colorLevel = 16;
        //新旧点颜色
        float redNew =  255,  greenNew  =  31, blueNew = 0;
        float redOld = 28, greenOld = 126, blueOld = 214;

    public slots:
        //拿到点云数据
        void recvPointsData(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        //拿到惯导数据
        void recvImuData(const message::msg::ImuData::SharedPtr msg);

    };
}

#endif // POINTCLOUDDATAMANAGER_H
