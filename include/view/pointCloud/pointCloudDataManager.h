#ifndef POINTCLOUDDATAMANAGER_H
#define POINTCLOUDDATAMANAGER_H
#include  "vector"
#include "QObject"
#include "pointCloudBase.h"
#include "eigen3/Eigen/Dense"

namespace NSPointCloud
{

#define DEFAULT_MAX_POINTS_BUFFER_SIZE 1024 * 5000
#define DEFAULT_MAX_IMU_DATA_BUFFER_SIZE 1024 * 5000
#define DEFAULT_MAX_CACHE_SIZE 6 * 1024 * 5000
#define DEFAULT_FUSION_ACCURACY 1000000 * 4


    class PointCloudDataManager: public QObject
    {
        Q_OBJECT
    public:
        PointCloudDataManager(int _maxSize = DEFAULT_MAX_CACHE_SIZE);

        // todo 整合位置信息
        // bool addPoint();
        //接收点云原始数据
        bool addPoint(message::msg::LidarData::SharedPtr &_newData);
        //接收imu原始数据
        bool addImuData(message::msg::ImuData::SharedPtr &_newData);

        //最大缓存
        unsigned long getMaxCacheSize() const;
        void setMaxCacheSize(unsigned long newMaxCacheSize);
        //数据r
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
        unsigned long maxPointsBufferSize = DEFAULT_MAX_POINTS_BUFFER_SIZE;
        unsigned long maxImuDataBufferSize = DEFAULT_MAX_IMU_DATA_BUFFER_SIZE;
        //点云原始数据缓冲区
        std::vector<message::msg::LidarDataFrame> pointsBuffer;
        //imu原始数据缓冲区
        std::vector<message::msg::ImuDataFrame> imuDataBuffer;

        //最大Cache大小
        unsigned long maxCacheSize = DEFAULT_MAX_CACHE_SIZE;
        //当前数据(x,y,z,r,b,g);
        std::vector<float> data;
        //上次更新数据前的数据大小
        unsigned long lastDataSize = 0;
        //每一批数据的大小
        std::vector<unsigned long>  batchSize;


        //数据融合
        void fuseData();
        //数据融合精度，值越小越高 (5ms误差)
        int fusionAccuracy = DEFAULT_FUSION_ACCURACY;


        //处理点的颜色（进行颜色层次划分）
        void handlePointsColor();
        //点颜色的层次数
        int colorLevel = 10;
        //新旧点颜色
        float redNew =  255,  greenNew  =  31, blueNew = 0;
        float redOld = 28, greenOld = 126, blueOld = 214;

    public slots:
    };
}

#endif // POINTCLOUDDATAMANAGER_H
