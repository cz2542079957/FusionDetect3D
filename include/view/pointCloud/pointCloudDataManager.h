#ifndef POINTCLOUDDATAMANAGER_H
#define POINTCLOUDDATAMANAGER_H
#include  "vector"
#include "QObject"
#include "pointCloudBase.h"
#include "eigen3/Eigen/Dense"

#include <QTimer>

namespace NSPointCloud
{

#define DEFAULT_MAX_POINTS_BUFFER_SIZE 1000 * 5000
#define DEFAULT_MAX_IMU_DATA_BUFFER_SIZE 1000 * 5000
#define DEFAULT_MAX_CACHE_SIZE 10 * 1000 * 1000
#define DEFAULT_FUSION_ACCURACY 1000000 * 5

    //点云 点结构
    struct PointCloudVertex
    {
        float x, y, z;
        float red = 0, green = 0, blue = 0;
    };

    //批次
    struct BatchFrame
    {
        unsigned long size;
        long long timeStemp;
        unsigned long firstDataIndex = 0;
    };

    /*
        PointCloudWidget的工具类，用于处理和融合点云和imu数据
        - 首先使用addPoint与addImuData来传入数据，会自动调用fuseData来融合数据，最终生成data的数据
    */
    /*class PointCloudDataManager: public QObject
    {
        Q_OBJECT
    public:
        PointCloudDataManager(int _maxSize = DEFAULT_MAX_CACHE_SIZE);

        //接收点云原始数据
        bool addPoint(message::msg::LidarData::SharedPtr &_newData);
        //接收imu原始数据
        bool addImuData(message::msg::ImuData::SharedPtr &_newData);
        //清空
        void clearData();

        //最大缓存
        unsigned long getMaxCacheSize() const;
        void setMaxCacheSize(unsigned long newMaxCacheSize);
        //数据r
        std::vector<float> getData() const;
        //获取当前数据大小
        unsigned long getCurrentCacheSize() const;
        //颜色层次
        int getColorLevel() const;
        //获取有多少点需要绘制(优化机制)
        unsigned long getPointNeedDrawNumber();
        //获取需要绘制的点数据
        std::vector<float> getDataNeedDraw() const;

    private:
        QTimer timer;

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
        //需要绘制的新数据（性能优化策略）
        std::vector<float> dataNeedDraw;
        //每一批数据的大小
        std::vector<unsigned long>  batchSize;

        //数据融合
        void fuseData();
        //是否启用自定义精度
        bool enableCustomAccuracy = false;
        //数据融合精度，值越小越高 (5ms误差)
        long long fusionAccuracy = DEFAULT_FUSION_ACCURACY;

        //处理点的颜色（进行颜色层次划分）
        void handlePointsColor();
        //点颜色的层次数
        long long colorLevel = 10;
        //当前已经处理完的批次index
        long long handledBatchIndex = -1;
        //新旧点颜色
        float redNew =  255,  greenNew  =  31, blueNew = 0;
        float redOld = 28, greenOld = 126, blueOld = 214;

    public slots:
        //定时任务（执行点云数据融合 与 颜色渲染）
        void  scheduledTask();

    signals:
        void updateGraph();
    };*/

    class PointCloudDataManager: public QObject
    {
        Q_OBJECT
    public:
        PointCloudDataManager(int _maxSize = DEFAULT_MAX_CACHE_SIZE);

        //接收点云原始数据
        bool addPoint(message::msg::LidarData::SharedPtr &_newData);
        //接收imu原始数据
        bool addImuData(message::msg::ImuData::SharedPtr &_newData);
        //清空
        void clearData();

        //最大缓存
        unsigned long getMaxCacheSize() const;
        void setMaxCacheSize(unsigned long newMaxCacheSize);
        //数据
        const std::vector<PointCloudVertex> &getData() const;
        //获取当前数据大小
        unsigned long getCurrentCacheSize() const;
        //获取有多少点需要绘制(优化机制)
        unsigned long getPointNeedDrawNumber();

    private:
        QTimer timer;

        unsigned long maxPointsBufferSize = DEFAULT_MAX_POINTS_BUFFER_SIZE;
        unsigned long maxImuDataBufferSize = DEFAULT_MAX_IMU_DATA_BUFFER_SIZE;
        //点云原始数据缓冲区
        std::vector<message::msg::LidarDataFrame> pointsBuffer;
        //imu原始数据缓冲区
        std::vector<message::msg::ImuDataFrame> imuDataBuffer;

        //最大Cache大小
        unsigned long maxCacheSize = DEFAULT_MAX_CACHE_SIZE;
        //当前数据
        std::vector<PointCloudVertex> data;
        //每一批数据的大小(单位：PointCloudVertex)
        std::vector<BatchFrame>  batches;
        //当前已经处理完颜色的批次index
        long long handledBatchIndex = -1;
        long long timeOffset = -1;

        //数据融合
        bool fuseData();
        //是否启用自定义精度
        bool enableCustomAccuracy = false;
        //数据融合精度，值越小越高，超过误差的点会放弃 (默认5ms误差)
        long long fusionAccuracy = DEFAULT_FUSION_ACCURACY;

        //处理点的颜色（进行颜色层次划分）
        bool handlePointsColor();
        //处理最近n毫秒的点颜色
        long long recentColorHandleTime = 300;
        //新旧点颜色
        float redNew =  255,  greenNew  =  31, blueNew = 0;
        float redOld = 28, greenOld = 126, blueOld = 214;

    public slots:
        //定时任务（执行点云数据融合 与 颜色渲染）
        void  scheduledTask();

    signals:
        void updateGraph();
    };
}

#endif // POINTCLOUDDATAMANAGER_H
