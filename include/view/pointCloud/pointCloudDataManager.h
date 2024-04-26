#ifndef POINTCLOUDDATAMANAGER_H
#define POINTCLOUDDATAMANAGER_H
#include "pointCloudBase.h"


//默认最大雷达原始数据缓冲区大小（单位：LidarDataFrame）
#define DEFAULT_MAX_POINTS_BUFFER_SIZE 1000 * 1000
//默认最大舵机数据原始数据缓冲区大小
#define DEFAULT_MAX_SERVO_DATA_BUFFER_SIZE 1000 * 1000
//默认最大imu原始数据缓冲区大小（单位：ImuDataFrame）
#define DEFAULT_MAX_IMU_DATA_BUFFER_SIZE 1000 * 1000
//默认最大编码器数据原始数据缓冲区大小
#define DEFAULT_MAX_ENCODER_DATA_BUFFER_SIZE 1000 * 1000
//默认最大缓存大小（单位：PointCloudVertex）
#define DEFAULT_MAX_CACHE_SIZE 10 * 1000 * 1000
//默认融合数据精度（纳秒）
#define DEFAULT_FUSION_ACCURACY 5000000ll
//默认融合最小精度（纳秒）
#define DEFAULT_MIN_FUSION_ACCURACY 1000000ll


//lidar数据溢出自动清理大小
#define LIDAR_DATA_AUTO_CLEAN_SIZE 100000
//lidarImu数据溢出自动清理大小
#define LIDAR_IMU_DATA_AUTO_CLEAN_SIZE 5000
//编码器数据溢出移动清理大小
#define ENCODER_DATA_AUTO_CLEAN_SIZE 1000
//carImu数据溢出自动清理大小
#define CAR_IMU_DATA_AUTO_CLEAN_SIZE 5000

struct Position
{
    //二维坐标
    QVector2D pos;
    //朝向方位
    QVector2D direct;
};

//点云 点结构
struct PointCloudVertex
{
    //空间坐标
    float x, y, z;
    // rgb 颜色(规格化0-1)
    float red = 0, green = 0, blue = 0;
};

//批次
struct BatchFrame
{
    // 批次大小
    unsigned long size;
    // 首个点时间戳
    long long timestamp;
    // 首个点在data数据中的index
    unsigned long firstDataIndex = 0;
};

/*
   作为PointCloudWidget的工具类，管理点云数据

   使用addPoint和addImuData传入原始数据，内部定时器会定时执行任务处理函数，自动融合数据、处理颜色渐变
*/
class PointCloudDataManager: public QObject
{
    Q_OBJECT
public:
    PointCloudDataManager(int _maxSize = DEFAULT_MAX_CACHE_SIZE);

    //接收点云原始数据
    bool addPoint(message::msg::LidarData::SharedPtr &_newData);
    //接收舵机原始数据
    bool addServoData(message::msg::CarServoData::SharedPtr &_newData);
    //接收lidar imu原始数据
    bool addLidarImuData(message::msg::ImuData::SharedPtr &_newData);
    //接收编码器原始数据
    bool addEncoderData(message::msg::CarEncoderData::SharedPtr &_newData);
    //接收car imu原始数据
    bool addCarImuData(message::msg::ImuData::SharedPtr &_newData);
    //清空
    void clearData();

    //最大缓存
    unsigned long getMaxCacheSize() const;
    void setMaxCacheSize(unsigned long _newMaxCacheSize);
    //数据
    const std::vector<PointCloudVertex> &getData() const;
    //获取当前数据大小
    unsigned long getCurrentCacheSize() const;
    //获取有多少点需要绘制(优化机制)
    unsigned long getPointNeedDrawNumber();

private:
    QTimer timer;
    //定时处理周期
    int tsakTimeInterval = 100;

    unsigned long maxPointsBufferSize = DEFAULT_MAX_POINTS_BUFFER_SIZE;
    unsigned long maxServoDataBufferSize = DEFAULT_MAX_SERVO_DATA_BUFFER_SIZE;
    unsigned long maxLidarImuDataBufferSize = DEFAULT_MAX_IMU_DATA_BUFFER_SIZE;
    unsigned long maxEncoderDataBufferSize = DEFAULT_MAX_ENCODER_DATA_BUFFER_SIZE;
    unsigned long maxCarImuDataBufferSize = DEFAULT_MAX_IMU_DATA_BUFFER_SIZE;
    //点云原始数据缓冲区
    std::vector<message::msg::LidarDataFrame> pointsBuffer;
    //舵机原始数据缓冲区
    std::vector<message::msg::CarServoData> servoDataBuffer;
    //lidar imu原始数据缓冲区
    std::vector<message::msg::ImuDataFrame> lidarImuDataBuffer;
    //编码器原始数据缓冲区
    std::vector<message::msg::CarEncoderData> encoderDataBuffer;
    //car imu原始数据缓冲区
    std::vector<message::msg::ImuDataFrame> carImuDataBuffer;

    //最大Cache大小
    unsigned long maxCacheSize = DEFAULT_MAX_CACHE_SIZE;
    //方位参数
    std::vector<Position> positions;
    //当前数据
    std::vector<PointCloudVertex> data;
    //每一批数据的大小(单位：PointCloudVertex)
    std::vector<BatchFrame>  batches;
    //当前已经处理完颜色的批次index
    long long handledBatchIndex = -1;
    long long timeOffset = -1;

    //数据融合
    bool fuseData();
    //融合方位数据 (imu和encoder)
    bool fusePositionData();
    //融合点云数据 (position、lidar以及servo)
    bool fuseLidarData();
    //是否启用自适应精度
    bool enableAutoAccuracy = true;
    //数据融合精度，值越小越高，超过误差的点会放弃 (默认5ms误差)
    long long fusionAccuracy = DEFAULT_FUSION_ACCURACY;

    //处理点的颜色（进行颜色层次划分）
    bool handlePointsColor();
    //处理最近n毫秒的点颜色
    long long recentColorHandleTime = 200;
    //新旧点颜色
    float redNew =  255,  greenNew  =  31, blueNew = 0;
    float redOld = 28, greenOld = 126, blueOld = 214;

public slots:
    //定时任务（执行点云数据融合 与 颜色渲染）
    void  scheduledTask();

signals:
    void updateGraph();
};
#endif // POINTCLOUDDATAMANAGER_H
