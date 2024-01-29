#include "pointCloudDataManager.h"

using namespace NSPointCloud;

PointCloudDataManager::PointCloudDataManager(int _maxCacheSize)
{
    pointsBuffer.reserve(maxPointsBufferSize);
    imuDataBuffer.reserve(maxImuDataBufferSize);
    maxCacheSize = _maxCacheSize;
    data.reserve(_maxCacheSize);

    //定时任务
    connect(&this->timer, SIGNAL(timeout()),  this,  SLOT(scheduledTask()));
    timer.start(std::chrono::milliseconds(tsakTimeInterval));
}

bool PointCloudDataManager::addPoint(message::msg::LidarData::SharedPtr &_newData)
{
    pointsBuffer.insert(pointsBuffer.end(), _newData->data.begin(), _newData->data.end());
    return true;
}

bool PointCloudDataManager::addImuData(message::msg::ImuData::SharedPtr &_newData)
{
    imuDataBuffer.insert(imuDataBuffer.end(), _newData->data.begin(), _newData->data.end());
    return true;
}


unsigned long PointCloudDataManager::getMaxCacheSize() const
{
    return maxCacheSize;
}

void PointCloudDataManager::setMaxCacheSize(unsigned long _newMaxCacheSize)
{
    maxCacheSize = _newMaxCacheSize;
    data.reserve(maxCacheSize);
}

const std::vector<PointCloudVertex> &PointCloudDataManager::getData() const
{
    return this->data;
}

void PointCloudDataManager::clearData()
{
    data.clear();
    batches.clear();
    handledBatchIndex = -1;
    pointsBuffer.clear();
    imuDataBuffer.clear();
}

unsigned long PointCloudDataManager::getCurrentCacheSize() const
{
    return data.size();
}

unsigned long PointCloudDataManager::getPointNeedDrawNumber()
{
    unsigned long length = 0;
    for (long i = batches.size() - 1;  i >= handledBatchIndex && i >= 0; i--)
    {
        length += batches[i].size;
    }
    return length;
}

void PointCloudDataManager::scheduledTask()
{
    bool updateFlag = false;
    //数据融合
    updateFlag |= fuseData();
    //颜色渲染
    updateFlag |= handlePointsColor();
    if (updateFlag)
    {
        emit updateGraph();
    }
}

bool PointCloudDataManager::fuseData()
{
    size_t minLen = std::min(pointsBuffer.size(), imuDataBuffer.size());
    if (minLen == 0 )
    {
        return false;
    }
    size_t count = 0 ;
    float ang2radCoe = M_PI / 180.0f;
    size_t imuIndex = 0;
    size_t pointsIndex = 0;
    size_t firstDataIndex = data.size() - 1;
    //由于imu数据密度较低，此处作为缓存优化性能
    size_t lastImuIndex = 0;
    double quaternion0 = imuDataBuffer[lastImuIndex].quaternion.quaternion_0;
    double quaternion1 = imuDataBuffer[lastImuIndex].quaternion.quaternion_1;
    double quaternion2 = imuDataBuffer[lastImuIndex].quaternion.quaternion_2;
    double quaternion3 = imuDataBuffer[lastImuIndex].quaternion.quaternion_3;
    Eigen::Quaterniond quaternion = Eigen::Quaterniond{quaternion0, quaternion1, quaternion2, quaternion3};
    float red = NSPointCloud::RGBNormalized(redOld), green = NSPointCloud::RGBNormalized(greenOld), blue = NSPointCloud::RGBNormalized(blueOld);
    while (imuIndex  < imuDataBuffer.size() && pointsIndex < pointsBuffer.size())
    {
        // RCLCPP_INFO(rclcpp::get_logger("main"), "1");
        if (pointsBuffer[pointsIndex].timestemp - imuDataBuffer[imuIndex].timestemp > fusionAccuracy)
        {
            imuIndex ++;
        }
        else if (abs(pointsBuffer[pointsIndex].timestemp - imuDataBuffer[imuIndex].timestemp) < fusionAccuracy)
        {
            if (pointsBuffer[pointsIndex].distance == 0 )
            {
                //距离为0视为无效点
                pointsIndex++;
                continue;
            }
            if (lastImuIndex != imuIndex)
            {
                //如果index相同则不再重复计算
                quaternion0 = imuDataBuffer[lastImuIndex].quaternion.quaternion_0;
                quaternion1 = imuDataBuffer[lastImuIndex].quaternion.quaternion_1;
                quaternion2 = imuDataBuffer[lastImuIndex].quaternion.quaternion_2;
                quaternion3 = imuDataBuffer[lastImuIndex].quaternion.quaternion_3;
                quaternion = Eigen::Quaterniond{quaternion0, quaternion1, quaternion2, quaternion3};
                //使用自定义精度
                if (enableCustomAccuracy)
                {
                    // RCLCPP_INFO(rclcpp::get_logger("main"), "%ld", imuDataBuffer[imuIndex].timestemp - imuDataBuffer[lastImuIndex].timestemp);
                    long long imuDataTimeInterval = imuDataBuffer[imuIndex].timestemp - imuDataBuffer[lastImuIndex].timestemp;
                    fusionAccuracy = std::min(imuDataTimeInterval,  15000000ll);
                }
                else
                {
                    fusionAccuracy = DEFAULT_FUSION_ACCURACY;
                }
                lastImuIndex = imuIndex;
            }
            // RCLCPP_INFO(rclcpp::get_logger("main"), "%ld", fusionAccuracy);
            double angle_rad =  pointsBuffer[pointsIndex].angle  * ang2radCoe  + M_PI_2;
            // 构建局部坐标向量
            Eigen::Vector3d local_vector( pointsBuffer[pointsIndex].distance * std::sin(angle_rad), pointsBuffer[pointsIndex].distance * std::cos(angle_rad), 0);
            Eigen::Vector3d worldVector = quaternion * local_vector;
            PointCloudVertex tempPoint =
            {
                static_cast<float>(worldVector.x()), static_cast<float>(worldVector.y()), static_cast<float>(worldVector.z()),
                red, green, blue
            };
            data.push_back(tempPoint);

            // RCLCPP_INFO(rclcpp::get_logger("main"), "%lf, %lf, %lf", imuDataBuffer[imuIndex].angular.roll, imuDataBuffer[imuIndex].angular.pitch,
            //             imuDataBuffer[imuIndex].angular.yaw);
            // RCLCPP_INFO(rclcpp::get_logger("main"), "%lld, %lld", imuDataBuffer[imuIndex].timestemp, pointsBuffer[pointsIndex].timestemp);
            count++;
            pointsIndex++;
        }
        else
        {
            pointsIndex++;
        }
    }
    // RCLCPP_INFO(rclcpp::get_logger("main"), "%ld", count);
    batches.push_back(BatchFrame{count, pointsBuffer[0].timestemp, firstDataIndex});

    pointsBuffer.erase(pointsBuffer.begin(), pointsBuffer.begin() +  pointsIndex);
    imuDataBuffer.erase(imuDataBuffer.begin(), imuDataBuffer.begin() + imuIndex);
    return true;
}

bool PointCloudDataManager::handlePointsColor()
{
    if (batches.size() == 0 || handledBatchIndex == (long long)batches.size() - 1)
    {
        //批次没有数据 或者 全部处理完毕则不需要更新
        return false;
    }
    long long timeNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    if (timeOffset == -1)
    {
        //计算时间偏移量,用于同步时间
        timeOffset = timeNow - batches[batches.size() - 1].timeStemp / 1000000;
    }
    //计算当前最后一次维护的批次号
    for (size_t i = handledBatchIndex + 1 ; i < batches.size(); i++)
    {
        if (timeNow - batches[i].timeStemp / 1000000 - timeOffset <  recentColorHandleTime)
        {
            break;
        }
        handledBatchIndex = i;
    }
    // RCLCPP_INFO(rclcpp::get_logger("main"), "%lld , %ld",  handledBatchIndex, batches.size());
    // RCLCPP_INFO(rclcpp::get_logger("main"), "%lld, %ld", batches[batches.size() - 1].timeStemp,  batches[batches.size() - 1].firstDataIndex);
    // RCLCPP_INFO(rclcpp::get_logger("main"), "%ld",  batches.size());
    int colors = batches.size() - handledBatchIndex;
    float redDelta = (redOld - redNew) / colors,
          greenDelta = (greenOld - greenNew) / colors,
          blueDelta = (blueOld - blueNew) / colors;
    // RCLCPP_INFO(rclcpp::get_logger("main"), "%d : %f, %f, %f", colors, redDelta, greenDelta, blueDelta);
    for (size_t i = handledBatchIndex + 1; i < batches.size(); i ++)
    {
        int offset = i - handledBatchIndex - 1;
        float red = NSPointCloud::RGBNormalized(redOld - redDelta * offset);
        float green = NSPointCloud::RGBNormalized(greenOld - greenDelta * offset);
        float blue = NSPointCloud::RGBNormalized(blueOld - blueDelta * offset);
        for (size_t j = batches[i].firstDataIndex; j < batches[i].firstDataIndex + batches[i].size; j ++)
        {
            data[j].red = red;
            data[j].green = green;
            data[j].blue = blue;
        }
    }
    return true;
}

