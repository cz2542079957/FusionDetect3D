#include "pointCloudDataManager.h"

PointCloudDataManager::PointCloudDataManager(int _maxCacheSize)
{
    pointsBuffer.reserve(maxPointsBufferSize);
    servoDataBuffer.reserve(maxServoDataBufferSize);
    lidarImuDataBuffer.reserve(maxLidarImuDataBufferSize);
    encoderDataBuffer.reserve(maxEncoderDataBufferSize);
    carImuDataBuffer.reserve(maxCarImuDataBufferSize);

    Position beginPosition;
    beginPosition.pos.setX(0), beginPosition.pos.setY(0);
    beginPosition.direct.setX(1.0), beginPosition.direct.setY(0);
    positions.push_back(beginPosition);

    maxCacheSize = _maxCacheSize;
    data.reserve(_maxCacheSize);

    // 定时任务
    connect(&this->timer, SIGNAL(timeout()), this, SLOT(scheduledTask()));
    timer.start(std::chrono::milliseconds(tsakTimeInterval));
}

bool PointCloudDataManager::addPoint(message::msg::LidarData::SharedPtr &_newData)
{
    pointsBuffer.insert(pointsBuffer.end(), _newData->data.begin(), _newData->data.end());
    return true;
}

bool PointCloudDataManager::addServoData(message::msg::CarServoData::SharedPtr &_newData)
{
    Q_UNUSED(_newData);
    // servoDataBuffer.push_back(*_newData);
    return true;
}

bool PointCloudDataManager::addLidarImuData(message::msg::ImuData::SharedPtr &_newData)
{
    // qDebug() << lidarImuDataBuffer.size();
    lidarImuDataBuffer.insert(lidarImuDataBuffer.end(), _newData->data.begin(), _newData->data.end());
    return true;
}

bool PointCloudDataManager::addEncoderData(message::msg::CarEncoderData::SharedPtr &_newData)
{
    // qDebug() << _newData->encoder1 << " " << _newData->encoder2;
    encoderDataBuffer.push_back(*_newData);
    return true;
}

bool PointCloudDataManager::addCarImuData(message::msg::ImuData::SharedPtr &_newData)
{
    carImuDataBuffer.insert(carImuDataBuffer.end(), _newData->data.begin(), _newData->data.end());
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
    servoDataBuffer.clear();
    lidarImuDataBuffer.clear();
    encoderDataBuffer.clear();
    carImuDataBuffer.clear();
}

unsigned long PointCloudDataManager::getCurrentCacheSize() const
{
    return data.size();
}

unsigned long PointCloudDataManager::getPointNeedDrawNumber()
{
    unsigned long length = 0;
    for (long i = batches.size() - 1; i >= handledBatchIndex && i >= 0; i--)
    {
        length += batches[i].size;
    }
    return length;
}

void PointCloudDataManager::scheduledTask()
{
    bool updateFlag = false;
    // 数据融合
    updateFlag |= fuseData();
    // 颜色渲染
    //  updateFlag |= handlePointsColor();
    if (updateFlag)
    {
        emit updateGraph();
    }
}

bool PointCloudDataManager::fuseData()
{
    bool updateFlag = false;
    updateFlag |= fusePositionData();
    updateFlag |= fuseLidarData();
    return updateFlag;
}

bool PointCloudDataManager::fusePositionData()
{
    // 融合carIMU和encoder数据
    // qDebug() << carImuDataBuffer.size() << " " << encoderDataBuffer.size();
    qDebug() << encoderDataBuffer[encoderDataBuffer.size() - 1].encoder1 << " " << encoderDataBuffer[encoderDataBuffer.size() - 1].encoder2 << " " <<
             encoderDataBuffer[encoderDataBuffer.size() - 1].encoder3 << " " << encoderDataBuffer[encoderDataBuffer.size() - 1].encoder4;
    if (encoderDataBuffer.size() == 0 || carImuDataBuffer.size() == 0)
    {
        if (encoderDataBuffer.size() > ENCODER_DATA_AUTO_CLEAN_SIZE)
        {
            encoderDataBuffer.clear();
        }
        if (carImuDataBuffer.size() > CAR_IMU_DATA_AUTO_CLEAN_SIZE)
        {
            carImuDataBuffer.clear();
        }
        return false;
    }


    return true;
}

bool PointCloudDataManager::fuseLidarData()
{
    // 融合position和lidar、lidarIMU数据
    // 以lidarImuDataBuffer为主，pointsBuffer配合融合
    //  qDebug() << pointsBuffer.size() << " " << lidarImuDataBuffer.size();
    if (pointsBuffer.size() == 0 || lidarImuDataBuffer.size() == 0)
    {
        if (lidarImuDataBuffer.size() > LIDAR_IMU_DATA_AUTO_CLEAN_SIZE)
        {
            lidarImuDataBuffer.clear();
        }
        if (pointsBuffer.size() > LIDAR_DATA_AUTO_CLEAN_SIZE)
        {
            pointsBuffer.clear();
        }
        return false;
    }

    size_t count = 0;
    float ang2radCoe = M_PI / 180.0f;
    size_t imuIndex = 0;
    size_t pointsIndex = 0;
    size_t firstDataIndex = data.size() - 1;
    // 由于imu数据密度较低，此处作为缓存优化性能
    size_t lastImuIndex = 0;
    Eigen::Matrix3d Rx = Eigen::AngleAxisd(-lidarImuDataBuffer[imuIndex].angular.pitch * ang2radCoe, Eigen::Vector3d::UnitX()).matrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(lidarImuDataBuffer[imuIndex].angular.yaw * ang2radCoe, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(-lidarImuDataBuffer[imuIndex].angular.roll * ang2radCoe, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d R = Rz * Ry * Rx;
    float red = RGBNormalized(redOld), green = RGBNormalized(greenOld), blue = RGBNormalized(blueOld);
    while (imuIndex < lidarImuDataBuffer.size() && pointsIndex < pointsBuffer.size())
    {
        if (lastImuIndex != imuIndex)
        {
            // 如果index相同则不再重复计算
            Rx = Eigen::AngleAxisd(-lidarImuDataBuffer[imuIndex].angular.pitch * ang2radCoe, Eigen::Vector3d::UnitX()).matrix();
            Ry = Eigen::AngleAxisd(lidarImuDataBuffer[imuIndex].angular.yaw * ang2radCoe, Eigen::Vector3d::UnitY()).matrix();
            Rz = Eigen::AngleAxisd(-lidarImuDataBuffer[imuIndex].angular.roll * ang2radCoe, Eigen::Vector3d::UnitZ()).matrix();
            R = Rx * Ry * Rz;
            if (lidarImuDataBuffer[imuIndex].angular_velocity.angular_velocity_y > -10)
            {
                lastImuIndex = imuIndex;
                imuIndex++;
                continue;
            }

            // 启用自动精度
            if (enableAutoAccuracy)
            {
                long long imuDataTimeInterval = lidarImuDataBuffer[imuIndex].timestamp - lidarImuDataBuffer[lastImuIndex].timestamp;
                fusionAccuracy = std::min(imuDataTimeInterval / 2, DEFAULT_FUSION_ACCURACY);
                if (fusionAccuracy < DEFAULT_MIN_FUSION_ACCURACY)
                {
                    // 可能是重复数据，排除
                    lastImuIndex = imuIndex;
                    imuIndex++;
                    continue;
                }
            }
            else
            {
                fusionAccuracy = DEFAULT_FUSION_ACCURACY;
            }
            lastImuIndex = imuIndex;
        }

        if (pointsBuffer[pointsIndex].timestamp - lidarImuDataBuffer[imuIndex].timestamp > fusionAccuracy)
        {
            imuIndex++;
        }
        else if (abs(pointsBuffer[pointsIndex].timestamp - lidarImuDataBuffer[imuIndex].timestamp) < fusionAccuracy)
        {
            if (pointsBuffer[pointsIndex].distance == 0)
            {
                // 距离为0视为无效点
                pointsIndex++;
                continue;
            }

            // RCLCPP_INFO(rclcpp::get_logger("main"), "%ld", fusionAccuracy);
            double angle_rad = (pointsBuffer[pointsIndex].angle + 90 - 0.5) * ang2radCoe;
            Eigen::Vector3d local_vector(-pointsBuffer[pointsIndex].distance * std::sin(angle_rad), pointsBuffer[pointsIndex].distance * std::cos(angle_rad), 0);
            Eigen::Vector3d worldVector = R * local_vector;

            // RGB(255, 51, 0)
            // RGB(28, 126, 214
            double normalized_distance = (pointsBuffer[pointsIndex].distance - 0) / 3;
            red = RGBNormalized((1 - normalized_distance) * 255 + normalized_distance * 28);
            green = RGBNormalized((1 - normalized_distance) * 51 + normalized_distance * 126);
            blue = RGBNormalized((1 - normalized_distance) * 0 + normalized_distance * 214);
            PointCloudVertex tempPoint =
            {
                static_cast<float>(-worldVector.x()), static_cast<float>(worldVector.z()), static_cast<float>(worldVector.y()),
                red, green, blue
            };
            data.push_back(tempPoint);

            count++;
            pointsIndex++;
        }
        else
        {
            pointsIndex++;
        }
    }
    // RCLCPP_INFO(rclcpp::get_logger("main"), "done： %ld %ld %ld", count, imuIndex, pointsIndex);
    batches.push_back(BatchFrame{count, pointsBuffer[0].timestamp, firstDataIndex});

    pointsBuffer.erase(pointsBuffer.begin(), pointsBuffer.begin() + pointsIndex);
    lidarImuDataBuffer.erase(lidarImuDataBuffer.begin(), lidarImuDataBuffer.begin() + imuIndex);

    return true;
}

bool PointCloudDataManager::handlePointsColor()
{
    if (batches.size() == 0 || handledBatchIndex == (long long)batches.size() - 1)
    {
        // 批次没有数据 或者 全部处理完毕则不需要更新
        return false;
    }
    long long timeNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    if (timeOffset == -1)
    {
        // 计算时间偏移量,用于同步时间
        timeOffset = timeNow - batches[batches.size() - 1].timestamp / 1000000;
    }
    // 计算当前最后一次维护的批次号
    for (size_t i = handledBatchIndex + 1; i < batches.size(); i++)
    {
        if (timeNow - batches[i].timestamp / 1000000 - timeOffset < recentColorHandleTime)
        {
            break;
        }
        handledBatchIndex = i;
    }
    // RCLCPP_INFO(rclcpp::get_logger("main"), "%lld , %ld",  handledBatchIndex, batches.size());
    // RCLCPP_INFO(rclcpp::get_logger("main"), "%lld, %ld", batches[batches.size() - 1].timestamp,  batches[batches.size() - 1].firstDataIndex);
    // RCLCPP_INFO(rclcpp::get_logger("main"), "%ld",  batches.size());
    int colors = batches.size() - handledBatchIndex;
    float redDelta = (redOld - redNew) / colors,
          greenDelta = (greenOld - greenNew) / colors,
          blueDelta = (blueOld - blueNew) / colors;
    // RCLCPP_INFO(rclcpp::get_logger("main"), "%d : %f, %f, %f", colors, redDelta, greenDelta, blueDelta);
    for (size_t i = handledBatchIndex + 1; i < batches.size(); i++)
    {
        int offset = i - handledBatchIndex - 1;
        float red = RGBNormalized(redOld - redDelta * offset);
        float green = RGBNormalized(greenOld - greenDelta * offset);
        float blue = RGBNormalized(blueOld - blueDelta * offset);
        for (size_t j = batches[i].firstDataIndex; j < batches[i].firstDataIndex + batches[i].size; j++)
        {
            data[j].red = red;
            data[j].green = green;
            data[j].blue = blue;
        }
    }
    return true;
}
