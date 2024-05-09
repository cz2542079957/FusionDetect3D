#include "pointCloudDataManager.h"
#include "pointCloudWidget.h"

PointCloudDataManager::PointCloudDataManager(PointCloudWidget *parent, int _maxCacheSize)
{
    this->parent = parent;

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
    connect(&this->timer, &QTimer::timeout, this, &PointCloudDataManager::scheduledTask);
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

const std::vector<PointCloudVertex> &PointCloudDataManager::getPositionData() const
{
    return this->positionsData;
}

const std::vector<PointCloudVertex> &PointCloudDataManager::getData() const
{
    return this->data;
}

void PointCloudDataManager::clearPointCloud()
{
    pointsBuffer.clear();
    servoDataBuffer.clear();
    lidarImuDataBuffer.clear();

    data.clear();
    batches.clear();
    handledBatchIndex = -1;
}

void PointCloudDataManager::clearPositionPoint()
{
    encoderDataBuffer.clear();
    carImuDataBuffer.clear();

    //不清空positions数据，因为点云计算依赖有效位置数据，
    positionsData.clear();
}

void PointCloudDataManager::syncIMURoll()
{
    imuSyncCoe = lidarImuDataBuffer[lidarImuDataBuffer.size() - 1].angular.yaw - carImuDataBuffer[carImuDataBuffer.size() - 1].angular.roll;
    // qDebug() << carImuDataBuffer[carImuDataBuffer.size() - 1].angular.roll << " " << lidarImuDataBuffer[lidarImuDataBuffer.size() - 1].angular.yaw;
}

unsigned long PointCloudDataManager::getCurrentPositionCacheSize() const
{
    return positionsData.size();
}

unsigned long PointCloudDataManager::getPositionNeedDrawNumber()
{
    return positionsData.size();
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
    updateFlag |= handlePointsColor();
    if (updateFlag)
    {
        emit updateGraph();
    }
}

bool PointCloudDataManager::fuseData()
{
    int currentMode = parent->car.getMode();
    static int lastMode = -1;
    bool updateFlag = false;
    if (currentMode != 1 && lastMode != currentMode)
    {
        batches.clear();
        handledBatchIndex = -1;
        pointsBuffer.clear();
        servoDataBuffer.clear();
        lidarImuDataBuffer.clear();
        encoderDataBuffer.clear();
        carImuDataBuffer.clear();
        lastMode = currentMode;
        return updateFlag;
    }
    // qDebug() << lastMode << " " <<    lastMode  ;
    lastMode = currentMode;
    updateFlag |= fusePositionData();
    updateFlag |= fuseLidarData();
    return updateFlag;
}

bool PointCloudDataManager::fusePositionData()
{
    // 融合carIMU和encoder数据
    // qDebug() << carImuDataBuffer.size() << " " << encoderDataBuffer.size();
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

    // 以编码器数据为主
    float ang2radCoe = M_PI / 180.0f;
    size_t encoderIndex = 0, carImuIndex = 0;
    // float red = RGBNormalized(redOld), green = RGBNormalized(greenOld), blue = RGBNormalized(blueOld);
    while (encoderIndex < encoderDataBuffer.size() &&  carImuIndex < carImuDataBuffer.size())
    {
        if (encoderIndex > 0)
        {
            positionFusionAccuracy = encoderDataBuffer[encoderIndex].timestamp - encoderDataBuffer[encoderIndex - 1].timestamp;
        }

        // 编码器数据中如果有正负不同的情况，说明是转向动作，不计算
        message::msg::CarEncoderData *encoderData = &encoderDataBuffer[encoderIndex];
        if (!((encoderData->encoder1 > 0 && encoderData->encoder2 > 0 && encoderData->encoder3 > 0 && encoderData->encoder4 > 0)
                || (encoderData->encoder1 < 0 && encoderData->encoder2 < 0 && encoderData->encoder3 < 0 && encoderData->encoder4 < 0)))
        {
            // qDebug() << encoderData->encoder1 << " " << encoderData->encoder2 << " " <<
            //          encoderData->encoder3 << " " << encoderData->encoder4;
            encoderIndex ++;
            continue;
        }

        if (encoderData->timestamp - carImuDataBuffer[carImuIndex].timestamp > DEFAULT_MIN_FUSION_ACCURACY * 2)
        {
            carImuIndex ++;
        }
        else if (carImuDataBuffer[carImuIndex].timestamp - encoderData->timestamp  > DEFAULT_MIN_FUSION_ACCURACY * 2)
        {
            encoderIndex ++;
        }
        else
        {
            // qDebug() << encoderDataBuffer[encoderIndex].encoder1 << " " << encoderDataBuffer[encoderIndex].encoder2;
            float avgEncoderData = (encoderData->encoder1 + encoderData->encoder2 + encoderData->encoder3 + encoderData->encoder4) / 4.0;

            // qDebug() << carImuDataBuffer[carImuIndex].angular.roll << " " <<  avgEncoderData;
            carImuDataBuffer[carImuIndex].angular.roll += imuSyncCoe - 0;
            float xDelta = std::sin(-carImuDataBuffer[carImuIndex].angular.roll * ang2radCoe) * avgEncoderData / 19050.17;
            float yDelta = std::cos(-carImuDataBuffer[carImuIndex].angular.roll * ang2radCoe) * avgEncoderData / 19050.17;
            float x = positions[positions.size() - 1].pos.x() - xDelta;
            float y = positions[positions.size() - 1].pos.y() - yDelta;
            Position tempPosition =
            {
                {x, y}, {0, 0}
            };
            PointCloudVertex tempPoint =
            {
                x, y, 0,
                RGBNormalized(255), RGBNormalized(248), RGBNormalized(91)
            };
            // qDebug() <<  x << " " << y;
            positions.push_back(tempPosition);
            positionsData.push_back(tempPoint);
            carImuIndex ++;
            encoderIndex ++;
        }
    }

    encoderDataBuffer.erase(encoderDataBuffer.begin(), encoderDataBuffer.begin() + encoderIndex);
    carImuDataBuffer.erase(carImuDataBuffer.begin(), carImuDataBuffer.begin() + carImuIndex);

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
    Eigen::Matrix3d Rx = Eigen::AngleAxisd(lidarImuDataBuffer[imuIndex].angular.yaw * ang2radCoe, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(-lidarImuDataBuffer[imuIndex].angular.pitch * ang2radCoe, Eigen::Vector3d::UnitX()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(-lidarImuDataBuffer[imuIndex].angular.roll * ang2radCoe, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d R = Rx * Ry * Rz;
    float red = RGBNormalized(redOld), green = RGBNormalized(greenOld), blue = RGBNormalized(blueOld);
    while (imuIndex < lidarImuDataBuffer.size() && pointsIndex < pointsBuffer.size())
    {
        //如果当前不是正向扫描，则跳过
        if (lidarImuDataBuffer[imuIndex].angular_velocity.angular_velocity_y > -10.0)
        {
            // qDebug() << lidarImuDataBuffer[imuIndex].angular_velocity.angular_velocity_y;
            lastImuIndex = imuIndex;
            imuIndex++;
            continue;
        }

        //更新旋转矩阵，更新融合精度
        if (lastImuIndex != imuIndex)
        {
            // 如果index相同则不再重复计算
            // Rx = Eigen::AngleAxisd(-lidarImuDataBuffer[imuIndex].angular.pitch * ang2radCoe, Eigen::Vector3d::UnitX()).matrix();
            // Ry = Eigen::AngleAxisd(lidarImuDataBuffer[imuIndex].angular.yaw * ang2radCoe, Eigen::Vector3d::UnitY()).matrix();
            Rx = Eigen::AngleAxisd(lidarImuDataBuffer[imuIndex].angular.yaw * ang2radCoe, Eigen::Vector3d::UnitY()).matrix();
            Ry = Eigen::AngleAxisd(-lidarImuDataBuffer[imuIndex].angular.pitch * ang2radCoe, Eigen::Vector3d::UnitX()).matrix();
            Rz = Eigen::AngleAxisd(-lidarImuDataBuffer[imuIndex].angular.roll * ang2radCoe, Eigen::Vector3d::UnitZ()).matrix();
            R = Rx * Ry * Rz;

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

        //如果激光点距离不在有效范围，则无视
        if (pointsBuffer[pointsIndex].distance <= MIN_LIDAR_DISTANCE || pointsBuffer[pointsIndex].distance  > MAX_LIDAR_DISTANCE)
        {
            pointsIndex++;
            continue;
        }

        if (pointsBuffer[pointsIndex].timestamp - lidarImuDataBuffer[imuIndex].timestamp > fusionAccuracy)
        {
            imuIndex++;
        }
        else if (lidarImuDataBuffer[imuIndex].timestamp - pointsBuffer[pointsIndex].timestamp > fusionAccuracy)
        {
            pointsIndex++;
        }
        else
        {
            // RCLCPP_INFO(rclcpp::get_logger("main"), "%ld", fusionAccuracy);
            double angle_rad = (pointsBuffer[pointsIndex].angle + 90 - 0) * ang2radCoe;
            Eigen::Vector3d local_vector(-pointsBuffer[pointsIndex].distance * std::sin(angle_rad), pointsBuffer[pointsIndex].distance * std::cos(angle_rad), 0);
            Eigen::Vector3d worldVector = R * local_vector;

            float positionX = positions[positions.size() - 1].pos.x(), positionY = positions[positions.size() - 1].pos.y();
            // RGB(255, 51, 0)
            // RGB(28, 126, 214
            double normalized_distance = (pointsBuffer[pointsIndex].distance - MIN_LIDAR_DISTANCE) / MAX_LIDAR_DISTANCE;
            red = RGBNormalized(255 - normalized_distance * 227);
            green = RGBNormalized(51 + normalized_distance * 75);
            blue = RGBNormalized(normalized_distance * 214);
            PointCloudVertex tempPoint =
            {
                static_cast<float>(-worldVector.x() + positionX), static_cast<float>(worldVector.z() + positionY), static_cast<float>(worldVector.y()),
                red, green, blue
            };
            data.push_back(tempPoint);

            count++;
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
    // int colors = batches.size() - handledBatchIndex;
    // float redDelta = (redOld - redNew) / colors,
    //       greenDelta = (greenOld - greenNew) / colors,
    //       blueDelta = (blueOld - blueNew) / colors;
    // // RCLCPP_INFO(rclcpp::get_logger("main"), "%d : %f, %f, %f", colors, redDelta, greenDelta, blueDelta);
    // for (size_t i = handledBatchIndex + 1; i < batches.size(); i++)
    // {
    //     int offset = i - handledBatchIndex - 1;
    //     float red = RGBNormalized(redOld - redDelta * offset);
    //     float green = RGBNormalized(greenOld - greenDelta * offset);
    //     float blue = RGBNormalized(blueOld - blueDelta * offset);
    //     for (size_t j = batches[i].firstDataIndex; j < batches[i].firstDataIndex + batches[i].size; j++)
    //     {
    //         data[j].red = red;
    //         data[j].green = green;
    //         data[j].blue = blue;
    //     }
    // }
    // for (size_t i = handledBatchIndex + 1; i < batches.size(); i++)
    // {
    //     for (size_t j = batches[i].firstDataIndex; j < batches[i].firstDataIndex + batches[i].size; j++)
    //     {
    //         double normalized_distance = (pointsBuffer[pointsIndex].distance - 0) / 3;
    //         float red = RGBNormalized((1 - normalized_distance) * 255 + normalized_distance * 28);
    //         float green = RGBNormalized((1 - normalized_distance) * 51 + normalized_distance * 126);
    //         float blue = RGBNormalized((1 - normalized_distance) * 0 + normalized_distance * 214);
    //         data[j].red = red;
    //         data[j].green = green;
    //         data[j].blue = blue;
    //     }
    // }
    return false;
}
