#include "pointCloudDataManager.h"

PointCloudDataManager::PointCloudDataManager(int _maxCacheSize)
{
    pointsBuffer.reserve(maxPointsBufferSize);
    servoDataBuffer.reserve(maxServoDataBufferSize);
    lidarImuDataBuffer.reserve(maxLidarImuDataBufferSize);
    encoderDataBuffer.reserve(maxEncoderDataBufferSize);

    Position beginPosition;
    beginPosition.pos.setX(0), beginPosition.pos.setY(0);
    beginPosition.direct.setX(1.0), beginPosition.direct.setY(0);
    positions.push_back(beginPosition);

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

bool PointCloudDataManager::addServoData(message::msg::CarServoData::SharedPtr &_newData)
{
    // servoDataBuffer.push_back(*_newData);
    return true;
}

bool PointCloudDataManager::addLidarImuData(message::msg::ImuData::SharedPtr &_newData)
{
    // qDebug() << lidarImuDataBuffer.size();
    //对数据进行插帧
    // std::vector<message::msg::ImuDataFrame> *data = &(_newData->data);
    // qDebug() << data->size();
    // for (size_t i = 1; i < data->size(); i ++)
    // {
    //     if ((*data)[i].timestemp - (*data)[i - 1].timestemp < DEFAULT_MIN_FUSION_ACCURACY )
    //     {
    //         //时间差过小，视为重复数据
    //         continue;
    //     }
    //     else
    //     {
    //         // qDebug() <<
    //         //插一帧
    //         message::msg::ImuDataFrame insertFrame;
    //         insertFrame.timestemp = ((*data)[i].timestemp + (*data)[i - 1].timestemp) / 2;
    //         insertFrame.quaternion.quaternion_0 = ((*data)[i].quaternion.quaternion_0 + (*data)[i - 1].quaternion.quaternion_0) / 2;
    //         insertFrame.quaternion.quaternion_1 = ((*data)[i].quaternion.quaternion_1 + (*data)[i - 1].quaternion.quaternion_1) / 2;
    //         insertFrame.quaternion.quaternion_2 = ((*data)[i].quaternion.quaternion_2 + (*data)[i - 1].quaternion.quaternion_2) / 2;
    //         insertFrame.quaternion.quaternion_3 = ((*data)[i].quaternion.quaternion_3 + (*data)[i - 1].quaternion.quaternion_3) / 2;

    //         lidarImuDataBuffer.push_back(insertFrame);
    //         lidarImuDataBuffer.push_back((*data)[i]);
    //     }
    // }
    // for (auto &i : _newData->data)
    // {
    //     if (i.angular_velocity.angular_velocity_z < 0)
    //     {
    //         i.timestemp -= 100000000;
    //     }
    //     else
    //     {
    //         i.timestemp += 100000000;
    //     }
    // }
    lidarImuDataBuffer.insert(lidarImuDataBuffer.end(), _newData->data.begin(), _newData->data.end());
    return true;
}

bool PointCloudDataManager::addEncoderData(message::msg::CarEncoderData::SharedPtr &_newData)
{
    // qDebug() << _newData->encoder1 << " " << _newData->encoder2;
    encoderDataBuffer.push_back(*_newData);
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
    // updateFlag |= handlePointsColor();
    if (updateFlag)
    {
        emit updateGraph();
    }
}

bool PointCloudDataManager::fuseData()
{
    // size_t minLen = std::min(pointsBuffer.size(), imuDataBuffer.size());
    // if (minLen == 0 )
    // {
    //     return false;
    // }
    bool updateFlag = false;

    updateFlag |= fusePositionData();
    updateFlag |= fuseLidarData();
    return updateFlag;

    // size_t count = 0 ;
    // float ang2radCoe = M_PI / 180.0f;
    // size_t imuIndex = 0;
    // size_t pointsIndex = 0;
    // size_t firstDataIndex = data.size() - 1;
    // //由于imu数据密度较低，此处作为缓存优化性能
    // size_t lastImuIndex = 0;
    // double quaternion0 = imuDataBuffer[lastImuIndex].quaternion.quaternion_0;
    // double quaternion1 = imuDataBuffer[lastImuIndex].quaternion.quaternion_1;
    // double quaternion2 = imuDataBuffer[lastImuIndex].quaternion.quaternion_2;
    // double quaternion3 = imuDataBuffer[lastImuIndex].quaternion.quaternion_3;
    // Eigen::Quaterniond quaternion = Eigen::Quaterniond{quaternion0, quaternion1, quaternion2, quaternion3};
    // float red = RGBNormalized(redOld), green = RGBNormalized(greenOld), blue = RGBNormalized(blueOld);
    // while (imuIndex  < imuDataBuffer.size() && pointsIndex < pointsBuffer.size())
    // {
    //     // RCLCPP_INFO(rclcpp::get_logger("main"), "1");
    //     if (pointsBuffer[pointsIndex].timestemp - imuDataBuffer[imuIndex].timestemp > fusionAccuracy)
    //     {
    //         imuIndex ++;
    //     }
    //     else if (abs(pointsBuffer[pointsIndex].timestemp - imuDataBuffer[imuIndex].timestemp) < fusionAccuracy)
    //     {
    //         if (pointsBuffer[pointsIndex].distance == 0 )
    //         {
    //             //距离为0视为无效点
    //             pointsIndex++;
    //             continue;
    //         }
    //         if (lastImuIndex != imuIndex)
    //         {
    //             //如果index相同则不再重复计算
    //             quaternion0 = imuDataBuffer[lastImuIndex].quaternion.quaternion_0;
    //             quaternion1 = imuDataBuffer[lastImuIndex].quaternion.quaternion_1;
    //             quaternion2 = imuDataBuffer[lastImuIndex].quaternion.quaternion_2;
    //             quaternion3 = imuDataBuffer[lastImuIndex].quaternion.quaternion_3;
    //             quaternion = Eigen::Quaterniond{quaternion0, quaternion1, quaternion2, quaternion3};
    //             //使用自定义精度
    //             if (enableCustomAccuracy)
    //             {
    //                 // RCLCPP_INFO(rclcpp::get_logger("main"), "%ld", imuDataBuffer[imuIndex].timestemp - imuDataBuffer[lastImuIndex].timestemp);
    //                 long long imuDataTimeInterval = imuDataBuffer[imuIndex].timestemp - imuDataBuffer[lastImuIndex].timestemp;
    //                 fusionAccuracy = std::min(imuDataTimeInterval,  15000000ll);
    //             }
    //             else
    //             {
    //                 fusionAccuracy = DEFAULT_FUSION_ACCURACY;
    //             }
    //             lastImuIndex = imuIndex;
    //         }
    //         // RCLCPP_INFO(rclcpp::get_logger("main"), "%ld", fusionAccuracy);
    //         double angle_rad =  pointsBuffer[pointsIndex].angle  * ang2radCoe  + M_PI_2;
    //         // 构建局部坐标向量
    //         Eigen::Vector3d local_vector( pointsBuffer[pointsIndex].distance * std::sin(angle_rad), pointsBuffer[pointsIndex].distance * std::cos(angle_rad), 0);
    //         Eigen::Vector3d worldVector = quaternion * local_vector;
    //         PointCloudVertex tempPoint =
    //         {
    //             static_cast<float>(worldVector.x()), static_cast<float>(worldVector.y()), static_cast<float>(worldVector.z()),
    //             red, green, blue
    //         };
    //         data.push_back(tempPoint);

    //         // RCLCPP_INFO(rclcpp::get_logger("main"), "%lf, %lf, %lf", imuDataBuffer[imuIndex].angular.roll, imuDataBuffer[imuIndex].angular.pitch,
    //         //             imuDataBuffer[imuIndex].angular.yaw);
    //         // RCLCPP_INFO(rclcpp::get_logger("main"), "%lld, %lld", imuDataBuffer[imuIndex].timestemp, pointsBuffer[pointsIndex].timestemp);
    //         count++;
    //         pointsIndex++;
    //     }
    //     else
    //     {
    //         pointsIndex++;
    //     }
    // }
    // RCLCPP_INFO(rclcpp::get_logger("main"), "%ld", count);
    // batches.push_back(BatchFrame{count, pointsBuffer[0].timestemp, firstDataIndex});

    // pointsBuffer.erase(pointsBuffer.begin(), pointsBuffer.begin() +  pointsIndex);
    // imuDataBuffer.erase(imuDataBuffer.begin(), imuDataBuffer.begin() + imuIndex);
}

bool PointCloudDataManager::fusePositionData()
{
    //融合imu和encoder数据

    // qDebug() << imuDataBuffer.size() << " " << encoderDataBuffer.size();
    if (encoderDataBuffer.size() > 1000)
    {
        encoderDataBuffer.clear();
    }
    return false;
}

bool PointCloudDataManager::fuseLidarData()
{
    //融合position和lidar、lidarIMU数据
    //以lidarImuDataBuffer为主，pointsBuffer配合融合
    // qDebug() << pointsBuffer.size() << " " << lidarImuDataBuffer.size();
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

    size_t count = 0 ;
    float ang2radCoe = M_PI / 180.0f;
    size_t imuIndex = 0;
    size_t pointsIndex = 0;
    size_t firstDataIndex = data.size() - 1;
    //由于imu数据密度较低，此处作为缓存优化性能
    size_t lastImuIndex = 0;
    // double quaternion0 = lidarImuDataBuffer[lastImuIndex].quaternion.quaternion_0;
    // double quaternion1 = lidarImuDataBuffer[lastImuIndex].quaternion.quaternion_1;
    // double quaternion2 = lidarImuDataBuffer[lastImuIndex].quaternion.quaternion_2;
    // double quaternion3 = lidarImuDataBuffer[lastImuIndex].quaternion.quaternion_3;
    // Eigen::Quaterniond quaternion = Eigen::Quaterniond{quaternion0, quaternion1, quaternion2, quaternion3};
    // quaternion.normalize();
    Eigen::Matrix3d Rx = Eigen::AngleAxisd(-lidarImuDataBuffer[imuIndex].angular.pitch * ang2radCoe, Eigen::Vector3d::UnitX()).matrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(lidarImuDataBuffer[imuIndex].angular.yaw * ang2radCoe, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(-lidarImuDataBuffer[imuIndex].angular.roll * ang2radCoe, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d R = Rz * Ry * Rx;
    float red = RGBNormalized(redOld), green = RGBNormalized(greenOld), blue = RGBNormalized(blueOld);
    while (imuIndex  < lidarImuDataBuffer.size() && pointsIndex < pointsBuffer.size())
    {
        if (lastImuIndex != imuIndex)
        {
            //如果index相同则不再重复计算
            Rx = Eigen::AngleAxisd(-lidarImuDataBuffer[imuIndex].angular.pitch * ang2radCoe, Eigen::Vector3d::UnitX()).matrix();
            Ry = Eigen::AngleAxisd(lidarImuDataBuffer[imuIndex].angular.yaw * ang2radCoe, Eigen::Vector3d::UnitY()).matrix();
            Rz = Eigen::AngleAxisd(-lidarImuDataBuffer[imuIndex].angular.roll * ang2radCoe, Eigen::Vector3d::UnitZ()).matrix();
            R = Rx * Ry * Rz;
            if (lidarImuDataBuffer[imuIndex].angular_velocity.angular_velocity_y > -10)
            {
                lastImuIndex = imuIndex;
                imuIndex ++;
                continue;
            }

            //启用自动精度
            if (enableAutoAccuracy)
            {
                long long imuDataTimeInterval = lidarImuDataBuffer[imuIndex].timestemp - lidarImuDataBuffer[lastImuIndex].timestemp;
                fusionAccuracy = std::min(imuDataTimeInterval / 2,  DEFAULT_FUSION_ACCURACY);
                if (fusionAccuracy < DEFAULT_MIN_FUSION_ACCURACY)
                {
                    //可能是重复数据，排除
                    lastImuIndex = imuIndex;
                    imuIndex ++;
                    continue;
                }
            }
            else
            {
                fusionAccuracy = DEFAULT_FUSION_ACCURACY;
            }
            lastImuIndex = imuIndex;
        }


        if (pointsBuffer[pointsIndex].timestemp - lidarImuDataBuffer[imuIndex].timestemp > fusionAccuracy)
        {
            imuIndex ++;
        }
        else if (abs(pointsBuffer[pointsIndex].timestemp - lidarImuDataBuffer[imuIndex].timestemp) < fusionAccuracy)
        {
            if (pointsBuffer[pointsIndex].distance == 0 )
            {
                //距离为0视为无效点
                pointsIndex++;
                continue;
            }

            // RCLCPP_INFO(rclcpp::get_logger("main"), "%ld", fusionAccuracy);
            double angle_rad =  (pointsBuffer[pointsIndex].angle + 90 - 0.5 )  * ang2radCoe;
            Eigen::Vector3d local_vector( -pointsBuffer[pointsIndex].distance * std::sin(angle_rad),   pointsBuffer[pointsIndex].distance * std::cos(angle_rad), 0);
            Eigen::Vector3d worldVector = R * local_vector;

            // RGB(255, 51, 0)
            // RGB(28, 126, 214
            double normalized_distance = (pointsBuffer[pointsIndex].distance - 0) / 3;
            red = RGBNormalized((1 - normalized_distance) * 255 + normalized_distance * 28);
            green = RGBNormalized((1 - normalized_distance) * 51 + normalized_distance * 126);
            blue = RGBNormalized((1 - normalized_distance) * 0 + normalized_distance * 214);
            PointCloudVertex tempPoint =
            {
                static_cast<float>(-worldVector.x()),  static_cast<float>(worldVector.z()), static_cast<float>(worldVector.y()),
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
    batches.push_back(BatchFrame{count, pointsBuffer[0].timestemp, firstDataIndex});

    pointsBuffer.erase(pointsBuffer.begin(), pointsBuffer.begin() +  pointsIndex);
    lidarImuDataBuffer.erase(lidarImuDataBuffer.begin(), lidarImuDataBuffer.begin() + imuIndex);

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
        float red = RGBNormalized(redOld - redDelta * offset);
        float green = RGBNormalized(greenOld - greenDelta * offset);
        float blue = RGBNormalized(blueOld - blueDelta * offset);
        for (size_t j = batches[i].firstDataIndex; j < batches[i].firstDataIndex + batches[i].size; j ++)
        {
            data[j].red = red;
            data[j].green = green;
            data[j].blue = blue;
        }
    }
    return true;
}

