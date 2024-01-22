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
    for (size_t i = 0 ; i < _newData->data.size() ; i++)
    {
        float ang2radCoe = M_PI / 180.0f;
        static Eigen::Vector3d velocity(0, 0, 0);
        static Eigen::Vector3d position(0, 0, 0);
        // 根据设备姿态计算重力在传感器坐标系下的分量
        Eigen::Vector3d gravity_in_sensor_frame;
        gravity_in_sensor_frame << 0, 0, -9.81;  // 重力矢量（指向地球中心）
        double dt = 0.01;
        // 将欧拉角转换为旋转矩阵
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix =  Eigen::AngleAxisd(_newData->data[i].angular.roll * ang2radCoe, Eigen::Vector3d::UnitX())
                           *   Eigen::AngleAxisd(_newData->data[i].angular.pitch * ang2radCoe, Eigen::Vector3d::UnitY())
                           *  Eigen::AngleAxisd(_newData->data[i].angular.yaw * ang2radCoe, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d gravity_compensation = rotation_matrix * gravity_in_sensor_frame;
        gravity_compensation.z() = -gravity_compensation.z();

        Eigen::Vector3d acceleration = {_newData->data[i].acceleration.acceleration_x,
                                        _newData->data[i].acceleration.acceleration_y,
                                        _newData->data[i].acceleration.acceleration_z,
                                       }  ;
        // RCLCPP_INFO(rclcpp::get_logger("main"), "        angular:  %10f , %10f , %10f",  _newData->data[i].angular.roll, _newData->data[i].angular.pitch,
        //             _newData->data[i].angular.yaw);
        // RCLCPP_INFO(rclcpp::get_logger("main"), "        acceleration： %10f , %10f , %10f", acceleration.x(), acceleration.y(), acceleration.z());
        // RCLCPP_INFO(rclcpp::get_logger("main"), "gravity_compensation： %10f , %10f , %10f",
        // gravity_compensation.x(), gravity_compensation.y(), gravity_compensation.z());

        Eigen::Vector3d corrected_acceleration = acceleration - gravity_compensation;

        // RCLCPP_INFO(rclcpp::get_logger("main"), " corrected_acceleration： %10f , %10f , %10f",
        // corrected_acceleration.x(), corrected_acceleration.y(), corrected_acceleration.z());

        Eigen::Vector3d world_acceleration = rotation_matrix.transpose() * corrected_acceleration;

        // 积分得到速度和位移
        velocity += world_acceleration * dt;
        position += velocity * dt;

        // RCLCPP_INFO(rclcpp::get_logger("main"), "     world_acceleration： %10.4f , %10.4f , %10.4f",
        //             world_acceleration.x(), world_acceleration.y(), world_acceleration.z());
        // RCLCPP_INFO(rclcpp::get_logger("main"), "velocity : %10f , %10f , %10f", velocity.x(), velocity.y(), velocity.z());
        // RCLCPP_INFO(rclcpp::get_logger("main"), "Position : %10f , %10f , %10f", position.x(), position.y(), position.z());
        // std::cout << "velocity: (" << velocity.x() << ", " << velocity.y() << ", " << velocity.z() << ")" << std::endl;
        // std::cout << "Position: (" << position.x() << ", " << position.y() << ", " << position.z() << ")" << std::endl;
    }
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
    double roll_rad = imuDataBuffer[lastImuIndex].angular.roll * ang2radCoe;
    double pitch_rad = imuDataBuffer[lastImuIndex].angular.pitch * ang2radCoe;
    double yaw_rad = imuDataBuffer[lastImuIndex].angular.yaw * ang2radCoe;
    Eigen::Matrix3d rx = Eigen::Matrix3d(Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX()));
    Eigen::Matrix3d ry = Eigen::Matrix3d(Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()));
    Eigen::Matrix3d rz = Eigen::Matrix3d(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
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
                roll_rad = imuDataBuffer[imuIndex].angular.roll * ang2radCoe;
                pitch_rad = imuDataBuffer[imuIndex].angular.pitch * ang2radCoe;
                yaw_rad = imuDataBuffer[imuIndex].angular.yaw * ang2radCoe;
                rx = Eigen::Matrix3d(Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX()));
                ry = Eigen::Matrix3d(Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()));
                rz = Eigen::Matrix3d(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
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
            // 构建局部坐标向量（这里假设角度是绕Z轴顺时针）
            Eigen::Vector3d local_vector(pointsBuffer[pointsIndex].distance * std::sin(angle_rad), pointsBuffer[pointsIndex].distance * std::cos(angle_rad), 0);
            // 将局部坐标向量转换到全局坐标系
            Eigen::Vector3d global_vector = rz  * ry * rx * local_vector;
            // RCLCPP_INFO(rclcpp::get_logger("main"), "%lf, %lf, %lf", global_vector.x(), global_vector.y(), global_vector.z());
            PointCloudVertex tempPoint =
            {
                static_cast<float>(global_vector.x()), static_cast<float>(global_vector.y()), static_cast<float>(global_vector.z()),
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

