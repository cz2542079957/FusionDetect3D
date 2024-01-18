#include "pointCloudDataManager.h"
#include "rclcpp/rclcpp.hpp"

#include <QMatrix4x4>

using namespace NSPointCloud;

PointCloudDataManager::PointCloudDataManager(int _maxCacheSize)
{
    maxCacheSize = _maxCacheSize;
    data.reserve(_maxCacheSize);
}

bool PointCloudDataManager::addPoint(message::msg::LidarData::SharedPtr &_newData)
{
    // RCLCPP_INFO_STREAM( rclcpp::get_logger("lidarNodeSubscriber"),
    //                     _newData.size());
    //渲染之前的点的颜色
    // handlePointsColor();
    // unsigned long offset = 0;
    // lastDataSize = data.size();
    // float red = RGBNormalized(255), green = RGBNormalized(51), blue = RGBNormalized(0);
    // std::vector<float> temp =  {red, green, blue};
    // while (offset + 3 <= _newData.size())
    // {
    //     //添加这三个点
    //     data.insert(data.end(), _newData.begin() + offset, _newData.begin() + offset + 3);
    //     //添加颜色
    //     data.insert(data.end(), temp.begin(), temp.end());
    //     offset += 3;
    // }
    // //记录此批数据数量, *2 是因为还有颜色数值
    // batchSize.push_back(_newData.size() * 2);
    // return true;
    pointsBuffer.insert(pointsBuffer.end(), _newData->data.begin(), _newData->data.end());
    fuseData();
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

void PointCloudDataManager::setMaxCacheSize(unsigned long _maxCacheSize)
{
    maxCacheSize = _maxCacheSize;
}

std::vector<float> PointCloudDataManager::getData() const
{
    return data;
}

unsigned long PointCloudDataManager::getCurrentCacheSize() const
{
    return data.size();
}

unsigned long PointCloudDataManager::getLastDataSize() const
{
    return lastDataSize;
}

void PointCloudDataManager::handlePointsColor()
{
    //从上上批往前
    long preIndex = batchSize.size() - 1;
    long offset = 0;
    long i = data.size() - 1;
    long j;
    float redDelta = (redOld - redNew) / colorLevel, greenDelta = (greenOld - greenNew) / colorLevel, blueDelta = (blueOld - blueNew) / colorLevel;
    while (preIndex - offset >= 0 && offset < colorLevel)
    {
        unsigned long length = batchSize[preIndex - offset];
        j =  i - length + 1;
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("1"),  preIndex << ", " <<  offset << ", " <<   i << ", " << j << ", " << length);
        while (j < i)
        {
            data[j + 3] = NSPointCloud::RGBNormalized(redNew + redDelta * (offset + 1));
            data[j + 4] = NSPointCloud::RGBNormalized(greenNew + greenDelta * (offset + 1));
            data[j + 5] = NSPointCloud::RGBNormalized(blueNew + blueDelta * (offset + 1));
            j += 6;
        }
        i -= length;
        offset ++;
    }
}

int PointCloudDataManager::getColorLevel() const
{
    return colorLevel;
}

unsigned long PointCloudDataManager::getPointNeedPaintNumber()
{
    unsigned long length = 0;
    long last =  (long)batchSize.size() - 1 - colorLevel;
    for (long i = (long)batchSize.size() - 1;  i > last && i >= 0; i--)
    {
        length += batchSize[i];
    }
    return length;
}

void PointCloudDataManager::fuseData()
{
    size_t minLen = std::min(pointsBuffer.size(), imuDataBuffer.size());
    if (minLen == 0 )
    {
        return ;
    }

    size_t count = 0 ;
    float ang2radCoe = M_PI / 180.0f;
    size_t imuIndex = 0;
    size_t pointsIndex = 0;
    lastDataSize = data.size();
    float red = RGBNormalized(redNew), green = RGBNormalized(greenNew), blue = RGBNormalized(blueNew);
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
                pointsIndex++;
                continue;
            }
            // 将角度从度转换为弧度
            const double roll_rad = imuDataBuffer[imuIndex].angular.roll * ang2radCoe;
            const double pitch_rad = imuDataBuffer[imuIndex].angular.pitch * ang2radCoe;
            const double yaw_rad = imuDataBuffer[imuIndex].angular.yaw * ang2radCoe;
            const double angle_rad = pointsBuffer[pointsIndex].angle * ang2radCoe;

            // 构建3x3旋转矩阵（顺序为Z-Y-X）
            Eigen::Matrix3d rotation_matrix;
            rotation_matrix <<
                            std::cos(yaw_rad) * std::cos(pitch_rad), std::cos(yaw_rad) * std::sin(pitch_rad) * std::sin(roll_rad) - std::sin(yaw_rad) * std::cos(roll_rad),
                                std::cos(yaw_rad) * std::sin(pitch_rad) * std::cos(roll_rad) + std::sin(yaw_rad) * std::sin(roll_rad),
                                std::sin(yaw_rad) * std::cos(pitch_rad), std::sin(yaw_rad) * std::sin(pitch_rad) * std::sin(roll_rad) + std::cos(yaw_rad) * std::cos(roll_rad),
                                std::sin(yaw_rad) * std::sin(pitch_rad) * std::cos(roll_rad) - std::cos(yaw_rad) * std::sin(roll_rad),
                                -std::sin(pitch_rad), std::cos(pitch_rad) * std::sin(roll_rad), std::cos(pitch_rad) * std::cos(roll_rad);

            // 构建局部坐标向量（这里假设角度是绕Z轴顺时针）
            Eigen::Vector3d local_vector(pointsBuffer[pointsIndex].distance * std::sin(angle_rad), pointsBuffer[pointsIndex].distance * std::cos(angle_rad), 0);

            // 将局部坐标向量转换到全局坐标系
            Eigen::Vector3d global_vector = rotation_matrix * local_vector;


            // RCLCPP_INFO(rclcpp::get_logger("main"), "%lf, %lf, %lf", global_vector.x(), global_vector.y(), global_vector.z());
            data.push_back(global_vector.x());
            data.push_back(global_vector.y());
            data.push_back(global_vector.z());
            data.push_back(red);
            data.push_back(green);
            data.push_back(blue);
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
    batchSize.push_back(count * 6);

    pointsBuffer.erase(pointsBuffer.begin(), pointsBuffer.begin() +  pointsIndex);
    imuDataBuffer.erase(imuDataBuffer.begin(), imuDataBuffer.begin() + imuIndex);
}




