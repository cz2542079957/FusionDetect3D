#include "pointCloudDataManager.h"
#include "pointCloudWidget.h"

using namespace NSPointCloud;

PointCloudDataManager::PointCloudDataManager(int _maxCacheSize)
{
    maxCacheSize = _maxCacheSize;
}

bool PointCloudDataManager::addPoint(std::vector<float> _newData)
{
    unsigned long offset = 0;
    lastDataSize = data.size();
    float red = RGBNormalized(255), green = RGBNormalized(51), blu = RGBNormalized(0);
    while (offset + 3 <= _newData.size())
    {
        //添加这三个点
        data.insert(data.end(), _newData.begin() + offset, _newData.begin() + offset + 3);
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("lidarNodeSubscriber"), _newData[offset + 0] <<
        //                    " ," << _newData[offset  + 1] <<
        //                    " ," << _newData[offset  + 2] );
        // data.push_back(_newData[offset + 0]);
        // data.push_back(_newData[offset + 1]);
        // data.push_back(_newData[offset + 2]);
        //添加颜色
        // data.push_back(0.8);
        // data.push_back(0.8);
        // data.push_back(0.8);
        data.insert(data.end(), 3, 0.8);
        offset += 3;
    }
}

bool PointCloudDataManager::addPoint(float x, float y, float z)
{

    // data.push_back(x);
    // data.push_back(y);
    // data.push_back(z);
    // if (data.size() >= currentBufferSize)
    // {
    //     extendBuffer();
    // }
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

bool PointCloudDataManager::extendBuffer()
{
}
