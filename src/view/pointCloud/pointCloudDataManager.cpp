#include "pointCloudDataManager.h"
#include "rclcpp/rclcpp.hpp"

using namespace NSPointCloud;

PointCloudDataManager::PointCloudDataManager(int _maxCacheSize)
{
    maxCacheSize = _maxCacheSize;
    data.reserve(_maxCacheSize);
}

bool PointCloudDataManager::addPoint(std::vector<float> _newData)
{
    //渲染之前的点的颜色
    handlePointsColor();
    unsigned long offset = 0;
    lastDataSize = data.size();
    float red = RGBNormalized(255), green = RGBNormalized(51), blue = RGBNormalized(0);
    std::vector<float> temp =  {red, green, blue};
    while (offset + 3 <= _newData.size())
    {
        //添加这三个点
        data.insert(data.end(), _newData.begin() + offset, _newData.begin() + offset + 3);
        //添加颜色
        data.insert(data.end(), temp.begin(), temp.end());
        offset += 3;
    }
    //记录此批数据数量, *2 是因为还有颜色数值
    batchSize.push_back(_newData.size() * 2);
    return true;
}

bool PointCloudDataManager::addPoint(float x, float y, float z)
{
    // TODO
    // data.push_back(x);
    // data.push_back(y);
    // data.push_back(z);
    // if (data.size() >= currentBufferSize)
    // {
    //     extendBuffer();
    // }
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
