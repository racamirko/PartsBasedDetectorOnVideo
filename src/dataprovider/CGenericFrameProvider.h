#ifndef CGENERICFRAMEPROVIDER_H
#define CGENERICFRAMEPROVIDER_H

#include <opencv2/core/core.hpp>

class CGenericFrameProvider
{
public:
    virtual double getFrameCount() = 0;
    virtual double getCurrentFrameNumber() = 0;
    virtual CGenericFrameProvider& operator>>(cv::Mat& _mat) = 0;
};

#endif // CGENERICFRAMEPROVIDER_H
