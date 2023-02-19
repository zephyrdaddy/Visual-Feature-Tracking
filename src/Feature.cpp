#include "Feature.hpp"
namespace visual_frontend
{

    Feature::Feature(cv::Point2f &pt) : point2d_(pt), prev_feature_(nullptr)
    {
    }

    void Feature::SetPoint(cv::Point2f &pt)
    {
        point2d_ = pt;
    }

}