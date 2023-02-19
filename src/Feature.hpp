#ifndef FEATURE_H
#define FEATURE_H

#include "Common.hpp"
#include "opencv4/opencv2/opencv.hpp"

namespace visual_frontend
{

    class Feature
    {
    public:
        Feature() : prev_feature_(nullptr){};
        Feature(cv::Point2f &pt);
        void SetPoint(cv::Point2f &pt);

        cv::Point2f &GetPoint()
        {
            // std::cout << "Internal " << point2d_ << std::endl;
            return point2d_;
        }
        void SetTrackCount(std::size_t cnt)
        {
            tracked_cnt_ = cnt;
        }

        std::size_t GetTrackCount()
        {
            return tracked_cnt_;
        }

        void SetPrevFeature(std::shared_ptr<Feature> feature)
        {
            prev_feature_ = feature;
        }

        std::shared_ptr<Feature> &GetPrevFeature()
        {
            return prev_feature_;
        }

        bool HasTrackedFeature()
        {
            return prev_feature_ != nullptr;
        }
        // Factory function

    private:
        std::size_t tracked_cnt_ = 0;
        // 2D point coordinate
        cv::Point2f point2d_;
        // Pointer to the prev feature point

        // ID
        int64_t feature_id_;
        int64_t frame_id_;

        std::shared_ptr<Feature> prev_feature_;
    };
}

#endif
