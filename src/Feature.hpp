#ifndef FEATURE_H
#define FEATURE_H

#include "Common.hpp"
#include "opencv4/opencv2/opencv.hpp"

namespace visual_frontend {

class Feature {
public:
    Feature() {};

    void SetPoint(cv::Point2f& pt);
    
    cv::Point2f& GetPoint() {
        // std::cout << "Internal " << point2d_ << std::endl;
        return point2d_;
    }
    void SetTrackCount(std::size_t cnt) {
        tracked_cnt_ = cnt;
    }

    std::size_t GetTrackCount() {
        return tracked_cnt_;
    }
// Factory function 


private:
    std::size_t tracked_cnt_ = 0;
    // 2D point coordinate
    cv::Point2f point2d_;
    // Pointer to the prev feature point

    // ID 
    long int id_;


};
}

#endif
