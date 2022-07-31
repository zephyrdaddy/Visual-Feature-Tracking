#include "GoodFeatureExtractor.hpp"

namespace visual_frontend {

// It assumes to get gray scale image.
void GoodFeatureExtractor::ExtractFeatures(const cv::Mat& img, std::shared_ptr<Frame>& frame) {
    // First extract features out of the image
    // std::vector<cv::Point2f> corners;
    // int max_corners = 3000;
    // double quality_level = 0.01;
    // double min_dist = 10;
    // int block_size = 3, grad_size = 3;
    // bool use_harris = false;
    // double k = 0.04;

    // cv::goodFeaturesToTrack(img, corners, max_corners, quality_level, min_dist, cv::Mat(), block_size, grad_size, use_harris, k);
    
    

    // features.resize(corners.size());
}


}
