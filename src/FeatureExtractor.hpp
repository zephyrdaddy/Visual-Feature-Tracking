#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H
#include "Common.hpp"
#include "Feature.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/features2d.hpp"
#include "Frame.hpp"

namespace visual_frontend {

class FeatureExtractor {
public:
    FeatureExtractor() {
        
    }

    ~FeatureExtractor() {

    }

    virtual void ExtractFeatures(const cv::Mat& img, std::shared_ptr<Frame>& frame) = 0;

private:

};
}

#endif