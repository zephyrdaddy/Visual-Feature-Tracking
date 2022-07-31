#ifndef GOOD_FEATURE_EXTRACTOR_H
#define GOOD_FEATURE_EXTRACTOR_H
#include "FeatureExtractor.hpp"
#include "Frame.hpp"

namespace visual_frontend {
class GoodFeatureExtractor : public FeatureExtractor {
public:
    GoodFeatureExtractor() : FeatureExtractor() {}
    ~GoodFeatureExtractor() {}
    void ExtractFeatures(const cv::Mat& img, std::shared_ptr<Frame>& frame);



private:


};

}

#endif