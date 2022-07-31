#ifndef SIFT_FEATUER_EXTRACTOR_H
#define SIFT_FEATUER_EXTRACTOR_H

#include "Common.hpp"
#include "FeatureExtractor.hpp"
#include "opencv4/opencv2/features2d.hpp"
#include "opencv4/opencv2/xfeatures2d/nonfree.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "Frame.hpp"

namespace visual_frontend {
class SiftFeatureExtractor : public FeatureExtractor {
public:
SiftFeatureExtractor() : FeatureExtractor() {}
~SiftFeatureExtractor() {}

void ExtractFeatures(const cv::Mat& img, std::shared_ptr<Frame>& frame);

private:

};
}

#endif