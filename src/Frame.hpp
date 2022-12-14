#ifndef FRAME_H
#define FRAME_H

#include "Common.hpp"
#include "Feature.hpp"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/opencv.hpp"


namespace visual_frontend {
class Frame {

public: 
    // typedef std::shared_ptr<Frame> Ptr;
    Frame() { //cv::Mat& image
        // Extract the features 
        // feature_extractor->ExtractFeatures(image, features_);
    };

    // static Frame::Ptr CreateFrame() {

    // }

    // void SetFeatureExtractor(std::shared_ptr<FeatureExtractor> feature_extractor) {
    //     feature_extractor_ = feature_extractor;
    // }

    std::size_t GetFeatureSize() {
        return features_.size();
    }

    std::vector<std::shared_ptr<Feature>>& GetFeatures() {
        return features_;
    }

    cv::Mat& GetDescr() {
        return descr_;
    }
    
    void SetImage(cv::Mat& img) {
        img_ = img;
    }

    cv::Mat& GetImage() {
        return img_;
    }

    void SetTimestamp(double t) {
        timestamp_ = t;
    }

    double GetTimestamp() {
        return timestamp_;
    }

    std::vector<cv::DMatch>& GetMatches() {
        return matches_;
    }

    cv::Mat descr_;


private:
    std::vector<std::shared_ptr<Feature>> features_;

    // Feature match info against the previous frame.
    std::vector<cv::DMatch> matches_;
    cv::Mat img_;
    double timestamp_;
    // std::shared_ptr<FeatureExtractor> feature_extractor_;
    // Feature extractor class 


};
}

#endif