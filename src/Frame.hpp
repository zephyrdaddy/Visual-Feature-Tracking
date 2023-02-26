#ifndef FRAME_H
#define FRAME_H

#include "Common.hpp"
#include "Feature.hpp"
#include "Calibration.hpp"
#include "Triangulator.hpp"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "Eigen/Eigen"

namespace visual_frontend
{
    class Frame
    {

    public:
        // typedef std::shared_ptr<Frame> Ptr;
        Frame(){
            // cv::Mat& image
            // Extract the features
            // feature_extractor->ExtractFeatures(image, features_);
        };

        Frame(cv::Mat &img, double timestamp,
              const std::shared_ptr<Calibration> &calibration) : img_(img), calibration_(calibration), timestamp_(timestamp),
                                                                 width_(img_.size().width), height_(img_.size().height)
        {

            K_ = Eigen::Matrix3d::Identity();
            K_(0, 0) = calibration_.get_fx();
            K_(1, 1) = calibration_.get_fy();
            K_(0, 2) = calibration_.get_cx();
            K_(1, 2) = calibration_.get_cy();
        };
        // static Frame::Ptr CreateFrame() {

        // }

        // void SetFeatureExtractor(std::shared_ptr<FeatureExtractor> feature_extractor) {
        //     feature_extractor_ = feature_extractor;
        // }

        std::size_t GetFeatureSize()
        {
            return features_.size();
        }

        std::vector<std::shared_ptr<Feature>> &GetFeatures()
        {
            return features_;
        }

        cv::Mat &GetDescr()
        {
            return descr_;
        }

        int GetHeight()
        {
            return height_;
        }

        int GetWidth()
        {
            return width_;
        }

        void SetImage(cv::Mat &img)
        {
            img_ = img;
        }

        cv::Mat &GetImage()
        {
            return img_;
        }

        void SetTimestamp(double t)
        {
            timestamp_ = t;
        }

        double GetTimestamp()
        {
            return timestamp_;
        }

        std::vector<cv::DMatch> &GetMatches()
        {
            return matches_;
        }

        Eigen::Matrix3d &GetIntrinsicMat()
        {
            return K_;
        }

        cv::Mat descr_;

    private:
        std::vector<std::shared_ptr<Feature>> features_;
        // Feature match info against the previous frame.
        std::vector<cv::DMatch> matches_;
        cv::Mat img_;
        const std::shared_ptr<Calibration> calibration_;
        Triangulator triangulator_;
        Eigen::Matrix3d K_;

        double timestamp_;
        int width_;
        int height_;
        uint32_t id_;
        // std::shared_ptr<FeatureExtractor> feature_extractor_;
        // Feature extractor class
    };
}

#endif