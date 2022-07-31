#include "SiftFeatureExtractor.hpp"

// using namespace cv::xfeatures2d;

namespace visual_frontend {

void SiftFeatureExtractor::ExtractFeatures(const cv::Mat& img, std::shared_ptr<Frame>& frame) {
    // First extract features out of the image
    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create( minHessian );
    std::vector<cv::KeyPoint> keypoints;
    detector->detectAndCompute( img, cv::noArray(), keypoints, frame->GetDescr() );

    std::vector<std::shared_ptr<Feature>>& features = frame->GetFeatures();
    
    features.resize(keypoints.size());
    for (std::size_t i = 0; i < keypoints.size(); i++) {
        features[i].reset(new Feature());
        features[i]->SetPoint( keypoints[i].pt );
    }

    // printf("Descriptor size %ld\n", frame->GetDescr().size());
    // std::cout << "Hello " << frame->GetDescr().size() << std::endl;
    // detector->compute(img, keypoints, descr_);
    
    // cv::Mat desc qriptors;
    // detector->detectAndCompute( img, cv::noArray(), keypoints, descriptors );
    //-- Draw keypoints
    // cv::Mat img_keypoints;
    // cv::drawKeypoints( img, keypoints, img_keypoints );
    // cv::imshow("test", img_keypoints);
    // cv::waitKey(1);


    // features.resize(keypoints.size());
}

}
