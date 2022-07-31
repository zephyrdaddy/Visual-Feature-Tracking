#include <ctime>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include "opencv4/opencv2/highgui.hpp"
#include "opencv4/opencv2/features2d.hpp"
#include "opencv4/opencv2/imgproc.hpp"

#include "Frame.hpp"
#include "GoodFeatureExtractor.hpp"
#include "SiftFeatureExtractor.hpp"
#include "Tracker.hpp"

#define DEBUG 0

void print_localtime() {
    std::time_t result = std::time(nullptr);
    std::cout << std::asctime(std::localtime(&result));
}

int main(int argc, char** argv) {
    std::string image_dir;
    if (argc > 1) {
        image_dir = argv[1];
    }
    std::string filename = image_dir + "/rgb.txt";
    std::ifstream in(filename);

    // Type of feature to test
    // Good Feature to Track
    std::shared_ptr<visual_frontend::GoodFeatureExtractor> feature_extractor(new visual_frontend::GoodFeatureExtractor());
    std::shared_ptr<visual_frontend::SiftFeatureExtractor> surf_feature_extractor(new visual_frontend::SiftFeatureExtractor());
    std::shared_ptr<visual_frontend::Tracker> tracker(new visual_frontend::Tracker());


    for (std::string line ; std::getline(in, line); ) {
        if (line[0] == '#') continue;
        std::stringstream ss(line);
        double timestamp;
        std::string img_name;
        ss >> timestamp >> img_name;
        printf("Timestamp %f image name %s\n", timestamp, img_name.c_str());

        cv::Mat image = cv::imread(image_dir + "/" + img_name);
        // std::cout <<"Uhm\n" << image << std::endl;
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        // printf("Now process the image into a frame\n");

        // Create Frame factory method
        std::shared_ptr<visual_frontend::Frame> frame(new visual_frontend::Frame()); //gray
        frame->SetImage(image);
        frame->SetTimestamp(timestamp);
        surf_feature_extractor->ExtractFeatures(gray, frame);





        // Track the features with the Tracker object
        tracker->TrackFrameByDescriptor(frame);

        // Who owns the history of frames? 
        
        // Determine if it's a keyframe. 
        

        // Key1 -> F -> F -> Key2 
        // Feature condensing 
        // Match between key1 and key2 based on the match info. 

        // Track counter list for each frame

        // Publish the tracked features. 
        if (DEBUG) {
            cv::imshow("Image", image);

            // Feature Tracking image
            // cv::Mat display = image.clone();
            // int radius = 4;



            // for (std::size_t i = 0; i < corners.size(); i++) {
            //     cv::circle(display, corners[i], radius, cv::Scalar(255, 0, 0));
            // }



            // cv::imshow("feature", display);
            // cv::waitKey(0);

            cv::waitKey(1);
        }

    }
        // Read each image. 
        
        // 1. first step between the neighboring images 


    
    print_localtime();

    cv::Mat src;

      
    return 0;
}
