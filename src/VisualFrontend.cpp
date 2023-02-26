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
#include "Calibration.hpp"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

#define DEBUG 1

void print_localtime()
{
    std::time_t result = std::time(nullptr);
    std::cout << std::asctime(std::localtime(&result));
}

int main(int argc, char **argv)
{
    std::string config_fname;
    if (argc > 1)
    {
        config_fname = argv[1];
    }

    // Read the json config file.
    FILE *fp = fopen(config_fname.c_str(), "rt");
    if (!fp)
    {
        printf("<4>Could not open the config file %s\n", config_fname.c_str());
        return -1;
    }

    char read_buffer[6000];
    rapidjson::FileReadStream is(fp, read_buffer, sizeof(read_buffer));
    rapidjson::Document json_cfg;
    json_cfg.ParseStream(is);

    std::string image_dir = json_cfg["image_dir"].GetString();
    rapidjson::Value& calibration_cfg = json_cfg["calibration"];
    rapidjson::Value& distortion_cfg = json_cfg["distortion"];
    // fx, fy, cx, cy
    std::vector<float> intrinsic(4, 0);
    std::vector<float> distortion(5, 0);
    intrinsic[0] = calibration_cfg["fx"].GetDouble();
    intrinsic[1] = calibration_cfg["fy"].GetDouble();
    intrinsic[2] = calibration_cfg["cx"].GetDouble();
    intrinsic[3] = calibration_cfg["cy"].GetDouble();
    
    distortion[0] = distortion_cfg["d0"].GetDouble();
    distortion[1] = distortion_cfg["d1"].GetDouble();
    distortion[2] = distortion_cfg["d2"].GetDouble();
    distortion[3] = distortion_cfg["d3"].GetDouble();
    distortion[4] = distortion_cfg["d4"].GetDouble();

    std::shared_ptr<visual_frontend::Calibration> calibration(new visual_frontend::Calibration(intrinsic, distortion));

    std::string filename = image_dir + "/rgb.txt";
    std::ifstream in(filename);

    // Type of feature to test
    // Good Feature to Track
    std::shared_ptr<visual_frontend::GoodFeatureExtractor> feature_extractor(new visual_frontend::GoodFeatureExtractor());
    std::shared_ptr<visual_frontend::SiftFeatureExtractor> surf_feature_extractor(new visual_frontend::SiftFeatureExtractor());
    std::shared_ptr<visual_frontend::Tracker> tracker(new visual_frontend::Tracker());

    for (std::string line; std::getline(in, line);)
    {
        if (line[0] == '#')
            continue;
        std::stringstream ss(line);
        double timestamp;
        std::string img_name;
        ss >> timestamp >> img_name;
        // printf("Timestamp %f image name %s\n", timestamp, img_name.c_str());

        cv::Mat image = cv::imread(image_dir + "/" + img_name);
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        // printf("Now process the image into a frame\n");

        // Create Frame factory method
        std::shared_ptr<visual_frontend::Frame> frame(new visual_frontend::Frame(image, timestamp, calibration)); // gray
        // std::shared_ptr<visual_frontend::Frame> frame(new visual_frontend::Frame()); // gray
        // frame->SetImage(image);
        // frame->SetTimestamp(timestamp);
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
        if (DEBUG)
        {
            // cv::imshow("Image", image);

           
            int radius = 2;
            cv::Mat tracking_visualization = image.clone();
            // How to handle the const
            // I certainly don't want things to change here.

            // Visualize the tracking
            std::vector<std::shared_ptr<visual_frontend::Feature>> &features = frame->GetFeatures();
            for (std::shared_ptr<visual_frontend::Feature> &feature : features)
            {
                cv::circle(tracking_visualization, feature->GetPoint(), radius, cv::Scalar(255, 0, 0));
                int track_cnt = 0;
                std::shared_ptr<visual_frontend::Feature> curr_feature = feature;
                while (curr_feature->HasTrackedFeature() && track_cnt < 10)
                {
                    std::shared_ptr<visual_frontend::Feature> prev_feature = curr_feature->GetPrevFeature();
                    // if (prev_feature == nullptr) break;
                    // Draw the connection.

                    cv::line(tracking_visualization, cv::Point(prev_feature->GetPoint().x, prev_feature->GetPoint().y),
                             cv::Point(curr_feature->GetPoint().x, curr_feature->GetPoint().y), cv::Scalar(0, 255, 0));
                    // cv::circle(tracking_visualization, prev_feature->GetPoint(), radius, cv::Scalar(0, 0, 255));
                    
                    curr_feature = prev_feature;
                    track_cnt++;
                }
            }
            cv::namedWindow("tracking_visualization", cv::WINDOW_GUI_EXPANDED);
            // cv::resizeWindow("tracking_visualization", )
            cv::imshow("tracking_visualization", tracking_visualization);


            cv::waitKey(10);
        }
    }
    // Read each image.

    // 1. first step between the neighboring images

    print_localtime();

    cv::Mat src;

    return 0;
}
