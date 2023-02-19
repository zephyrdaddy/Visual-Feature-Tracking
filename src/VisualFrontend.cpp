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

#define DEBUG 1

void print_localtime()
{
    std::time_t result = std::time(nullptr);
    std::cout << std::asctime(std::localtime(&result));
}

int main(int argc, char **argv)
{
    std::string image_dir;
    if (argc > 1)
    {
        image_dir = argv[1];
    }
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
        printf("Timestamp %f image name %s\n", timestamp, img_name.c_str());

        cv::Mat image = cv::imread(image_dir + "/" + img_name);
        // std::cout <<"Uhm\n" << image << std::endl;
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        // printf("Now process the image into a frame\n");

        // Create Frame factory method
        std::shared_ptr<visual_frontend::Frame> frame(new visual_frontend::Frame()); // gray
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
        if (DEBUG)
        {
            // cv::imshow("Image", image);

            // Feature Tracking image
            // cv::Mat display = image.clone();
            // int radius = 4;

            // for (std::size_t i = 0; i < corners.size(); i++) {
            //     cv::circle(display, corners[i], radius, cv::Scalar(255, 0, 0));
            // }

            // cv::imshow("feature", display);
            // cv::waitKey(0);
            int radius = 2;
            cv::Mat tracking_visualization = image.clone();
            // How to handle the const
            // I certainly don't want things to change here.

            // Visualize the tracking
            std::vector<std::shared_ptr<visual_frontend::Feature>> &features = frame->GetFeatures();
            for (std::shared_ptr<visual_frontend::Feature> &feature : features)
            {
                cv::circle(tracking_visualization, feature->GetPoint(), radius, cv::Scalar(255, 0, 0));

                std::shared_ptr<visual_frontend::Feature> curr_feature = feature;
                int track_cnt = 0;
                while (curr_feature->HasTrackedFeature())
                {
                    std::shared_ptr<visual_frontend::Feature> prev_feature = curr_feature->GetPrevFeature();
                    // if (prev_feature == nullptr) break;
                    // Draw the connection.

                    cv::line(tracking_visualization, cv::Point(prev_feature->GetPoint().x, prev_feature->GetPoint().y),
                             cv::Point(curr_feature->GetPoint().x, curr_feature->GetPoint().y), cv::Scalar(0, 255, 0));
                    // cv::circle(tracking_visualization, prev_feature->GetPoint(), radius, cv::Scalar(0, 0, 255));

                    curr_feature = prev_feature;
                    
                    track_cnt++;
                    if (track_cnt > 1) {
                        printf("Tracked %d\n", track_cnt);
                    }
                }
            }
                        cv::imshow("tracking_visualization", tracking_visualization);

            // for (std::size_t i = 0; i < curr_matches.size(); i++)
            // {
            //     cv::DMatch match = curr_matches[i];
            //     if (match.trainIdx == -1 || match.queryIdx == -1)
            //     {
            //         tracking_failure_flag = true;
            //         continue;
            //     }
            //     if (abs(features_to[match.queryIdx]->GetPoint().x - features_from[match.trainIdx]->GetPoint().x) > 5 || abs(features_to[match.queryIdx]->GetPoint().y - features_from[match.trainIdx]->GetPoint().y) > 5)
            //     {
            //         // printf("Check this idx %d\n", cnt_tmp);
            //         match.trainIdx = -1;
            //         match.queryIdx = -1;
            //         continue;
            //     }
            //     cv::circle(display, curr_features[i]->GetPoint(), radius, cv::Scalar(255, 0, 0));

            //     cv::line(display, cv::Point(features_to[match.queryIdx]->GetPoint().x, features_to[match.queryIdx]->GetPoint().y),
            //              cv::Point(features_from[match.trainIdx]->GetPoint().x, features_from[match.trainIdx]->GetPoint().y), cv::Scalar(0, 255, 0));

            //     for (std::size_t i = frame_list_.size() - 1; i > 1; i--)
            //     {
            //         std::vector<cv::DMatch> &next_matches = frame_list_[i]->GetMatches();
            //         if (match.trainIdx == -1)
            //             continue;

            //         // printf("Track idx check %d %d %d\n", next_matches.size(), match.queryIdx, match.trainIdx);

            //         cv::DMatch next_match = next_matches[match.trainIdx];
            //         match = next_match;

            //         std::vector<std::shared_ptr<Feature>> &tracked_features_to = frame_list_[i]->GetFeatures();
            //         std::vector<std::shared_ptr<Feature>> &tracked_features_from = frame_list_[i - 1]->GetFeatures();
            //         if (next_match.trainIdx == -1 || next_match.queryIdx == -1)
            //         {
            //             tracking_failure_flag = true;
            //             continue;
            //         }
            //         if (abs(tracked_features_to[next_match.queryIdx]->GetPoint().x - tracked_features_from[next_match.trainIdx]->GetPoint().x) > 5 || abs(tracked_features_to[next_match.queryIdx]->GetPoint().y - tracked_features_from[next_match.trainIdx]->GetPoint().y) > 5)
            //         {
            //             // printf("Check this idx %d\n", cnt_tmp);
            //             // next_match.trainIdx = -1;
            //             // next_match.queryIdx = -1;
            //             // printf("Hello\n");
            //             continue;
            //         }
            //         printf("size check %d %d %d\n", next_matches.size(), tracked_features_to.size(), tracked_features_from.size());

            //         if (next_match.queryIdx > tracked_features_to.size() || next_match.trainIdx > tracked_features_from.size())
            //         {
            //             std::cout << "Query " << next_match.queryIdx << " Train " << next_match.trainIdx << std::endl;
            //             std::cout << tracked_features_from.size() << " " << tracked_features_to.size() << std::endl;
            //             printf("%f %f\n", tracked_features_to[next_match.queryIdx]->GetPoint().x, tracked_features_to[next_match.queryIdx]->GetPoint().y);
            //             printf("%f %f\n", tracked_features_from[next_match.trainIdx]->GetPoint().x, tracked_features_from[next_match.trainIdx]->GetPoint().y);
            //         }

            //         cv::line(display, cv::Point(tracked_features_to[next_match.queryIdx]->GetPoint().x, tracked_features_to[next_match.queryIdx]->GetPoint().y),
            //                  cv::Point(tracked_features_from[next_match.trainIdx]->GetPoint().x, tracked_features_from[next_match.trainIdx]->GetPoint().y), cv::Scalar(0, 255, 0));
            //     }
            // }

            cv::waitKey(0);
        }
    }
    // Read each image.

    // 1. first step between the neighboring images

    print_localtime();

    cv::Mat src;

    return 0;
}
