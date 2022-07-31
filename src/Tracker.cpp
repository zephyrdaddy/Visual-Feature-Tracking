#include "Tracker.hpp"

namespace visual_frontend {

Tracker::Tracker() : descr_matcher_(nullptr) {

}

void Tracker::TrackFrameByDescriptor(std::shared_ptr<Frame> frame) {
    if (descr_matcher_ == nullptr) {
        descr_matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
    }

    // Maintain the list of frames.
    // TODO : Later change this vector to be a circular buffer for sliding windowing.
    frame_list_.push_back(frame);
    //Matching
    // Descriptor comparison
    if (frame_list_.size() == 1) {
        return;
    }
    std::shared_ptr<Frame> curr_frame = frame_list_[frame_list_.size() - 1];
    std::shared_ptr<Frame> prev_frame = frame_list_[frame_list_.size() - 2];

    // Match the 2D features by descriptors 
    std::vector<std::vector<cv::DMatch>> matches;
    // descr_matcher_->match(prev_frame->GetDescr(), curr_frame->GetDescr(), matches);
    descr_matcher_->knnMatch(curr_frame->GetDescr(), prev_frame->GetDescr(), matches, 2);
    
    // How to have one to one matching 
    std::vector<cv::DMatch>& curr_matches = curr_frame->GetMatches();

    std::vector<std::shared_ptr<Feature>> curr_features = curr_frame->GetFeatures();
    std::vector<std::shared_ptr<Feature>> prev_features = prev_frame->GetFeatures();
    std::cout << curr_features.size() << " " << prev_features.size() << std::endl;
    std::cout << curr_frame->GetDescr().size() << std::endl;
    // DMatch default constructor queryIdx(-1), trainIdx(-1), imgIdx(-1), distance(FLT_MAX)
    curr_matches.resize(matches.size());
    // If we do this, we loose the query idx
    for (std::size_t i = 0; i < matches.size(); i ++) {
        const float ratio = 0.8;
        
        if (matches[i][0].distance < ratio * matches[i][1].distance) {
            curr_matches[i] = matches[i][0];
            // std::cout << curr_matches[i].trainIdx << std::endl;
            // std::cout << prev_features.size() << std::endl;
            // curr_features[i]->SetTrackCount(prev_features[curr_matches[i].trainIdx]->GetTrackCount() + 1);

            // Update the track counter. 
        }
    }

    // Undistortion model based on the camera model

    std::vector<cv::Point2f> un_curr_pts, un_prev_pts;
    // int idx_valid = 0;
    for (int i = 0; i < curr_matches.size(); i++) {
        if (curr_matches[i].queryIdx == -1) continue;
        // printf("%d %d %d %d\n", curr_matches[i].queryIdx, curr_features.size(), curr_matches[i].trainIdx, prev_features.size());
        un_curr_pts.push_back(curr_features[curr_matches[i].queryIdx]->GetPoint());
        // cv::Point2f curr_point = curr_features[curr_matches[i].queryIdx]->GetPoint() ;
        // printf("%f %f test\n", curr_point.x, curr_point.y);
        // un_curr_pts.push_back(cv::Point2f(curr_point.x, curr_point.y));
        // std::cout <<"Curr " << curr_point << "Size " << un_curr_pts.size() << std::endl;

        // std::cout << "Test " << curr_features[curr_matches[i].queryIdx]->GetPoint() << std::endl;
        // std::cout << "Point " << un_curr_pts[idx_valid] << std::endl;
        // un_curr_pts[i]  = curr_point;
        // std::cout << "Again Point " << un_curr_pts[i] << std::endl;
        
        un_prev_pts.push_back(prev_features[curr_matches[i].trainIdx]->GetPoint());
        // std::cout << "Prev Test " << prev_features[curr_matches[i].trainIdx]->GetPoint() << std::endl;
        // std::cout << "prv Point " << un_prev_pts[idx_valid] << std::endl;
        // idx_valid++;
    }
    

    printf("Size %d %d\n", un_curr_pts.size(), un_prev_pts.size());

    // for (std::size_t i = 0; i < curr_features.size(); i++) {
    //     un_curr_pts[i] = curr_features[i]->GetPoint();
        
    // }

    // for (std::size_t i = 0; i < prev_features.size(); i++) {
    //     un_prev_pts[i] = prev_features[i]->GetPoint();
    // }
    //     for (unsigned int i = 0; i < cur_pts.size(); i++)
    //     {
    //         Eigen::Vector3d tmp_p;
    //         m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
    //         tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
    //         tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
    //         un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

    //         m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
    //         tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
    //         tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
    //         un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
    //     }
    double fx = 535.4;
    double fy = 539.2;
    double cx = 320.1;
    double cy = 247.6;

    //recovering the pose and the essential matrix
    // cv::Mat E, R, t, status;
    // float data[3][3] = {{fx, 0, cx}, {0, fy, cy}, {0, 0, 1}};
    // cv::Mat K = cv::Mat(3, 3, CV_32FC1, data);
    // E = cv::findEssentialMat(un_curr_pts, un_prev_pts, K, cv::FM_RANSAC, 0.999, 1.0, status);
    // cv::recoverPose(E, un_curr_pts, un_prev_pts, K, R, t, status);
    // // std::cout << status << std::endl;
    // std::cout << R << std::endl;
    // std::cout << t << std::endl;
    std::vector<uchar> status;
    cv::Mat F = cv::findFundamentalMat(un_curr_pts, un_prev_pts, cv::FM_RANSAC, 3.0, 0.99, status);
    double f11 = F.at<double>(0, 0);
    double f12 = F.at<double>(0, 1);
    double f13 = F.at<double>(0, 2);
    double f21 = F.at<double>(1, 0);
    double f22 = F.at<double>(1, 1);
    double f23 = F.at<double>(1, 2);
    double f31 = F.at<double>(2, 0);
    double f32 = F.at<double>(2, 1);
    double f33 = F.at<double>(2, 2);
    std::cout << F << std::endl;
    std::cout << f11 << std::endl;
    const float th = 3.841; // 3.841; // 1.9598 pixel distance

    // draw the left points corresponding epipolar lines in right image
    std::vector<cv::Vec3f> lines1;
    cv::computeCorrespondEpilines(cv::Mat(un_prev_pts), // image points
                                  2,                   // in image 1 (can also be 2)
                                  F,         // F matrix
                                  lines1);             // vector of epipolar lines
    std::vector<cv::Vec3f> lines2;
    cv::computeCorrespondEpilines(cv::Mat(un_curr_pts), // image points
                                  1,                   // in image 1 (can also be 2)
                                  F,         // F matrix
                                  lines2);             // vector of epipolar lines

    int cnt = 0;
    printf("Size Line1 %d\n", lines1.size());
    // for all epipolar lines
    for (auto it = lines1.begin(); it != lines1.end(); ++it)
    {

        float u1 = un_prev_pts[cnt].x;
        float v1 = un_prev_pts[cnt].y;
        
        float a1 = (*it)[0];
        float b1 = (*it)[1];
        float c1 = (*it)[2];
        float num1 = a1 * u1 + b1 * v1 + c1;
        float squareDist2 = num1 * num1 / (a1 * a1 + b1 * b1);
       

        if (squareDist2 > th ) {
            curr_matches[cnt].queryIdx = -1;
            curr_matches[cnt].trainIdx = -1;
            // printf("%f %f %f\n", (*it)[0], (*it)[1], (*it)[2]);
            // printf("Point %f %f\n", u1, v1);
            // printf("skip %f\n", squareDist2);
            // printf("%f\n", squareDist2);
            // cv::Mat epipolar2 = curr_frame->GetImage().clone();
            // printf("distance %f %f\n ", num1, squareDist2);

            // cv::line(epipolar2, cv::Point(0, -(*it)[2] / (*it)[1]),
            // cv::Point(epipolar2.cols, -((*it)[2] + (*it)[0] * epipolar2.cols) / (*it)[1]),
            // cv::Scalar(255, 255, 255));
            // cv::circle(epipolar2, un_prev_pts[cnt], 5, cv::Scalar(255, 0, 0), 3 );
            // cv::imshow("d", epipolar2);
            // cv::waitKey(0);
            cnt++;
            continue;
        }
        cnt++;

        // if (cnt > 675) {
        //     // draw the epipolar line between first and last column

        // }

    }
    cnt = 0;
    for (auto it = lines2.begin(); it != lines2.end(); ++it)
    {

        float u1 = un_curr_pts[cnt].x;
        float v1 = un_curr_pts[cnt].y;
        
        // printf("point %f %f\n", u1, v1);

        float a1 = (*it)[0];
        float b1 = (*it)[1];
        float c1 = (*it)[2];
        float num1 = a1 * u1 + b1 * v1 + c1;
        float squareDist2 = num1 * num1 / (a1 * a1 + b1 * b1);
       

        if (squareDist2 > th ) {
            curr_matches[cnt].queryIdx = -1;
            curr_matches[cnt].trainIdx = -1;
            // printf("%f %f %f\n", (*it)[0], (*it)[1], (*it)[2]);
            // printf("Point %f %f\n", u1, v1);
            // printf("skip %f\n", squareDist2);
            // printf("%f\n", squareDist2);
            // cv::Mat epipolar1 = curr_frame->GetImage().clone();
            // printf("distance %f %f\n ", num1, squareDist2);

            // cv::line(epipolar1, cv::Point(0, -(*it)[2] / (*it)[1]),
            // cv::Point(epipolar1.cols, -((*it)[2] + (*it)[0] * epipolar1.cols) / (*it)[1]),
            // cv::Scalar(255, 255, 255));
            // cv::circle(epipolar1, un_curr_pts[cnt], 5, cv::Scalar(255, 0, 0), 3 );
            // cv::imshow("epipolar2", epipolar1);
            // cv::waitKey(0);
            cnt++;
            continue;
        }
        cnt++;

        // if (cnt > 675) {
        //     // draw the epipolar line between first and last column

        // }

    }
    // cv::Mat test_match = curr_frame->GetImage().clone();

    // for (int i = 0; i < un_curr_pts.size(); i++) {
    //     cv::line(test_match, un_curr_pts[i], un_prev_pts[i], cv::Scalar(0, 255, 0));

    // }

    // cv::imshow("Test Match", test_match);
    // cv::waitKey(0);

    // // Reprojection error check
    // for (int i = 0; i < curr_matches.size(); i++) {
    //     if (curr_matches[i].queryIdx == -1) continue;
    //     // printf("%d %d %d %d\n", curr_matches[i].queryIdx, curr_features.size(), curr_matches[i].trainIdx, prev_features.size());
    //     float u1 = un_curr_pts[i].x;
    //     float v1 = un_curr_pts[i].y ; //.push_back(curr_features[curr_matches[i].queryIdx]->GetPoint());
    //     float u2 = un_prev_pts[i].x;
    //     float v2 = un_prev_pts[i].y ; //.push_back(prev_features[curr_matches[i].trainIdx]->GetPoint());

    //     float a2 = f11 * u1 + f12 * v1 + f13;
    //     float b2 = f21 * u1 + f22 * v1 + f23;
    //     float c2 = f31 * u1 + f32 * v1 + f33;
    //     printf("%f %f %f\n", a2, b2, c2);
    //     // Epipolar line to feature distance
    //     float num2 = a2 * u2 + b2 * v2 + c2;
    //     float squareDist1 = num2 * num2 / (a2 * a2 + b2 * b2);

    //     if (squareDist1 > th) {
    //         curr_matches[i].queryIdx = -1;
    //         curr_matches[i].trainIdx = -1;
    //         printf("skip %f %d\n", squareDist1, i);
    //         continue;
    //     }

    //     float a1 = u2 * f11 + v2 * f21 + f31;
    //     float b1 = u2 * f12 + v2 * f22 + f32;
    //     float c1 = u2 * f13 * v2 * f23 + f33;

    //     float num1 = a1 * u1 + b1 * v1 + c1;
    //     float squareDist2 = num1 * num1 / (a1 * a1 + b1 * b1);
    //     cv::Mat epipolar = curr_frame->GetImage().clone();
    //     printf("%f %f %f\n", a1, b1, c1);

    //     float y1 = -c1 / b1;
    //     float y2 = -(640 * a1 + c1) / b1;
    //     printf("%f %f\n", y1, y2);
    //     cv::line(epipolar, cv::Point(0, y1), 
    //                        cv::Point(640, y2), cv::Scalar(0, 255, 0));
    //     cv::imshow("Epipolar", epipolar);
    //     cv::waitKey(0);

    //     if (squareDist2 > th) {
    //         curr_matches[i].queryIdx = -1;
    //         curr_matches[i].trainIdx = -1;
    //         printf("skip %f\n", squareDist2);

    //         continue;
    //     }

    // }
    for (int i = 0; i < curr_matches.size(); i++) {
        if (curr_matches[i].queryIdx == -1) continue;

        curr_features[i]->SetTrackCount(prev_features[curr_matches[i].trainIdx]->GetTrackCount() + 1);

    }
    
    // Do another check using the fundamental matrix estimation.

    cv::Mat display = curr_frame->GetImage().clone();
    int radius = 4;


    // for (std::size_t i = 0; i < curr_features.size(); i++) {
        
    //     cv::circle(display, curr_features[i]->GetPoint(), radius, cv::Scalar(255, 0, 0));
    // }
    std::size_t check_idx = 0;
    if (frame_list_.size() > 10) {
        check_idx = frame_list_.size() - 10;
    }
    bool tracking_failure_flag = false;

        // for (std::size_t i = frame_list_.size() - 1; i > check_idx + 1; i--) { 
        //     std::vector<cv::DMatch>& next_matches = frame_list_[i]->GetMatches();
        //     std::vector<std::shared_ptr<Feature>>& features = frame_list_[i]->GetFeatures();
        //     std::vector<std::shared_ptr<Feature>>& prev = frame_list_[i - 1]->GetFeatures();
        //     printf("%d %d %d\n", next_matches.size(), features.size(), prev.size());
        // }
        // return;

    std::vector<std::shared_ptr<Feature>>& features_to = frame_list_[frame_list_.size() - 1]->GetFeatures();
    std::vector<std::shared_ptr<Feature>>& features_from = frame_list_[frame_list_.size() - 2]->GetFeatures();
    for (std::size_t i = 0; i < curr_matches.size(); i++) {
        cv::DMatch match = curr_matches[i];
        if (match.trainIdx == -1 || match.queryIdx == -1) {
            tracking_failure_flag = true;
            continue;
        }
        if (abs(features_to[match.queryIdx]->GetPoint().x - features_from[match.trainIdx]->GetPoint().x) > 5 || abs(features_to[match.queryIdx]->GetPoint().y - features_from[match.trainIdx]->GetPoint().y) > 5) {
            // printf("Check this idx %d\n", cnt_tmp);
            match.trainIdx = -1;
            match.queryIdx = -1;
            continue;
        }
        cv::circle(display, curr_features[i]->GetPoint(), radius, cv::Scalar(255, 0, 0));


        cv::line(display, cv::Point(features_to[match.queryIdx]->GetPoint().x, features_to[match.queryIdx]->GetPoint().y), 
                        cv::Point(features_from[match.trainIdx]->GetPoint().x, features_from[match.trainIdx]->GetPoint().y), cv::Scalar(0, 255, 0));

        for (std::size_t i = frame_list_.size() - 2; i > check_idx + 1; i--) { 
            std::vector<cv::DMatch>& next_matches = frame_list_[i]->GetMatches();
            if (match.trainIdx == -1) continue;

            // printf("Track idx check %d %d %d\n", next_matches.size(), match.queryIdx, match.trainIdx);

            cv::DMatch next_match = next_matches[match.trainIdx];
            match = next_match;

            std::vector<std::shared_ptr<Feature>>& tracked_features_to = frame_list_[i]->GetFeatures();
            std::vector<std::shared_ptr<Feature>>& tracked_features_from = frame_list_[i - 1]->GetFeatures();
            if (next_match.trainIdx == -1 || next_match.queryIdx == -1) {
                tracking_failure_flag = true;
                continue;
            }
            if (abs(tracked_features_to[next_match.queryIdx]->GetPoint().x - tracked_features_from[next_match.trainIdx]->GetPoint().x) > 5 || abs(tracked_features_to[next_match.queryIdx]->GetPoint().y - tracked_features_from[next_match.trainIdx]->GetPoint().y) > 5) {
                // printf("Check this idx %d\n", cnt_tmp);
                // next_match.trainIdx = -1;
                // next_match.queryIdx = -1;
                // printf("Hello\n");
                continue;
            }
        printf("size check %d %d %d\n", next_matches.size(), tracked_features_to.size(), tracked_features_from.size());

        if (next_match.queryIdx > tracked_features_to.size() || next_match.trainIdx > tracked_features_from.size()) {
            std::cout << "Query " << next_match.queryIdx << " Train " << next_match.trainIdx << std::endl;
            std::cout << tracked_features_from.size() << " " << tracked_features_to.size() << std::endl;
            printf("%f %f\n", tracked_features_to[next_match.queryIdx]->GetPoint().x, tracked_features_to[next_match.queryIdx]->GetPoint().y);
            printf("%f %f\n", tracked_features_from[next_match.trainIdx]->GetPoint().x, tracked_features_from[next_match.trainIdx]->GetPoint().y);
        }
        
        cv::line(display, cv::Point(tracked_features_to[next_match.queryIdx]->GetPoint().x, tracked_features_to[next_match.queryIdx]->GetPoint().y), 
                        cv::Point(tracked_features_from[next_match.trainIdx]->GetPoint().x, tracked_features_from[next_match.trainIdx]->GetPoint().y), cv::Scalar(0, 255, 0));

        }
    }

    // for (std::size_t i = frame_list_.size() - 1; i > check_idx; i--) {
    //     // If tracking was successful.
    //     std::vector<cv::DMatch>& tracking_matches = frame_list_[i]->GetMatches();

    //     std::vector<std::shared_ptr<Feature>> features_to = frame_list_[i]->GetFeatures();
    //     std::vector<std::shared_ptr<Feature>> features_from = frame_list_[i - 1]->GetFeatures();
    //     // for (auto match : tracking_matches) {
    //     int cnt_tmp = 0;
    //     for (std::size_t j = 0; j < tracking_matches.size(); j++) {
    //         auto& match = tracking_matches[j];
    //         // std::cout << match.trainIdx << std::endl;
    //         // printf("Idx %d\n", i);
    //         if (match.trainIdx == -1 || match.queryIdx == -1) {
    //             tracking_failure_flag = true;
    //             continue;
    //         }
    //         if (abs(features_to[match.queryIdx]->GetPoint().x - features_from[match.trainIdx]->GetPoint().x) > 5 || abs(features_to[match.queryIdx]->GetPoint().y - features_from[match.trainIdx]->GetPoint().y) > 5) {
    //             // printf("Check this idx %d\n", cnt_tmp);
    //             match.trainIdx = -1;
    //             match.queryIdx = -1;
    //             continue;
    //         }
    //         cnt_tmp++;
    //         // if (abs(features_to[match.queryIdx]->GetPoint().x - features_from[match.trainIdx]->GetPoint().x) < 5 && abs(features_to[match.queryIdx]->GetPoint().y - features_from[match.trainIdx]->GetPoint().y) < 5) {
    //         //     // printf("Check this idx %d\n", cnt_tmp);
    //         //     continue;
    //         //     // continue;
    //         // }
    //         // float features_to[match.queryIdx]->GetPoint().x, features_to[match.queryIdx]->GetPoint().y;
            
    //         cv::line(display, cv::Point(features_to[match.queryIdx]->GetPoint().x, features_to[match.queryIdx]->GetPoint().y), 
    //                        cv::Point(features_from[match.trainIdx]->GetPoint().x, features_from[match.trainIdx]->GetPoint().y), cv::Scalar(0, 255, 0));
    //     }
    //     // Don't draw the lines for every feature.
    //     // Only draw the lines for the feature currently being tracked.

    // }

    cv::imshow("Tracking", display);
    cv::waitKey(1);


}   

}



// void symmetryTest(const std::vector<cv::DMatch> &matches1,const std::vector<cv::DMatch> &matches2,std::vector<cv::DMatch>& symMatches)
// {
//     symMatches.clear();
//     for (vector<DMatch>::const_iterator matchIterator1= matches1.begin();matchIterator1!= matches1.end(); ++matchIterator1)
//     {
//         for (vector<DMatch>::const_iterator matchIterator2= matches2.begin();matchIterator2!= matches2.end();++matchIterator2)
//         {
//             if ((*matchIterator1).queryIdx ==(*matchIterator2).trainIdx &&(*matchIterator2).queryIdx ==(*matchIterator1).trainIdx)
//             {
//                 symMatches.push_back(DMatch((*matchIterator1).queryIdx,(*matchIterator1).trainIdx,(*matchIterator1).distance));
//                 break;
//             }
//         }
//     }
// }


// void ransacTest(const std::vector<cv::DMatch> matches,const std::vector<cv::KeyPoint>&keypoints1,const std::vector<cv::KeyPoint>& keypoints2,std::vector<cv::DMatch>& goodMatches,double distance,double confidence,double minInlierRatio)
// {
//     goodMatches.clear();
//     // Convert keypoints into Point2f
//     std::vector<cv::Point2f> points1, points2;
//     for (std::vector<cv::DMatch>::const_iterator it= matches.begin();it!= matches.end(); ++it)
//     {
//         // Get the position of left keypoints
//         float x= keypoints1[it->queryIdx].pt.x;
//         float y= keypoints1[it->queryIdx].pt.y;
//         points1.push_back(cv::Point2f(x,y));
//         // Get the position of right keypoints
//         x= keypoints2[it->trainIdx].pt.x;
//         y= keypoints2[it->trainIdx].pt.y;
//         points2.push_back(cv::Point2f(x,y));
//     }
//     // Compute F matrix using RANSAC
//     std::vector<uchar> inliers(points1.size(),0);
//     cv::Mat fundemental= cv::findFundamentalMat(cv::Mat(points1),cv::Mat(points2),inliers,CV_FM_RANSAC,distance,confidence); // confidence probability
//     // extract the surviving (inliers) matches
//     std::vector<uchar>::const_iterator
//     itIn= inliers.begin();
//     std::vector<cv::DMatch>::const_iterator
//     itM= matches.begin();
//     // for all matches
//     for ( ;itIn!= inliers.end(); ++itIn, ++itM)
//     {
//         if (*itIn)
//         { // it is a valid match
//             goodMatches.push_back(*itM);
//         }
//     }
// }

// https://stackoverflow.com/questions/17967950/improve-matching-of-feature-points-with-opencv