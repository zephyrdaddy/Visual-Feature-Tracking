#ifndef TRACKER_H
#define TRACKER_H
#include "Common.hpp"
#include "Feature.hpp"
#include "Frame.hpp"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/features2d.hpp"
#include "opencv4/opencv2/xfeatures2d/nonfree.hpp"
#include "opencv4/opencv2/calib3d/calib3d.hpp"

namespace visual_frontend {

class Tracker {

public:
    Tracker();

    void TrackFrameByDescriptor(std::shared_ptr<Frame> frame);    

private:
    std::vector<int> tracked_cnt_;
    std::vector<int> ids_;

    // Descriptor Matcher 
    cv::Ptr<cv::DescriptorMatcher> descr_matcher_;

    std::unordered_map<int, int> feature_id_map_;

    std::vector<std::shared_ptr<Frame>> frame_list_;

    // Circular buffer for the frame list? 
    // For tracking yes
    // For the key frame list, just use the vector 
};
}

#endif

// Frame to frame tracker 
// Tracking method
// 1. Descriptor based 
// In this case, we can update the descriptor of the tracked feature selecting the most distinguished feature
// Each feature's descriptor is maintained in a vector. It's like an unique descriptor id for each vector.
// The order of the list indicates the id of each feature.
// i.e.) scenario
// If there is no data yet, extract features from the image. 
// Initialize the feature vector with the current image's features and descriptors
// 
// image 1 -> 



// Output -> tracked image visualized with tails 
// Pose of the images . Get it from the ground truth dataset.
// With the poses and the feature correspondences, triangulate the features. 
// Then, 3D cloud will be generated too. 

// Now, there are multiple tracking methods so to speak.

// Variation factors
// Image tracking methods
// 1. Different visual features
// 2. Descriptor based search
// 3. optical flow based
// 4. patch based rovio

// Static or Dynamic, so environmental factor
// 1. feature dessert
// 2. static
// 3. dynamic
// 4. close 
// 5. far 
// How to evaluate the tracking performance? 



// Ideal case to real case 