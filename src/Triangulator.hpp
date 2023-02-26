#pragma once

#include "Common.hpp"
#include "Feature.hpp"
#include "Frame.hpp"
#include "Eigen/Eigen"
#include "Eigen/SVD"
#include "opencv4/opencv2/calib3d/calib3d.hpp"
namespace visual_frontend
{

    // Basic function of this class is to triangulate a 3D point from features in two frames.
    // Key question is whether I should include the features for optimizing the 3D point from multiple frames in here
    // Or not.

    // Who owns this object?
    // 3D Constructor class -> Mapper or MapConstructor
    //
    class Triangulator
    {
    public:
        Triangulator();

        void Triangulate(const std::shared_ptr<Feature> &feature_1,
                         const std::shared_ptr<Feature> &feature_2,
                         const std::shared_ptr<Frame> &frame_1,
                         const std::shared_ptr<Frame> &frame_2,
                         const Eigen::Matrix3d &essential_mat,
                         Eigen::Vector3d &landmark);

    private:
    };

}