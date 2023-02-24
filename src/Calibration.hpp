#pragma once
#include "Common.hpp"

namespace visual_frontend
{

    class Calibration
    {
    public:
        Calibration(const std::vector<float> &intrinsic, const std::vector<float> &distortion) : intrinsic_(intrinsic), distortion_(distortion){};
        ~Calibration(){};

        // undistort all the points or just a single point?  
        // First deal with a single point 
        // Do we need both distorted and undistorted 2D feature points in the memory?
        // Is Eigen library data types good for image processing? 
        // If we are using opencv libraries, then cv Mat is probably better
        // 
        void undistort();

        void unproject();
        
        void project();



    private:
        std::vector<float> intrinsic_;
        std::vector<float> distortion_;
    };

}