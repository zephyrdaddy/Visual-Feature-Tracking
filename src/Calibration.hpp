#pragma once
#include "Common.hpp"
#include "Eigen/Eigen"
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

        float get_fx() {
            return intrinsic_[0];
        }

        float get_fy() {
            return intrinsic_[1];
        }

        float get_cx() {
            return intrinsic_[2];
        }

        float get_cy() {
            return intrinsic_[3];
        }


    private:
        std::vector<float> intrinsic_;
        std::vector<float> distortion_;
    };

}