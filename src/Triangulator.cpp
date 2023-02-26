#include "Triangulator.hpp"

namespace visual_frontend
{

    Triangulator::Triangulator()
    {
    }

    // TriangulatePoints()
    // With majority
    void triangulate_points(const std::vector<std::shared_ptr<Feature>> &feature_1,
                            const std::vector<std::shared_ptr<Feature>> &feature_2,
                            const std::shared_ptr<Frame> &frame_1,
                            const std::shared_ptr<Frame> &frame_2,
                            const Eigen::Matrix3d &essential_mat,
                            Eigen::Vector3d &landmark)
    {
    }

    void Triangulate(const std::shared_ptr<Feature> &feature_1,
                     const std::shared_ptr<Feature> &feature_2,
                     const std::shared_ptr<Frame> &frame_1,
                     const std::shared_ptr<Frame> &frame_2,
                     const Eigen::Matrix3d &essential_mat,
                     Eigen::Vector3d &landmark)
    {
        // Find the best fit.
        // x  = P X
        // x = K [R | t] X
        // x' = P'X
        // Make the matrix
        // 3 x 4
        Eigen::MatrixXd P1(3, 4), P2(3, 4);
        P1 = Eigen::MatrixXd::Identity(3, 4);
        P2 = Eigen::MatrixXd::Identity(3, 4);
        std::cout << "P1 \n"
                  << P1 << std::endl;
        Eigen::Matrix3d K1, K2;
        K1 = frame_1->GetIntrinsicMat(); // Eigen::Matrix3d::Identity();
        K2 = frame_2->GetIntrinsicMat(); // Eigen::Matrix3d::Identity();
        std::cout << "K1 \n"
                  << K1 << std::endl;

        // 3 x 4
        Eigen::MatrixXd T1(3, 4), T2(3, 4);
        T1 = K1 * P1;
        T2 = K2 * P2;

        // Decompose Essential Matrix.
        // Relative from the

        // Make the vectors

        // p1_1, p2_1, p3_1
        // p2_1, p2_2, p2_3
        //

        // x is known
        // P is known
    }

}