#ifndef STELLA_VSLAM_SOLVE_FUNDAMENTAL_SOLVER_H
#define STELLA_VSLAM_SOLVE_FUNDAMENTAL_SOLVER_H

#include "stella_vslam/type.h"

#include <vector>

#include <opencv2/core.hpp>

//! extended lib of graph-cut ransac
#include <thread>
#include <opencv2/calib3d.hpp>
#include <Eigen/Eigen>
#include "stella_vslam/solve/GCRANSAC/GCRANSAC.h"
#include "stella_vslam/solve/GCRANSAC/flann_neighborhood_graph.h"
#include "stella_vslam/solve/GCRANSAC/grid_neighborhood_graph.h"
#include "stella_vslam/solve/GCRANSAC/uniform_sampler.h"
#include "stella_vslam/solve/GCRANSAC/prosac_sampler.h"
#include "stella_vslam/solve/GCRANSAC/progressive_napsac_sampler.h"
#include "stella_vslam/solve/GCRANSAC/preemption_sprt.h"
#include "stella_vslam/solve/GCRANSAC/types.h"
#include "stella_vslam/solve/GCRANSAC/statistics.h"
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
//!

namespace stella_vslam
{
namespace solve
{

class fundamental_solver
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Constructor
    fundamental_solver(const std::vector<cv::KeyPoint>& undist_keypts_1, const std::vector<cv::KeyPoint>& undist_keypts_2,
                       const std::vector<std::pair<int, int>>& matches_12, const float sigma);

    // FW: same constructor but with image cols and rows for graph-cut ransac
    fundamental_solver(const std::vector<cv::KeyPoint>& undist_keypts_1, const std::vector<cv::KeyPoint>& undist_keypts_2,
                        const std::vector<std::pair<int, int>>& matches_12, const float sigma,
                        const int img_cols, const int img_rows);

    //! Destructor
    virtual ~fundamental_solver() = default;

    //! Find the most reliable fundamental matrix via RASNAC
    void find_via_ransac(const unsigned int max_num_iter, const bool recompute = true);

    // FW: find fundamental matrix using graph-cut ransac: https://github.com/danini/graph-cut-ransac
    void find_via_graph_cut_ransac();

    //! Check if the solution is valid or not
    bool solution_is_valid() const {
        return solution_is_valid_;
    }

    //! Get the best score
    double get_best_score() const {
        return best_score_;
    }

    //! Get the most reliable fundamental matrix
    Mat33_t get_best_F_21() const {
        return best_F_21_;
    }

    //! Get the inlier matches
    std::vector<bool> get_inlier_matches() const {
        return is_inlier_match_;
    }

    //! Compute a fundamental matrix with 8-point algorithm
    static Mat33_t compute_F_21(const std::vector<cv::Point2f>& keypts_1, const std::vector<cv::Point2f>& keypts_2);

    //! Decompose a fundamental matrix to four pairs of rotation and translation
    static bool decompose(const Mat33_t& F_21, const Mat33_t& cam_matrix_1, const Mat33_t& cam_matrix_2,
                          eigen_alloc_vector<Mat33_t>& init_rots, eigen_alloc_vector<Vec3_t>& init_transes);

    //! Create a fundamental matrix from camera poses
    static Mat33_t create_F_21(const Mat33_t& rot_1w, const Vec3_t& trans_1w, const Mat33_t& cam_matrix_1,
                               const Mat33_t& rot_2w, const Vec3_t& trans_2w, const Mat33_t& cam_matrix_2);

private:
    //! Check inliers of the epipolar constraint
    //! (Note: inlier flags are set to `inlier_match` and a score is returned)
    float check_inliers(const Mat33_t& F_21, std::vector<bool>& is_inlier_match);

    //! undistorted keypoints of shot 1
    const std::vector<cv::KeyPoint> undist_keypts_1_;
    //! undistorted keypoints of shot 2
    const std::vector<cv::KeyPoint> undist_keypts_2_;
    //! matched indices between shots 1 and 2
    const std::vector<std::pair<int, int>>& matches_12_;
    //! standard deviation of keypoint detection error
    const float sigma_;

    //! solution is valid or not
    bool solution_is_valid_ = false;
    //! best score of RANSAC
    double best_score_ = 0.0;
    //! most reliable fundamental matrix
    Mat33_t best_F_21_;
    //! inlier matches computed via RANSAC
    std::vector<bool> is_inlier_match_;

    // FW:
    int _source_image_cols;
    int _source_image_rows;
};

} // namespace solve
} // namespace stella_vslam
#endif // STELLA_VSLAM_SOLVE_FUNDAMENTAL_SOLVER_H
