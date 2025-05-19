#ifndef STELLA_VSLAM_SOLVE_HOMOGRAPHY_SOLVER_H
#define STELLA_VSLAM_SOLVE_HOMOGRAPHY_SOLVER_H

#include "stella_vslam/camera/base.h"

#include <vector>

#include <opencv2/core.hpp>

#include <spdlog/spdlog.h>

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

namespace stella_vslam {
namespace solve {

class homography_solver {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Constructor
    homography_solver(const std::vector<cv::KeyPoint>& undist_keypts_1, const std::vector<cv::KeyPoint>& undist_keypts_2,
                      const std::vector<std::pair<int, int>>& matches_12, const float sigma);

    // FW: same constructor but with image cols and rows for graph-cut ransac
    homography_solver(const std::vector<cv::KeyPoint>& undist_keypts_1, const std::vector<cv::KeyPoint>& undist_keypts_2,
                        const std::vector<std::pair<int, int>>& matches_12, const float sigma,
                        const int img_cols, const int img_rows);

    //! Destructor
    virtual ~homography_solver() = default;

    //! Find the most reliable homography matrix via RASNAC
    void find_via_ransac(const unsigned int max_num_iter, const bool recompute = true);

    // // FW: the only difference is use only 4 points
    // void find_via_ransac_extended(const unsigned int max_num_iter, const bool recompute = true);

    // FW: find homography using graph-cut ransac: https://github.com/danini/graph-cut-ransac
    void find_via_graph_cut_ransac();

    //! Check if the solution is valid or not
    bool solution_is_valid() const {
        return solution_is_valid_;
    }

    //! Get the best score
    double get_best_score() const {
        return best_score_;
    }

    //! Get the most reliable essential matrix
    Mat33_t get_best_H_21() const {
        return best_H_21_;
    }

    //! Get the inlier matches
    std::vector<bool> get_inlier_matches() const {
        return is_inlier_match_;
    }

    //! Compute a homography matrix with 4-point algorithm
    static Mat33_t compute_H_21(const std::vector<cv::Point2f>& keypts_1, const std::vector<cv::Point2f>& keypts_2);

    //! Decompose a homography matrix to eight pairs of rotation and translation
    static bool decompose(const Mat33_t& H_21, const Mat33_t& cam_matrix_1, const Mat33_t& cam_matrix_2,
                          eigen_alloc_vector<Mat33_t>& init_rots, eigen_alloc_vector<Vec3_t>& init_transes, eigen_alloc_vector<Vec3_t>& init_normals);

private:
    //! Check inliers of homography transformation
    //! (Note: inlier flags are set to_inlier_match and a score is returned)
    float check_inliers(const Mat33_t& H_21, std::vector<bool>& is_inlier_match);

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
    //! most reliable homography matrix
    Mat33_t best_H_21_;
    //! inlier matches computed via RANSAC
    std::vector<bool> is_inlier_match_;

    // FW:
    int _source_image_cols;
    int _source_image_rows;
};

} // namespace solve
} // namespace stella_vslam

#endif // STELLA_VSLAM_SOLVE_HOMOGRAPHY_SOLVER_H
