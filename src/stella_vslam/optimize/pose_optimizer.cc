#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/optimize/pose_optimizer.h"
#include "stella_vslam/optimize/g2o/se3/pose_opt_edge_wrapper.h"
#include "stella_vslam/util/converter.h"

#include <vector>
#include <mutex>

#include <Eigen/StdVector>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

namespace stella_vslam {
namespace optimize {

pose_optimizer::pose_optimizer(const unsigned int num_trials, const unsigned int num_each_iter)
    : num_trials_(num_trials), num_each_iter_(num_each_iter) {}

unsigned int pose_optimizer::optimize(data::frame& frm) const {
    // FW: the pose optimizer is the so called motion-only BA, happens right after a frame is tracked successfully
    //  Input: frame with 3D-2D point correspondences
    //  (the position of linked 3D point is considered as constant, while the pose of the frame is optimized to minimum re-projection error)

    // --------------------------------------------------------------------------------------------------------------------------------------
    // ---------------------------------------------------------[1] build optimizer ---------------------------------------------------------
    // --------------------------------------------------------------------------------------------------------------------------------------

    // The solver type defines the method to solve the matrix inverse and the structure of the sparse matrix.
    // define a linear solver, which can be:
    //      LinearSolverCholmod: use sparse cholesky decomposition method. Inherited from LinearSolverCCS
    //      LinearSolverCSparse: use the CSparse method. Inherited from LinearSolverCCS
    //      LinearSolverPCG: uses the preconditioned conjugate gradient method, inherited from LinearSolver
    //      LinearSolverDense: use dense cholesky decomposition method. Inherited from LinearSolver
    //      LinearSolverEigen: the only dependency is eigen, which is solved by sparse Cholesky in eigen,
    //                         so it can be easily used in other places after compilation, and its performance is similar to CSparse.
    //                         Inherited from LinearSolver.
    // the dimension of the pose matrix, can be:
    //      BlockSolver_6_3: indicates that the pose is 6-dimensional and the observation point is 3-dimensional. Used for BA in 3D SLAM
    //      BlockSolver_7_3: a scale is added on the basis of BlockSolver_6_3
    //      BlockSolver_3_2: indicates that the pose is 3-dimensional and the observation point is 2-dimensional
    //  pose->6DOF, landmark->3DOF
    auto linear_solver = ::g2o::make_unique<::g2o::LinearSolverEigen<::g2o::BlockSolver_6_3::PoseMatrixType>>();
    // create a block solver, using the linear solver defined above
    auto block_solver = ::g2o::make_unique<::g2o::BlockSolver_6_3>(std::move(linear_solver));
    // create the optimization algorithm, such as Gauss-Newton, Gradient-Descent , Levenberg-Marquardt:
    //      g2o::OptimizationAlgorithmGaussNewton
    //      g2o::OptimizationAlgorithmLevenberg
    //      g2o::OptimizationAlgorithmDogleg
    auto algorithm = new ::g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    // !declaring an sparse optimizer  // graph model
    ::g2o::SparseOptimizer optimizer;
    // setup the optimizer
    optimizer.setAlgorithm(algorithm);
    // print debug log in the terminal, if true
    optimizer.setVerbose(false);

    // number of 3D points observed in this frame, initialized as 0
    unsigned int num_init_obs = 0;

    // --------------------------------------------------------------------------------------------------------------------------------------
    // -----------------------------------------[2] Convert frame to g2o vertex and set to optimizer-----------------------------------------
    // --------------------------------------------------------------------------------------------------------------------------------------

    // create a frame vertex
    auto frm_vtx = new g2o::se3::shot_vertex();
    // set vertex id == frame id   
    frm_vtx->setId(frm.id_);
     // initialize the estimation value as the current pose of this frame
    frm_vtx->setEstimate(util::converter::to_g2o_SE3(frm.cam_pose_cw_));
    // vertex not fixed, means to be optimized
    frm_vtx->setFixed(false);
    // add this frame vertex to the optimizer
    optimizer.addVertex(frm_vtx);

    // --------------------------------------------------------------------------------------------------------------------------------------
    // ----------------------------------------[3] Connect landmark's vertex with reprojection edge------------------------------------------
    // --------------------------------------------------------------------------------------------------------------------------------------
    // number of key points
    const unsigned int num_keypts = frm.num_keypts_;

    // container of reprojection edge
    using pose_opt_edge_wrapper = g2o::se3::pose_opt_edge_wrapper<data::frame>;
    // one frame observe many 3D points, this vector saves all the edge connected to those points
    std::vector<pose_opt_edge_wrapper> pose_opt_edge_wraps;
    // number of edges = number of keypoints observed by this frame
    pose_opt_edge_wraps.reserve(num_keypts);
    //  Chi-square value with significance level of 5%
    //  Degrees of freedom n=2
    constexpr float chi_sq_2D = 5.99146;
    const float sqrt_chi_sq_2D = std::sqrt(chi_sq_2D);
    // Degrees of freedom n=3
    constexpr float chi_sq_3D = 7.81473;
    const float sqrt_chi_sq_3D = std::sqrt(chi_sq_3D);

    // loop though all the key-points within the frame, find valid landmark
    for (unsigned int idx = 0; idx < num_keypts; ++idx) {
        auto lm = frm.landmarks_.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        ++num_init_obs;
        frm.outlier_flags_.at(idx) = false;

        // Connect the frame's vertex with a reprojection edge
        // cv::KeyPoint
        const auto& undist_keypt = frm.undist_keypts_.at(idx);
        // x_right -> disparities, for monocular, it should be less than 0
        const float x_right = frm.stereo_x_right_.at(idx);
        // octave: (pyramid layer) from which the keypoint has been extracted
        const float inv_sigma_sq = frm.inv_level_sigma_sq_.at(undist_keypt.octave);
        const auto sqrt_chi_sq = (frm.camera_->setup_type_ == camera::setup_type_t::Monocular)
                                     ? sqrt_chi_sq_2D
                                     : sqrt_chi_sq_3D;
        auto pose_opt_edge_wrap = pose_opt_edge_wrapper(&frm, frm_vtx, lm->get_pos_in_world(),
                                                        idx, undist_keypt.pt.x, undist_keypt.pt.y, x_right,
                                                        inv_sigma_sq, sqrt_chi_sq);
        pose_opt_edge_wraps.push_back(pose_opt_edge_wrap);
        optimizer.addEdge(pose_opt_edge_wrap.edge_);
    }

    // if number of 3D points observed less than 5, skip this optimization
    if (num_init_obs < 5) {
        return 0;
    }

    // --------------------------------------------------------------------------------------------------------------------------------------
    // ------------------------------------------------------[4] Run robust BA---------------------------------------------------------------
    // --------------------------------------------------------------------------------------------------------------------------------------

    unsigned int num_bad_obs = 0;
    // num_trials_ = 4 
    for (unsigned int trial = 0; trial < num_trials_; ++trial) {
        // !start bundle optimization, use the optimizer defined in [1,2], with the edge added in [3]
        // initialize the optimizer
        optimizer.initializeOptimization();
        // run the optimizer, num_each_iter_ = 10
        optimizer.optimize(num_each_iter_);

        num_bad_obs = 0;

        // loop through the pose_edge
        for (auto& pose_opt_edge_wrap : pose_opt_edge_wraps) {
            // ::g2o::OptimizableGraph::Edge
            auto edge = pose_opt_edge_wrap.edge_;

            // compute error when outlier_flag == true
            if (frm.outlier_flags_.at(pose_opt_edge_wrap.idx_)) {
                edge->computeError();
            }

            if (pose_opt_edge_wrap.is_monocular_) {
                // Monocular
                if (chi_sq_2D < edge->chi2()) {
                    // find outliers according to chi2 value of the edge
                    // which will be optimized in next iteration
                    frm.outlier_flags_.at(pose_opt_edge_wrap.idx_) = true;
                    pose_opt_edge_wrap.set_as_outlier();
                    ++num_bad_obs;
                }
                else {
                    frm.outlier_flags_.at(pose_opt_edge_wrap.idx_) = false;
                    pose_opt_edge_wrap.set_as_inlier();
                }
            }
            // Stereo/RGB-D
            else {
                if (chi_sq_3D < edge->chi2()) {
                    frm.outlier_flags_.at(pose_opt_edge_wrap.idx_) = true;
                    pose_opt_edge_wrap.set_as_outlier();
                    ++num_bad_obs;
                }
                else {
                    frm.outlier_flags_.at(pose_opt_edge_wrap.idx_) = false;
                    pose_opt_edge_wrap.set_as_inlier();
                }
            }

            if (trial == num_trials_ - 2) {
                edge->setRobustKernel(nullptr);
            }
        }

        if (num_init_obs - num_bad_obs < 5) {
            break;
        }
    }

    // 5. Update information

    frm.set_cam_pose(frm_vtx->estimate());

    return num_init_obs - num_bad_obs;
}

} // namespace optimize
} // namespace stella_vslam