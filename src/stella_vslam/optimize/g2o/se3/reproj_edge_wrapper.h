#ifndef STELLA_VSLAM_OPTIMIZE_G2O_SE3_REPROJ_EDGE_WRAPPER_H
#define STELLA_VSLAM_OPTIMIZE_G2O_SE3_REPROJ_EDGE_WRAPPER_H

#include "stella_vslam/camera/perspective.h"
#include "stella_vslam/camera/fisheye.h"
#include "stella_vslam/camera/equirectangular.h"
#include "stella_vslam/optimize/g2o/se3/perspective_reproj_edge.h"
#include "stella_vslam/optimize/g2o/se3/equirectangular_reproj_edge.h"
#include <g2o/core/robust_kernel_impl.h>

// FW:
#include "stella_vslam/data/landmark_line.h"
#include "stella_vslam/optimize/g2o/line3d.h"
#include "stella_vslam/optimize/g2o/se3/reproj_edge_line3d_orthonormal.h" // orthonormal representation

namespace stella_vslam {

namespace data {
class landmark;
class Line; // FW:
}

namespace optimize {
namespace g2o {
namespace se3 {

template<typename T>
class reproj_edge_wrapper {
public:
    reproj_edge_wrapper() = delete;

    /**
     * @brief Construct a new reprojection edge (Binary) wrapper object, e.g. for local/global BA
    * 
     * @param shot keyframe
     * @param shot_vtx  keyframe vertex
     * @param lm 3D point landmark
     * @param lm_vtx 3D point landmark vertex
     * @param idx keypoint id
     * @param obs_x pixel position x of the keypoint
     * @param obs_y pixel position y of the keypoint
     * @param obs_x_right < 0, if monocular
     * @param inv_sigma_sq inverse of covariance matrix
     * @param sqrt_chi_sq
     * @param use_huber_loss default = true
    */
    reproj_edge_wrapper(T* shot, shot_vertex* shot_vtx,
                        data::landmark* lm, landmark_vertex* lm_vtx,
                        const unsigned int idx, const float obs_x, const float obs_y, const float obs_x_right,
                        const float inv_sigma_sq, const float sqrt_chi_sq, const bool use_huber_loss = true);

    // FW:
    // reprojection edge between keyframe and MapLine (Plücker coordinates and orthonormal representation)
    reproj_edge_wrapper(T* shot, shot_vertex* shot_vtx, VertexLine3D* lm_vtx,
                        const unsigned int idx,
                        const cv::Point2f sp, const cv::Point2f ep,
                        const float inv_sigma_sq, const float sqrt_chi_sq,
                        const bool use_huber_loss = true);

    virtual ~reproj_edge_wrapper() = default;

    // why inlier has level of edge 0?
    inline bool is_inlier() const {
        return edge_->level() == 0;
    }

    // why outlier has level of edge 1?
    inline bool is_outlier() const {
        return edge_->level() != 0;
    }

    inline void set_as_inlier() const {
        edge_->setLevel(0);
    }

    inline void set_as_outlier() const {
        edge_->setLevel(1);
    }

    inline bool depth_is_positive() const;

    // FW:
    inline bool depth_is_positive_via_endpoints_trimming() const;

    // members
    ::g2o::OptimizableGraph::Edge* edge_;

    camera::base* camera_;
    // for keyframe
    T* shot_;
    data::landmark* lm_;
    const unsigned int idx_;
    const bool is_monocular_;

    data::Line* _lm_line; // FW: added corresponding landmark_line
};

// Constructor: T-> data::keyframe
template<typename T>
reproj_edge_wrapper<T>::reproj_edge_wrapper(T* shot, shot_vertex* shot_vtx,
                                            data::landmark* lm, landmark_vertex* lm_vtx,
                                            const unsigned int idx, const float obs_x, const float obs_y, const float obs_x_right,
                                            const float inv_sigma_sq, const float sqrt_chi_sq, const bool use_huber_loss)
    : camera_(shot->camera_), shot_(shot), lm_(lm), idx_(idx), is_monocular_(obs_x_right < 0) {
    switch (camera_->model_type_) {
        case camera::model_type_t::Perspective: {
            auto c = static_cast<camera::perspective*>(camera_);
            if (is_monocular_) {
                // create a reprojection error edge (Binary)
                auto edge = new mono_perspective_reproj_edge();

                const Vec2_t obs{obs_x, obs_y};
                // y-real
                edge->setMeasurement(obs);
                // information matrix
                edge->setInformation(Mat22_t::Identity() * inv_sigma_sq);

                // intrinsic parameters of the camera
                edge->fx_ = c->fx_;
                edge->fy_ = c->fy_;
                edge->cx_ = c->cx_;
                edge->cy_ = c->cy_;

                // 0: point landmark; 1: frame/keyframe
                edge->setVertex(0, lm_vtx);
                edge->setVertex(1, shot_vtx);

                edge_ = edge;
            }
            else {
                auto edge = new stereo_perspective_reproj_edge();

                const Vec3_t obs{obs_x, obs_y, obs_x_right};
                edge->setMeasurement(obs);
                edge->setInformation(Mat33_t::Identity() * inv_sigma_sq);

                edge->fx_ = c->fx_;
                edge->fy_ = c->fy_;
                edge->cx_ = c->cx_;
                edge->cy_ = c->cy_;
                edge->focal_x_baseline_ = camera_->focal_x_baseline_;

                edge->setVertex(0, lm_vtx);
                edge->setVertex(1, shot_vtx);

                edge_ = edge;
            }
            break;
        }
        case camera::model_type_t::Fisheye: {
            auto c = static_cast<camera::fisheye*>(camera_);
            if (is_monocular_) {
                auto edge = new mono_perspective_reproj_edge();

                const Vec2_t obs{obs_x, obs_y};
                edge->setMeasurement(obs);
                edge->setInformation(Mat22_t::Identity() * inv_sigma_sq);

                edge->fx_ = c->fx_;
                edge->fy_ = c->fy_;
                edge->cx_ = c->cx_;
                edge->cy_ = c->cy_;

                edge->setVertex(0, lm_vtx);
                edge->setVertex(1, shot_vtx);

                edge_ = edge;
            }
            else {
                auto edge = new stereo_perspective_reproj_edge();

                const Vec3_t obs{obs_x, obs_y, obs_x_right};
                edge->setMeasurement(obs);
                edge->setInformation(Mat33_t::Identity() * inv_sigma_sq);

                edge->fx_ = c->fx_;
                edge->fy_ = c->fy_;
                edge->cx_ = c->cx_;
                edge->cy_ = c->cy_;
                edge->focal_x_baseline_ = camera_->focal_x_baseline_;

                edge->setVertex(0, lm_vtx);
                edge->setVertex(1, shot_vtx);

                edge_ = edge;
            }
            break;
        }
        case camera::model_type_t::Equirectangular: {
            assert(is_monocular_);

            auto c = static_cast<camera::equirectangular*>(camera_);

            auto edge = new equirectangular_reproj_edge();

            const Vec2_t obs{obs_x, obs_y};
            edge->setMeasurement(obs);
            edge->setInformation(Mat22_t::Identity() * inv_sigma_sq);

            edge->cols_ = c->cols_;
            edge->rows_ = c->rows_;

            edge->setVertex(0, lm_vtx);
            edge->setVertex(1, shot_vtx);

            edge_ = edge;

            break;
        }
    }

    // loss function
    if (use_huber_loss) {
        auto huber_kernel = new ::g2o::RobustKernelHuber();
        huber_kernel->setDelta(sqrt_chi_sq);
        edge_->setRobustKernel(huber_kernel);
    }
}

// FW:
// plp
// edge wrapper for line reprojection edge (Plücker coordinates, and orthonormal representation)
template<typename T>
reproj_edge_wrapper<T>::reproj_edge_wrapper(T* shot, shot_vertex* shot_vtx, VertexLine3D* lm_vtx,
                                            const unsigned int idx,
                                            const cv::Point2f sp, const cv::Point2f ep,
                                            const float inv_sigma_sq, const float sqrt_chi_sq,
                                            const bool use_huber_loss)
    : camera_(shot->camera_),
        shot_(shot),
        lm_(nullptr),
        _lm_line(shot->get_landmark_line(idx)),
        idx_(idx),
        is_monocular_(true)
{
    auto c = static_cast<camera::perspective*>(camera_);

    // create a reprojection error edge (Binary)
    auto edge = new stella_vslam::optimize::g2o::se3::reproj_edge_line3d();

    Vec4_t obs{sp.x, sp.y, ep.x, ep.y};
    edge->setMeasurement(obs);                                // y-real
    edge->setInformation(Mat22_t::Identity() * inv_sigma_sq); // information matrix * inv_sigma_sq

    // intrinsic parameters of the camera
    edge->_fx = c->fx_;
    edge->_fy = c->fy_;
    edge->_cx = c->cx_;
    edge->_cy = c->cy_;

    edge->_K << c->fy_, 0.0, 0.0,
        0.0, c->fx_, 0.0,
        -c->fy_ * c->cx_, -c->fx_ * c->cy_, c->fx_ * c->fy_;

    edge->_cam_matrix = c->eigen_cam_matrix_;

    edge->setVertex(0, shot_vtx);
    edge->setVertex(1, lm_vtx);

    edge_ = edge;

    // loss function
    if (use_huber_loss) {
        auto huber_kernel = new ::g2o::RobustKernelHuber();
        huber_kernel->setDelta(sqrt_chi_sq);
        edge_->setRobustKernel(huber_kernel);
    }
}

// inline member function
template<typename T>
bool reproj_edge_wrapper<T>::depth_is_positive() const {
    switch (camera_->model_type_) {
        case camera::model_type_t::Perspective: {
            if (is_monocular_) {
                return static_cast<mono_perspective_reproj_edge*>(edge_)->mono_perspective_reproj_edge::depth_is_positive();
            }
            else {
                return static_cast<stereo_perspective_reproj_edge*>(edge_)->stereo_perspective_reproj_edge::depth_is_positive();
            }
        }
        case camera::model_type_t::Fisheye: {
            if (is_monocular_) {
                return static_cast<mono_perspective_reproj_edge*>(edge_)->mono_perspective_reproj_edge::depth_is_positive();
            }
            else {
                return static_cast<stereo_perspective_reproj_edge*>(edge_)->stereo_perspective_reproj_edge::depth_is_positive();
            }
        }
        case camera::model_type_t::Equirectangular: {
            return true;
        }
    }

    return true;
}

// FW:
template <typename T>
bool reproj_edge_wrapper<T>::depth_is_positive_via_endpoints_trimming() const
{
    return static_cast<stella_vslam::optimize::g2o::se3::reproj_edge_line3d*>(edge_)->stella_vslam::optimize::g2o::se3::reproj_edge_line3d::depth_is_positive_via_endpoints_trimming();
}

} // namespace se3
} // namespace g2o
} // namespace optimize
} // namespace stella_vslam

#endif // STELLA_VSLAM_OPTIMIZE_G2O_SE3_REPROJ_EDGE_WRAPPER_H
