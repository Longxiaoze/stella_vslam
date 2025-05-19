#ifndef STELLA_VSLAM_OPTIMIZER_G2O_EXTENDED_PLANE_POINT_EDGE_WRAPPER_H
#define STELLA_VSLAM_OPTIMIZER_G2O_EXTENDED_PLANE_POINT_EDGE_WRAPPER_H

#include "stella_vslam/optimize/g2o/extended/plane_point_distance_edge.h"
#include <g2o/core/robust_kernel_impl.h>


namespace stella_vslam
{
    namespace data
    {
        class landmark;
        class Plane;
    } // namespace data

    namespace optimize
    {
        namespace g2o
        {
            namespace extended
            {
                class point2plane_edge_wrapper
                {
                public:
                    point2plane_edge_wrapper() = delete;

                    point2plane_edge_wrapper(data::landmark *lm, landmark_vertex *lm_vtx,
                                             const Vec4_t pl_function,
                                             const bool use_huber_loss = true);

                    virtual ~point2plane_edge_wrapper() = default;

                    inline bool is_inlier() const
                    {
                        return edge_->level() == 0;
                    }

                    inline bool is_outlier() const
                    {
                        return edge_->level() != 0;
                    }

                    inline void set_as_inlier() const
                    {
                        edge_->setLevel(0);
                    }

                    inline void set_as_outlier() const
                    {
                        edge_->setLevel(1);
                    }

                    // members
                    ::g2o::OptimizableGraph::Edge *edge_;
                    data::landmark *_lm;
                };

                point2plane_edge_wrapper::point2plane_edge_wrapper(data::landmark *lm, landmark_vertex *lm_vtx,
                                                                   const Vec4_t pl_function,
                                                                   const bool use_huber_loss)
                    : _lm(lm)
                {
                    auto edge = new point_plane_distance_edge();
                    Vec4_t obs = pl_function;
                    edge->setMeasurement(obs);
                    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity()); // information matrix

                    edge->setVertex(0, lm_vtx);

                    edge_ = edge;

                    // loss function
                    if (use_huber_loss)
                    {
                        auto huber_kernel = new ::g2o::RobustKernelHuber();
                        huber_kernel->setDelta(1.0);
                        edge_->setRobustKernel(huber_kernel);
                    }
                }

            } // namespace extended
        }     // namespace g2o
    }         // namespace optimize
} // namespace stella_vslam

#endif