

#ifndef STELLA_VSLAM_OPTIMIZER_G2O_LANDMARK_VERTEX_LINE3D_H
#define STELLA_VSLAM_OPTIMIZER_G2O_LANDMARK_VERTEX_LINE3D_H

#include "stella_vslam/type.h"
#include "stella_vslam/optimize/g2o/line3d.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>

namespace stella_vslam
{
    namespace optimize
    {
        namespace g2o
        {
            // FW:
            //  (This file re-written from g2o library)

            class VertexLine3D : public ::g2o::BaseVertex<4, Line3D>
            {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

                VertexLine3D();
                virtual bool read(std::istream &is);
                virtual bool write(std::ostream &os) const;

                virtual void setToOriginImpl()
                {
                    _estimate = Line3D(); // Plücker coordinates
                }

                virtual void oplusImpl(const number_t *update_)
                {
                    Eigen::Map<const ::g2o::Vector4> update(update_);
                    _estimate.oplus(update); //  Orthonormal representation
                }

                virtual bool setEstimateDataImpl(const number_t *est)
                {
                    Eigen::Map<const Vector6> _est(est);
                    _estimate = Line3D(_est);
                    return true;
                }

                virtual bool getEstimateData(number_t *est) const
                {
                    Eigen::Map<Vector6> _est(est);
                    _est = _estimate;
                    return true;
                }

                virtual int estimateDimension() const
                {
                    return 6;
                }
            };
        }
    }
}

#endif