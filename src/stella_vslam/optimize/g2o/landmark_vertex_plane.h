

#ifndef STELLA_VSLAM_OPTIMIZER_G2O_LANDMARK_VERTEX_PLANE_H
#define STELLA_VSLAM_OPTIMIZER_G2O_LANDMARK_VERTEX_PLANE_H

#include "stella_vslam/type.h"
#include <g2o/core/base_vertex.h>
#include "stella_vslam/data/landmark_plane.h"
#include "stella_vslam/optimize/g2o/Plane3D.h"

#include <g2o/config.h>
#include <g2o/stuff/misc.h>
#include <g2o/core/eigen_types.h>

namespace stella_vslam
{
    namespace optimize
    {
        namespace g2o
        {
            class landmark_vertex_plane
                : public ::g2o::BaseVertex<3, Plane3D>
            {
                // A vertex for 3D plane landmark:
                // The DOF of variables to be optimized is 3 -> (phi,psi,d), the datatype is Plane3D
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                landmark_vertex_plane();

                bool read(std::istream &is) override;
                bool write(std::ostream &os) const override;

                // reset function
                void setToOriginImpl() override
                {
                    _estimate = Plane3D(); // initialize the variables to be optimized
                }

                // update function (important), this is the ⊞ operater in mainfold
                void oplusImpl(const double *update_) override
                {
                    Eigen::Map<const ::g2o::Vector3> update(update_);
                    _estimate.oplus(update);
                }

                bool setEstimateDataImpl(const double *est) override
                {
                    Eigen::Map<const ::g2o::Vector4> _est(est);
                    _estimate.fromVector(_est);
                    return true;
                }

                bool getEstimateData(double *est) const override
                {
                    Eigen::Map<::g2o::Vector4> _est(est);
                    _est = _estimate.toVector();
                    return true;
                }

                int estimateDimension() const override
                {
                    return 4;
                }
            };

        } // namespace g2o
    }     // namespace optimize
} // namespace stella_vslam

#endif