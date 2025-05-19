

#ifndef STELLA_VSLAM_OPTIMIZER_G2O_EXTENDED_PLANE_POINT_DISTANCE_EDGE_H
#define STELLA_VSLAM_OPTIMIZER_G2O_EXTENDED_PLANE_POINT_DISTANCE_EDGE_H

#include "stella_vslam/type.h"
#include "stella_vslam/optimize/g2o/landmark_vertex.h"
#include "stella_vslam/optimize/g2o/landmark_vertex_plane.h"
#include "stella_vslam/optimize/g2o/Plane3D.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>

namespace stella_vslam
{
    namespace optimize
    {
        namespace g2o
        {
            namespace extended
            {
                class point_plane_distance_edge final
                    : public ::g2o::BaseUnaryEdge<1, Vec4_t, landmark_vertex>
                {
                public:
                    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                    point_plane_distance_edge();

                    // minimizing distance error to zero
                    void computeError() override
                    {
                        const landmark_vertex *vt_pt = static_cast<const landmark_vertex *>(_vertices[0]);

                        Vec3_t pos_w = vt_pt->estimate();
                        Vec4_t plane3D_function(_measurement); // (n, d)
                        _error[0] = (pos_w.dot(plane3D_function.head<3>()) + plane3D_function(3)) /
                                    plane3D_function.head<3>().norm();
                    }

                    // no implementation needed?
                    virtual bool read(std::istream &is);

                    virtual bool write(std::ostream &os) const;
                };
            } // namespace extended
        }     // namespace g2o
    }         // namespace optimize
} // namespace stella_vslam

#endif