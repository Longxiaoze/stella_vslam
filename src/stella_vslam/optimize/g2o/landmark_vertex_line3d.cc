

#include "stella_vslam/optimize/g2o/landmark_vertex_line3d.h"

namespace stella_vslam
{
    namespace optimize
    {
        namespace g2o
        {
            VertexLine3D::VertexLine3D() : ::g2o::BaseVertex<4, Line3D>()
            {
            }

            bool VertexLine3D::read(std::istream &is)
            {
                Vector6 lv;
                bool state = ::g2o::internal::readVector(is, lv);
                setEstimate(Line3D(lv));
                return state;
            }

            bool VertexLine3D::write(std::ostream &os) const
            {
                return ::g2o::internal::writeVector(os, _estimate);
            }
        }
    }
}