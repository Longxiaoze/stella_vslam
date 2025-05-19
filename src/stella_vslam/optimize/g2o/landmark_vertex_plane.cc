

#include "stella_vslam/optimize/g2o/landmark_vertex_plane.h"

namespace stella_vslam
{
    namespace optimize
    {
        namespace g2o
        {
            landmark_vertex_plane::landmark_vertex_plane()
                : ::g2o::BaseVertex<3, Plane3D>()
            {
            }

            bool landmark_vertex_plane::read(std::istream &is)
            {
                ::g2o::Vector4 lv;
                bool state = ::g2o::internal::readVector(is, lv);
                setEstimate(Plane3D(lv));
                return state;
            }

            bool landmark_vertex_plane::write(std::ostream &os) const
            {
                bool state = ::g2o::internal::writeVector(os, _estimate.toVector());
                return state;
            }
        } // namespace g2o
    }     // namespace optimize
} // namespace stella_vslam
