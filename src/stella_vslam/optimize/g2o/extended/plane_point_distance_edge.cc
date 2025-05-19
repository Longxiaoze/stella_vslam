

#include "stella_vslam/optimize/g2o/extended/plane_point_distance_edge.h"

namespace stella_vslam
{
    namespace optimize
    {
        namespace g2o
        {
            namespace extended
            {
                point_plane_distance_edge::point_plane_distance_edge()
                    : ::g2o::BaseUnaryEdge<1, Vec4_t, landmark_vertex>()
                {
                }

                bool point_plane_distance_edge::read(std::istream &is)
                {
                    for (unsigned int i = 0; i < 4; ++i)
                    {
                        is >> _measurement(i);
                    }

                    for (unsigned int i = 0; i < 2; ++i)
                    {
                        for (unsigned int j = i; j < 2; ++j)
                        {
                            is >> information()(i, j);
                            if (i != j)
                            {
                                information()(j, i) = information()(i, j);
                            }
                        }
                    }

                    return true;
                }

                bool point_plane_distance_edge::write(std::ostream &os) const
                {
                    for (unsigned int i = 0; i < 4; ++i)
                    {
                        os << measurement()(i) << " ";
                    }

                    for (unsigned int i = 0; i < 2; ++i)
                    {
                        for (unsigned int j = i; j < 2; ++j)
                        {
                            os << " " << information()(i, j);
                        }
                    }

                    return os.good();
                }
            } // namespace extended
        }     // namespace g2o
    }         // namespace optimize
} // namespace stella_vslam