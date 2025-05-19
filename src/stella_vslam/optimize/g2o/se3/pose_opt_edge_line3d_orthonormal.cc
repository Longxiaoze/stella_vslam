

#include "stella_vslam/optimize/g2o/se3/pose_opt_edge_line3d_orthonormal.h"

namespace stella_vslam
{
    namespace optimize
    {
        namespace g2o
        {
            namespace se3
            {

                pose_opt_edge_line3d::pose_opt_edge_line3d()
                    : ::g2o::BaseUnaryEdge<2, Vec4_t, shot_vertex>()
                {
                }

                bool pose_opt_edge_line3d::read(std::istream &is)
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

                bool pose_opt_edge_line3d::write(std::ostream &os) const
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

            } // namespace se3
        }     // namespace g2o
    }         // namespace optimize
} // namespace stella_vslam
