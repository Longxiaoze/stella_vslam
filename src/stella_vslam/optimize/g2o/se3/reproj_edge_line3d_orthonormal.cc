

#include "stella_vslam/optimize/g2o/se3/reproj_edge_line3d_orthonormal.h"

namespace stella_vslam
{
    namespace optimize
    {
        namespace g2o
        {
            namespace se3
            {
                reproj_edge_line3d::reproj_edge_line3d()
                    : ::g2o::BaseBinaryEdge<2, Vec4_t, shot_vertex, VertexLine3D>()
                {
                }

                bool reproj_edge_line3d::read(std::istream &is)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        is >> _measurement[i];
                    }

                    for (int i = 0; i < 2; ++i)
                    {
                        for (int j = i; j < 2; ++j)
                        {
                            is >> information()(i, j);
                            if (i != j)
                                information()(j, i) = information()(i, j);
                        }
                    }

                    return true;
                }

                bool reproj_edge_line3d::write(std::ostream &os) const
                {
                    for (int i = 0; i < 4; i++)
                    {
                        os << measurement()[i] << " ";
                    }

                    for (int i = 0; i < 2; ++i)
                    {
                        for (int j = i; j < 2; ++j)
                        {
                            os << " " << information()(i, j);
                        }
                    }

                    return os.good();
                }

            }
        }
    }
}