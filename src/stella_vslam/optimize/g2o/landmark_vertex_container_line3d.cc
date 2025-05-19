

#include "stella_vslam/optimize/g2o/landmark_vertex_container_line3d.h"

namespace stella_vslam
{
    namespace optimize
    {
        namespace g2o
        {

            landmark_vertex_container_line3d::landmark_vertex_container_line3d(const unsigned int offset,
                                                                               const unsigned int num_reserve)
                : offset_(offset)
            {
                vtx_container_.reserve(num_reserve);
            }

            VertexLine3D *landmark_vertex_container_line3d::create_vertex(const unsigned int id,
                                                                          const Vec6_t &pos_w,
                                                                          const bool is_constant)
            {
                // Create vertex
                const auto vtx_id = offset_ + id;
                auto vtx = new VertexLine3D();
                vtx->setId(vtx_id);
                auto line = Line3D(pos_w);         // pos_w: Plücker coordinates
                vtx->setEstimate(std::move(line)); // set the value of parameters need to be optimized, here, initialized by the value of pos_w
                vtx->setFixed(is_constant);

                vtx->setMarginalized(true);

                // Register in database
                vtx_container_[id] = vtx;

                // Update max ID
                if (max_vtx_id_ < vtx_id)
                {
                    max_vtx_id_ = vtx_id;
                }

                // Return the created vertex
                return vtx;
            }

        } // namespace g2o
    }     // namespace optimize
} // namespace stella_vslam
