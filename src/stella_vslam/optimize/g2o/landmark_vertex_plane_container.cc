

#include "stella_vslam/optimize/g2o/landmark_vertex_plane_container.h"

namespace stella_vslam
{
    namespace optimize
    {
        namespace g2o
        {
            landmark_vertex_plane_container::landmark_vertex_plane_container(const unsigned int offset,
                                                                             const unsigned int num_reserve)
                : _offset(offset)
            {
                _vtx_container.reserve(num_reserve);
            }

            landmark_vertex_plane *landmark_vertex_plane_container::create_vertex(data::Plane *pl, const bool is_constant)
            {
                // create a vertex
                const auto vtx_id = _offset + pl->_id;
                auto vtx = new landmark_vertex_plane();
                vtx->setId(vtx_id);
                auto plane = toPlane3D(pl);
                vtx->setEstimate(std::move(plane));
                vtx->setFixed(is_constant);

                // FW: TODO: debug this
                vtx->setMarginalized(true);

                // register in the database
                _vtx_container[pl->_id] = vtx;

                // update maximum id
                if (_max_vtx_id < vtx_id)
                {
                    _max_vtx_id = vtx_id;
                }

                return vtx;
            }

            Plane3D landmark_vertex_plane_container::toPlane3D(data::Plane *pl)
            {
                Vec3_t normal = pl->get_normal();
                double offset = pl->get_offset();

                ::g2o::Vector4 v;
                v << normal(0), normal(1), normal(2), offset;

                // FW: why?
                if (offset < 0)
                    v = -v;

                return Plane3D(v);
            }

        } // namespace g2o
    }     // namespace optimize
} // namespace stella_vslam