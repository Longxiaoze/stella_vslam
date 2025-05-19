

#ifndef STELLA_VSLAM_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_EXTENDED_PLANE_H
#define STELLA_VSLAM_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_EXTENDED_PLANE_H

namespace stella_vslam
{
    namespace data
    {
        class keyframe;
        class map_database;
        class Plane; // FW:
    }

    namespace optimize
    {
        class local_bundle_adjuster_extended_plane
        {
        public:
            explicit local_bundle_adjuster_extended_plane(data::map_database *map_db,
                                                          const unsigned int num_first_iter = 5,
                                                          const unsigned int num_second_iter = 10);

            virtual ~local_bundle_adjuster_extended_plane() = default;

            void optimize(data::keyframe *curr_keyfrm, bool *const force_stop_flag) const;

        private:
            data::map_database *_map_db;

            //! number of iterations of first optimization
            const unsigned int num_first_iter_; // 5
            //! number of iterations of second optimization
            const unsigned int num_second_iter_; // 10
        };

    } // namespace optimize
} // namespace stella_vslam

#endif