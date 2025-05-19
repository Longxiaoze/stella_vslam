

#ifndef STELLA_VSLAM_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_EXTENDED_LINE_H
#define STELLA_VSLAM_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_EXTENDED_LINE_H

#include "stella_vslam/type.h"

namespace stella_vslam
{
    namespace data
    {
        class keyframe;
        class map_database;
        class Line; // FW:
    }

    namespace optimize
    {
        class local_bundle_adjuster_extended_line
        {
        public:
            explicit local_bundle_adjuster_extended_line(data::map_database *map_db,
                                                         const unsigned int num_first_iter = 5,
                                                         const unsigned int num_second_iter = 10);

            virtual ~local_bundle_adjuster_extended_line() = default;

            void optimize(data::keyframe *curr_keyfrm, bool *const force_stop_flag) const;

            // FW:
            inline Mat33_t skew(const Vec3_t &t) const
            {
                Mat33_t S;
                S << 0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0;
                return S;
            }

            // FW: re-estimate two endpoints for visualization 3D line in the map
            bool endpoint_trimming(data::Line *local_lm_line,
                                   const Vec6_t &plucker_coord,
                                   Vec6_t &updated_pose_w) const;

        private:
            data::map_database *_map_db;

            //! number of iterations of first optimization
            const unsigned int num_first_iter_; // 5
            //! number of iterations of second optimization
            const unsigned int num_second_iter_; // 10

            // FW: print Debug info in the terminal
            const bool _setVerbose = false;
        };

    } // namespace optimize
} // namespace stella_vslam

#endif // STELLA_VSLAM_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_H
