#ifndef STELLA_VSLAM_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_H
#define STELLA_VSLAM_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_H

#include "stella_vslam/type.h"

namespace stella_vslam {
namespace data {
class map_database;
class Line; // FW:
}

namespace optimize {

class global_bundle_adjuster {
public:
    /**
     * Constructor
     * @param map_db
     * @param num_iter
     * @param use_huber_kernel
     */
    explicit global_bundle_adjuster(data::map_database* map_db, const unsigned int num_iter = 10, const bool use_huber_kernel = true);

    /**
     * Destructor
     */
    virtual ~global_bundle_adjuster() = default;

    /**
     * Perform optimization
     * @param lead_keyfrm_id_in_global_BA
     * @param force_stop_flag
     */
    void optimize(const unsigned int lead_keyfrm_id_in_global_BA = 0, bool* const force_stop_flag = nullptr) const;

    // FW:
    inline Mat33_t skew(const Vec3_t &t) const
    {
        Mat33_t S;
        S << 0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0;
        return S;
    }

    // FW: re-estimate two endpoints for visualization 3D line in the map
    bool endpoint_trimming(data::Line* local_lm_line,
                            const Vec6_t &plucker_coord,
                            Vec6_t &updated_pose_w) const;

private:
    //! map database
    const data::map_database* map_db_;

    //! number of iterations of optimization
    unsigned int num_iter_;

    //! use Huber loss or not
    const bool use_huber_kernel_;
};

} // namespace optimize
} // namespace stella_vslam

#endif // STELLA_VSLAM_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_H
