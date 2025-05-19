#ifndef STELLA_VSLAM_OPTIMIZE_GRAPH_OPTIMIZER_H
#define STELLA_VSLAM_OPTIMIZE_GRAPH_OPTIMIZER_H

#include "stella_vslam/module/type.h"

#include <map>
#include <set>

namespace stella_vslam {

namespace data {
class keyframe;
class map_database;
class Line; // FW:
}

namespace optimize {

class graph_optimizer {
public:
    /**
     * Constructor
     * @param map_db
     * @param fix_scale
     */
    explicit graph_optimizer(data::map_database* map_db, const bool fix_scale);

    /**
     * Destructor
     */
    virtual ~graph_optimizer() = default;

    /**
     * Perform pose graph optimization
     * @param loop_keyfrm
     * @param curr_keyfrm
     * @param non_corrected_Sim3s
     * @param pre_corrected_Sim3s
     * @param loop_connections
     */
    void optimize(data::keyframe* loop_keyfrm, data::keyframe* curr_keyfrm,
                  const module::keyframe_Sim3_pairs_t& non_corrected_Sim3s,
                  const module::keyframe_Sim3_pairs_t& pre_corrected_Sim3s,
                  const std::map<data::keyframe *, std::set<data::keyframe *>>& loop_connections) const;

    // FW:
    inline Mat33_t skew(const Vec3_t& t) const {
        Mat33_t S;
        S << 0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0;
        return S;
    }

    // FW:
    bool endpoint_trimming(data::Line *local_lm_line,
                            const Vec6_t& plucker_coord,
                            Vec6_t& updated_pose_w) const;

private:
    //! map database
    const data::map_database* map_db_;

    //! SE3 optimization or Sim3 optimization
    const bool fix_scale_;
};

} // namespace optimize
} // namespace stella_vslam

#endif // STELLA_VSLAM_GRAPH_OPTIMIZER_H
