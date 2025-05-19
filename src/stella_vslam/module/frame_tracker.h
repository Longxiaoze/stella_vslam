#ifndef STELLA_VSLAM_MODULE_FRAME_TRACKER_H
#define STELLA_VSLAM_MODULE_FRAME_TRACKER_H

#include "stella_vslam/type.h"
#include "stella_vslam/optimize/pose_optimizer.h"
#include "stella_vslam/optimize/pose_optimizer_extended_line.h"

#include "stella_vslam/initialize/base.h"
#include "stella_vslam/initialize/perspective.h"

namespace stella_vslam {

namespace camera {
class base;
} // namespace camera

namespace data {
class frame;
class keyframe;
class Line;
} // namespace data

namespace module
{

class frame_tracker
{
public:
    explicit frame_tracker(camera::base* camera, const unsigned int num_matches_thr = 20);

    bool motion_based_track(data::frame& curr_frm, const data::frame& last_frm, const Mat44_t& velocity) const;

    bool bow_match_based_track(data::frame& curr_frm, const data::frame& last_frm, data::keyframe* ref_keyfrm) const;

    bool robust_match_based_track(data::frame& curr_frm, const data::frame& last_frm, data::keyframe* ref_keyfrm) const;

    // FW:
    // plp: set_using_line_tracking
    void set_using_line_tracking();

private:
    unsigned int discard_outliers(data::frame& curr_frm) const;

    // FW:
    // plp: discard_outliers_line
    unsigned int discard_outliers_line(data::frame& curr_frm) const;

    const camera::base* camera_;
    const unsigned int num_matches_thr_;

    const optimize::pose_optimizer pose_optimizer_;
    const optimize::pose_optimizer_extended_line _pose_optimizer_extended_line;

    // FW:
    // plp: _b_use_line_tracking
    bool _b_use_line_tracking = false;
};

} // namespace module
} // namespace stella_vslam

#endif // STELLA_VSLAM_MODULE_FRAME_TRACKER_H
