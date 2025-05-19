#ifndef STELLA_VSLAM_MATCH_PROJECTION_H
#define STELLA_VSLAM_MATCH_PROJECTION_H

#include "stella_vslam/type.h"
#include "stella_vslam/match/base.h"

#include <set>

namespace stella_vslam
{

namespace data
{
class frame;
class keyframe;
class landmark;
class Line; // FW:
}

namespace match {

class projection final : public base
{
public:
    explicit projection(const float lowe_ratio = 0.6, const bool check_orientation = true)
        : base(lowe_ratio, check_orientation) {}

    ~projection() final = default;

    //-----------------------------------------
    //! Find the 2D point of frame and 3D correspondence, and record the correspondence information in frame.landmarks_
    // used in tracking_module::search_local_landmarks()
    unsigned int match_frame_and_landmarks(data::frame& frm, const std::vector<data::landmark*>& local_landmarks, const float margin = 5.0) const;

    // FW:
    // find the 2D and 3D line correspondence
    // used in tracking_module::search_local_landmarks_line()
    // plp: find the 2D and 3D line correspondence used in tracking_module::search_local_landmarks_line()
    unsigned int match_frame_and_landmarks_line(data::frame& frm, const std::vector<data::Line*>& local_landmarks_line, const float margin = 5.0) const;

    //-----------------------------------------
    //! Reproject the 3D point observed in the last frame to the current frame and record the corresponding information in frame.landmarks_
    // used in frame::tracker::motion_based_track()
    unsigned int match_current_and_last_frames(data::frame& curr_frm, const data::frame& last_frm, const float margin) const;

    // FW:
    // find the 2D and 3D line correspondence
    // used in frame::tracker::motion_based_track()
    // plp: find the 2D and 3D line correspondence used in frame::tracker::motion_based_track()
    unsigned int match_current_and_last_frames_line(data::frame& curr_frm, const data::frame& last_frm, const float margin) const;

    //-----------------------------------------
    //! Reproject the 3D point observed by keyfarme to the current frame and record the corresponding information in frame.landmarks_
    //! If the current frame already corresponds, specify already_matched_lms so that it will not be reprojected.
    // used in see -> relocalizer::relocalize()
    unsigned int match_frame_and_keyframe(data::frame& curr_frm, data::keyframe* keyfrm, const std::set<data::landmark*>& already_matched_lms,
                                          const float margin, const unsigned int hamm_dist_thr) const;

    // FW:
    // find the 2D and 3D line correspondence
    // used in see -> relocalizer::relocalize()
    // plp: find the 2D and 3D line correspondence used in see -> relocalizer::relocalize()
    unsigned int match_frame_and_keyframe_line(data::frame& curr_frm, data::keyframe* keyfrm, const std::set<data::Line*>& already_matched_lms,
                                                const float margin, const unsigned int hamm_dist_thr) const;

    //-----------------------------------------
    //! After converting the coordinates of the 3D point with Sim3, reproject it to the keyframe and record the corresponding information in matched_lms_in_keyfrm.
    //! If the corresponding information is already recorded in matched_lms_in_keyfrm, it is excluded from the search.
    //! (NOTE: keyframe feature score and matched_lms_in_keyfrm.size () match)
    // used in loop_detector::validate_candidates()
    unsigned int match_by_Sim3_transform(data::keyframe* keyfrm, const Mat44_t& Sim3_cw, const std::vector<data::landmark*>& landmarks,
                                         std::vector<data::landmark*>& matched_lms_in_keyfrm, const float margin) const;

    //! Using the specified Sim3, convert and reproject the 3D points observed in each keyframe to the other keyframe, and find the corresponding points.
    //! matched_lms_in_keyfrm_1 records the 3D points observed in keyframe2, which correspond to the feature points (index) in keyframe1.
    // used in loop_detector::select_loop_candidate_via_Sim3()
    unsigned int match_keyframes_mutually(data::keyframe* keyfrm_1, data::keyframe* keyfrm_2, std::vector<data::landmark*>& matched_lms_in_keyfrm_1,
                                          const float& s_12, const Mat33_t& rot_12, const Vec3_t& trans_12, const float margin) const;
};

} // namespace match
} // namespace stella_vslam

#endif // STELLA_VSLAM_MATCH_PROJECTION_H
