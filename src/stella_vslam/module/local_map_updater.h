#ifndef STELLA_VSLAM_MODULE_LOCAL_MAP_UPDATER_H
#define STELLA_VSLAM_MODULE_LOCAL_MAP_UPDATER_H

#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/landmark_line.h"

namespace stella_vslam {

namespace data {
class frame;
class keyframe;
class landmark;

// FW:
class Line;
} // namespace data

namespace module {

class local_map_updater {
public:
    using keyframe_weights_t = std::unordered_map<data::keyframe*, unsigned int>;

    //! Constructor
    explicit local_map_updater(const data::frame& curr_frm, const unsigned int max_num_local_keyfrms);

    //! Destructor
    ~local_map_updater() = default;

    //! Get the local keyframes
    std::vector<data::keyframe*> get_local_keyframes() const;

    //! Get the local landmarks
    std::vector<data::landmark*> get_local_landmarks() const;

    // FW: return _local_lms_line
    std::vector<data::Line*> get_local_landmarks_line() const;

    //! Get the nearest covisibility
    data::keyframe* get_nearest_covisibility() const;

    //! Acquire the new local map
    bool acquire_local_map();

    // FW: call find_local_landmarks_line(), see below
    bool acquire_local_map_line();

private:
    //! Find the local keyframes
    bool find_local_keyframes();

    //! Compute keyframe weights
    keyframe_weights_t count_keyframe_weights() const;

    //! Find the first-order local keyframes
    auto find_first_local_keyframes(const keyframe_weights_t& keyfrm_weights)
        -> std::vector<data::keyframe*>;

    //! Find the second-order local keyframes
    auto find_second_local_keyframes(const std::vector<data::keyframe*>& first_local_keyframes) const
        -> std::vector<data::keyframe*>;

    //! Find the local landmarks
    bool find_local_landmarks();

    // FW: find local landmarks of 3D line from local keyframes
    bool find_local_landmarks_line();

    // frame ID
    const unsigned int frm_id_;
    // landmark associations
    const std::vector<data::landmark*> frm_lms_;
    // the number of keypoints
    const unsigned int num_keypts_;
    // maximum number of the local keyframes
    const unsigned int max_num_local_keyfrms_;

    // found local keyframes
    std::vector<data::keyframe*> local_keyfrms_;
    // found local landmarks
    std::vector<data::landmark*> local_lms_;
    // the nearst keyframe in covisibility graph, which will be found in find_first_local_keyframes()
    data::keyframe* nearest_covisibility_;

    // FW:
    std::vector<data::Line*> _local_lms_line;
};

} // namespace module
} // namespace stella_vslam

#endif // STELLA_VSLAM_MODULE_LOCAL_MAP_UPDATER_H
