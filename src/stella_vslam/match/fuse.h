#ifndef STELLA_VSLAM_MATCH_FUSE_H
#define STELLA_VSLAM_MATCH_FUSE_H

#include "stella_vslam/type.h"
#include "stella_vslam/match/base.h"

namespace stella_vslam {

namespace data {
class keyframe;
class landmark;
} // namespace data

namespace match {

class fuse final : public base {
public:
    explicit fuse(const float lowe_ratio = 0.6)
        : base(lowe_ratio, true) {}

    ~fuse() final = default;

    //! 3次元点(landmarks_to_check)をkeyframeに再投影し，keyframeで観測している3次元点と重複しているものを探す
    //! 重複しているものは同じindexでduplicated_lms_in_keyfrmに記録される
    //! replace_duplication()とは異なり，関数内でreplaceはしない
    //! NOTE: landmarks_to_check.size() == duplicated_lms_in_keyfrm.size()
    unsigned int detect_duplication(data::keyframe* keyfrm, const Mat44_t& Sim3_cw, const std::vector<data::landmark*>& landmarks_to_check,
                                    const float margin, std::vector<data::landmark*>& duplicated_lms_in_keyfrm);

    //! 3次元点(landmarks_to_check)をkeyframeに再投影し，keyframeで観測している3次元点と重複しているものを探す
    //! 重複しているものはより信頼できる3次元点を選択してreplaceする
    //! detect_duplication()とは異なり，関数内でreplaceを行う
    template<typename T>
    unsigned int replace_duplication(data::keyframe* keyfrm, const T& landmarks_to_check, const float margin = 3.0);
    // FW: similar function for 3D line
    // plp: Explicitly materialize replace_duplication_line
    template <typename T>
    unsigned int replace_duplication_line(data::keyframe *keyfrm, const T &landmarks_to_check, const float margin = 3.0);

};

} // namespace match
} // namespace stella_vslam

#endif // STELLA_VSLAM_MATCH_FUSE_H
