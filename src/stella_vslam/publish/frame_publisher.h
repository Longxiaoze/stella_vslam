#ifndef STELLA_VSLAM_PUBLISH_FRAME_PUBLISHER_H
#define STELLA_VSLAM_PUBLISH_FRAME_PUBLISHER_H

#include "stella_vslam/config.h"
#include "stella_vslam/tracking_module.h"

#include <mutex>
#include <vector>

#include <opencv2/core/core.hpp>

// FW: for LSD ...
#include <opencv2/features2d.hpp>
#include "stella_vslam/feature/line_descriptor/line_descriptor_custom.hpp"
#include "stella_vslam/data/landmark_line.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace stella_vslam {

class tracking_module;

namespace data {
class map_database;
} // namespace data

namespace publish {

class frame_publisher {
public:
    /**
     * Constructor
     */
    frame_publisher(const std::shared_ptr<config>& cfg, data::map_database* map_db,
                    const unsigned int img_width = 1024);

    /**
     * Destructor
     */
    virtual ~frame_publisher();

    /**
     * Update tracking information
     * NOTE: should be accessed from system thread
     */
    void update(tracking_module* tracker);

    /**
     * Get the current image with tracking information
     * NOTE: should be accessed from viewer thread
     */
    cv::Mat draw_frame(const bool draw_text = true);

    // FW: functionality for segmentation input
    cv::Mat draw_seg_mask();

protected:
    unsigned int draw_initial_points(cv::Mat& img, const std::vector<cv::KeyPoint>& init_keypts,
                                     const std::vector<int>& init_matches, const std::vector<cv::KeyPoint>& curr_keypts,
                                     const float mag = 1.0) const;

    unsigned int draw_tracked_points(cv::Mat& img, const std::vector<cv::KeyPoint>& curr_keypts,
                                     const std::vector<bool>& is_tracked, const bool mapping_is_enabled,
                                     const float mag = 1.0) const;

    // FW:
    unsigned int draw_tracked_lines(cv::Mat& img, const std::vector<cv::line_descriptor::KeyLine>& curr_keylines,
                                    const std::vector<bool>& is_tracked_line, const bool mapping_is_enabled,
                                    const float mag = 1.0) const;

    void draw_info_text(cv::Mat& img, const tracker_state_t tracking_state, const unsigned int num_tracked,
                         const unsigned int num_tracked_line,
                        const double elapsed_ms, const bool mapping_is_enabled) const;

    // colors (BGR)
    const cv::Scalar mapping_color_{0, 255, 255};
    const cv::Scalar localization_color_{255, 255, 0};

    //! config
    std::shared_ptr<config> cfg_;
    //! map database
    data::map_database* map_db_;
    //! maximum size of output images
    const int img_width_;

    // -------------------------------------------
    //! mutex to access variables below
    std::mutex mtx_;

    //! raw img
    cv::Mat img_;
    //! tracking state
    tracker_state_t tracking_state_;

    //! initial keypoints
    std::vector<cv::KeyPoint> init_keypts_;
    //! matching between initial frame and current frame
    std::vector<int> init_matches_;

    //! current keypoints
    std::vector<cv::KeyPoint> curr_keypts_;

    // FW:
    std::vector<cv::line_descriptor::KeyLine> _curr_keylines;

    //! elapsed time for tracking
    double elapsed_ms_ = 0.0;

    //! mapping module status
    bool mapping_is_enabled_;

    //! tracking flag for each current keypoint
    std::vector<bool> is_tracked_;

    // FW:
    std::vector<bool> _is_tracked_line;

    // FW: flag of segmentation input
    cv::Mat _seg_mask_img;
};

} // namespace publish
} // namespace stella_vslam
#endif // STELLA_VSLAM_PUBLISH_FRAME_PUBLISHER_H
