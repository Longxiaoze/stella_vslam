#ifndef STELLA_VSLAM_DATA_COMMON_H
#define STELLA_VSLAM_DATA_COMMON_H

#include "stella_vslam/type.h"
#include "stella_vslam/camera/base.h"

#include <opencv2/core.hpp>
#include <nlohmann/json_fwd.hpp>

// FW: for LSD ...
// plp: include line_descriptor_custom.hpp
#include <opencv2/features2d.hpp>
#include "stella_vslam/feature/line_descriptor/line_descriptor_custom.hpp"
#include "stella_vslam/data/landmark_line.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace stella_vslam {
namespace data {

nlohmann::json convert_rotation_to_json(const Mat33_t& rot_cw);

Mat33_t convert_json_to_rotation(const nlohmann::json& json_rot_cw);

nlohmann::json convert_translation_to_json(const Vec3_t& trans_cw);

Vec3_t convert_json_to_translation(const nlohmann::json& json_trans_cw);

nlohmann::json convert_keypoints_to_json(const std::vector<cv::KeyPoint>& keypts);

std::vector<cv::KeyPoint> convert_json_to_keypoints(const nlohmann::json& json_keypts);

// FW:
// plp: convert keylines to json
nlohmann::json convert_keylines_to_json(const std::vector<cv::line_descriptor::KeyLine> &keylines);

// FW:
// plp: convert json to keylines
std::vector<cv::line_descriptor::KeyLine> convert_json_to_keylines(const nlohmann::json &json_keylines);

nlohmann::json convert_undistorted_to_json(const std::vector<cv::KeyPoint>& undist_keypts);

std::vector<cv::KeyPoint> convert_json_to_undistorted(const nlohmann::json& json_undist_keypts, const std::vector<cv::KeyPoint>& keypts = {});

nlohmann::json convert_descriptors_to_json(const cv::Mat& descriptors);

cv::Mat convert_json_to_descriptors(const nlohmann::json& json_descriptors);
// FW:
// plp: convert line lbd descriptors to json
nlohmann::json convert_lbd_descriptors_to_json(const cv::Mat &descriptors);
// FW:
// plp: convert json to line lbd descriptors
cv::Mat convert_json_to_lbd_descriptors(const nlohmann::json &json_lbd_descriptors);


/**
 * Assign all keypoints to cells to accelerate projection matching
 * @param camera
 * @param undist_keypts
 * @param keypt_indices_in_cells
 */
void assign_keypoints_to_grid(camera::base* camera, const std::vector<cv::KeyPoint>& undist_keypts,
                              std::vector<std::vector<std::vector<unsigned int>>>& keypt_indices_in_cells);

/**
 * Assign all keypoints to cells to accelerate projection matching
 * @param camera
 * @param undist_keypts
 * @return
 */
auto assign_keypoints_to_grid(camera::base* camera, const std::vector<cv::KeyPoint>& undist_keypts)
    -> std::vector<std::vector<std::vector<unsigned int>>>;

/**
 * Get x-y index of the cell in which the specified keypoint is assigned
 * @param camera
 * @param keypt
 * @param cell_idx_x
 * @param cell_idx_y
 * @return
 */
inline bool get_cell_indices(camera::base* camera, const cv::KeyPoint& keypt, int& cell_idx_x, int& cell_idx_y) {
    cell_idx_x = cvFloor((keypt.pt.x - camera->img_bounds_.min_x_) * camera->inv_cell_width_);
    cell_idx_y = cvFloor((keypt.pt.y - camera->img_bounds_.min_y_) * camera->inv_cell_height_);
    return (0 <= cell_idx_x && cell_idx_x < static_cast<int>(camera->num_grid_cols_)
            && 0 <= cell_idx_y && cell_idx_y < static_cast<int>(camera->num_grid_rows_));
}

/**
 * Get keypoint indices in cell(s) in which the specified point is located
 * @param camera
 * @param undist_keypts
 * @param keypt_indices_in_cells
 * @param ref_x
 * @param ref_y
 * @param margin
 * @param min_level
 * @param max_level
 * @return
 */
std::vector<unsigned int> get_keypoints_in_cell(camera::base* camera, const std::vector<cv::KeyPoint>& undist_keypts,
                                                const std::vector<std::vector<std::vector<unsigned int>>>& keypt_indices_in_cells,
                                                const float ref_x, const float ref_y, const float margin,
                                                const int min_level = -1, const int max_level = -1);

// FW:
// plp: get keylines in cell
std::vector<unsigned int> get_keylines_in_cell(const std::vector<cv::line_descriptor::KeyLine> &keylines,
                                                const float ref_x1, const float ref_y1,
                                                const float ref_x2, const float ref_y2,
                                                const float margin,
                                                const int min_level = -1, const int max_level = -1);
} // namespace data
} // namespace stella_vslam

#endif // STELLA_VSLAM_DATA_COMMON_H
