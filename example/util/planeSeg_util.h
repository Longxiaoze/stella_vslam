

#ifndef EXAMPLE_UTIL_PLANE_SEG_UTIL_H
#define EXAMPLE_UTIL_PLANE_SEG_UTIL_H

#include <string>
#include <vector>
// #include <spdlog/spdlog.h>

class ImageSequence_withPlaneSeg
{
public:
    struct frame
    {
        frame(const std::string &rgb_img_path,
              const std::string &depth_img_path,
              const std::string &mask_img_path,
              const double timestamp)
            : rgb_img_path_(rgb_img_path),
              depth_img_path_(depth_img_path),
              mask_img_path_(mask_img_path),
              timestamp_(timestamp){};

        const std::string rgb_img_path_;
        const std::string depth_img_path_;
        const std::string mask_img_path_;
        const double timestamp_;
    };

    explicit ImageSequence_withPlaneSeg(const std::string &seq_dir_path, bool b_predicted_depth, const double min_timediff_thr = 0.1);

    virtual ~ImageSequence_withPlaneSeg() = default;

    std::vector<frame> get_frames() const;

private:
    struct img_info
    {
        img_info(const double timestamp, const std::string &img_file_path)
            : timestamp_(timestamp), img_file_path_(img_file_path){};

        const double timestamp_;
        const std::string img_file_path_;
    };

    std::vector<img_info> acquire_image_information(const std::string &seq_dir_path,
                                                    const std::string &timestamp_file_path) const;

    std::vector<double> timestamps_;
    std::vector<std::string> rgb_img_file_paths_;
    std::vector<std::string> depth_img_file_paths_;
    std::vector<std::string> mask_img_file_paths_; // segmentation mask

    bool b_predicted_depth_;
};

#endif