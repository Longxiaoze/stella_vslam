

#ifndef EXAMPLE_UTIL_EUROC_PLANESEG_UTIL_H
#define EXAMPLE_UTIL_EUROC_PLANESEG_UTIL_H

#include <string>
#include <vector>

class euroc_sequence
{
public:
    struct frame
    {
        frame(const std::string &left_img_path, const std::string &right_img_path, const double timestamp, const std::string &mask_img_path)
            : left_img_path_(left_img_path), right_img_path_(right_img_path), timestamp_(timestamp), _mask_img_path(mask_img_path){};

        const std::string left_img_path_;
        const std::string right_img_path_;
        const double timestamp_;

        // FW:
        const std::string _mask_img_path;
    };

    explicit euroc_sequence(const std::string &seq_dir_path);

    virtual ~euroc_sequence() = default;

    std::vector<frame> get_frames() const;

private:
    std::vector<double> timestamps_;
    std::vector<std::string> left_img_file_paths_;
    std::vector<std::string> right_img_file_paths_;

    // FW:
    std::vector<std::string> _mask_img_file_paths;
};

#endif // EXAMPLE_UTIL_EUROC_UTIL_H
