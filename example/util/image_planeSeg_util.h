

#ifndef EXAMPLE_UTIL_IMAGE_PLANESEG_UTIL_H
#define EXAMPLE_UTIL_IMAGE_PLANESEG_UTIL_H

#include <string>
#include <vector>

class image_sequence
{
public:
    struct frame
    {
        frame(const std::string &img_path,
              const std::string &mask_img_path,
              const double timestamp)
            : img_path_(img_path),
              mask_img_path_(mask_img_path),
              timestamp_(timestamp){};

        const std::string img_path_;
        const std::string mask_img_path_;
        const double timestamp_;
    };

    image_sequence(const std::string &img_dir_path, const double fps);

    virtual ~image_sequence() = default;

    std::vector<frame> get_frames() const;

private:
    const double fps_;

    std::vector<std::string> img_file_paths_;
    std::vector<std::string> mask_file_paths_;
};

#endif // EXAMPLE_UTIL_IMAGE_UTIL_H
