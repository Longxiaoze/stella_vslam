

#include "image_planeSeg_util.h"

#include <dirent.h>
#include <algorithm>
#include <stdexcept>

#include <iostream>

image_sequence::image_sequence(const std::string &img_dir_path, const double fps)
    : fps_(fps)
{
    // RGB images
    std::string img_dir_rgb_path = img_dir_path + "/rgb";
    DIR *dir;
    if ((dir = opendir(img_dir_rgb_path.c_str())) == nullptr)
    {
        throw std::runtime_error("directory " + img_dir_rgb_path + " does not exist");
    }
    dirent *dp;
    for (dp = readdir(dir); dp != nullptr; dp = readdir(dir))
    {
        const std::string img_file_name = dp->d_name;
        if (img_file_name == "." || img_file_name == "..")
        {
            continue;
        }

        img_file_paths_.push_back(img_dir_path + "/rgb/" + img_file_name);
    }
    closedir(dir);
    std::sort(img_file_paths_.begin(), img_file_paths_.end());

    // segmentation mask images
    std::string img_dir_seg_path = img_dir_path + "/seg";
    DIR *tdir;
    if ((tdir = opendir(img_dir_seg_path.c_str())) == nullptr)
    {
        throw std::runtime_error("directory " + img_dir_seg_path + " does not exist");
    }
    dirent *tdp;
    for (tdp = readdir(tdir); tdp != nullptr; tdp = readdir(tdir))
    {
        const std::string seg_img_file_name = tdp->d_name;
        if (seg_img_file_name == "." || seg_img_file_name == "..")
        {
            continue;
        }

        mask_file_paths_.push_back(img_dir_path + "/seg/" + seg_img_file_name);
    }
    closedir(tdir);
    std::sort(mask_file_paths_.begin(), mask_file_paths_.end());
}

std::vector<image_sequence::frame> image_sequence::get_frames() const
{
    std::vector<frame> frames;
    for (unsigned int i = 0; i < img_file_paths_.size(); ++i)
    {
        frames.emplace_back(frame{img_file_paths_.at(i), mask_file_paths_.at(i), (1.0 / fps_) * i});
    }
    return frames;
}
