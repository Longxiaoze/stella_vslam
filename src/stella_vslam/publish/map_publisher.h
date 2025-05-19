#ifndef STELLA_VSLAM_PUBLISH_MAP_PUBLISHER_H
#define STELLA_VSLAM_PUBLISH_MAP_PUBLISHER_H

#include "stella_vslam/type.h"

#include <mutex>

namespace stella_vslam {

class config;

namespace data {
class keyframe;
class landmark;
class map_database;
class Plane;
class Line;
} // namespace data

namespace publish {

class map_publisher {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Constructor
     * @param cfg
     * @param map_db
     */
    map_publisher(const std::shared_ptr<config>& cfg, data::map_database* map_db);

    /**
     * Destructor
     */
    virtual ~map_publisher();

    /**
     * Set current camera pose
     * NOTE: should be accessed from tracker thread
     * @param cam_pose_cw
     */
    void set_current_cam_pose(const Mat44_t& cam_pose_cw);

    /**
     * Get current camera pose
     * NOTE: should be accessed from viewer thread
     * @return
     */
    Mat44_t get_current_cam_pose();

    /**
     * Get all keyframes
     * @param all_keyfrms
     * @return number of keyframes in map
     */
    unsigned int get_keyframes(std::vector<data::keyframe*>& all_keyfrms);

    /**
     * Get all landmarks and local landmarks
     * @param all_landmarks
     * @param local_landmarks
     * @return number of landmarks in map
     */
    unsigned int get_landmarks(std::vector<data::landmark*>& all_landmarks,
                               std::set<data::landmark*>& local_landmarks);

    // FW: get all the landmark planes
    unsigned int get_landmark_planes(std::vector<data::Plane*>& all_landmark_planes);
    bool seg_or_not() const;

    // FW: draw planes
    struct PlaneColor
    {
        double _r;
        double _g;
        double _b;

        PlaneColor(double r, double g, double b) : _r(r), _g(g), _b(b)
        {
        }
    };
    std::vector<PlaneColor> get_available_color();

    // FW: get all the landmark lines
    unsigned int get_landmark_lines(std::vector<data::Line*>& all_landmark_lines);
    bool using_line_tracking() const;

private:
    //! config
    std::shared_ptr<config> cfg_;
    //! map database
    data::map_database* map_db_;

    std::vector<PlaneColor> _mPlaneColors;

    // -------------------------------------------
    //! mutex to access camera pose
    std::mutex mtx_cam_pose_;
    Mat44_t cam_pose_cw_ = Mat44_t::Identity();
};

} // namespace publish
} // namespace stella_vslam

#endif // STELLA_VSLAM_PUBLISH_MAP_PUBLISHER_H
