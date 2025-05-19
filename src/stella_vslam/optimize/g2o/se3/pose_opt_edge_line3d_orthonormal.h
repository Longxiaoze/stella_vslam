

#ifndef STELLA_VSLAM_OPTIMIZER_G2O_SE3_POSE_OPT_EDGE_LINE3D_ORTHONORMAL_H
#define STELLA_VSLAM_OPTIMIZER_G2O_SE3_POSE_OPT_EDGE_LINE3D_ORTHONORMAL_H

#include "stella_vslam/type.h"
#include "stella_vslam/optimize/g2o/landmark_vertex.h"
#include "stella_vslam/optimize/g2o/se3/shot_vertex.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>

// FW:
#include "stella_vslam/optimize/g2o/line3d.h"
#include "stella_vslam/optimize/g2o/landmark_vertex_line3d.h"
#include "stella_vslam/util/converter.h"
#include <g2o/types/slam3d/parameter_se3_offset.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>

namespace stella_vslam
{
    namespace optimize
    {
        namespace g2o
        {
            namespace se3
            {

                class pose_opt_edge_line3d final
                    : public ::g2o::BaseUnaryEdge<2, Vec4_t, shot_vertex>
                {
                public:
                    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                    pose_opt_edge_line3d(); // Unary edge

                    bool read(std::istream &is) override;

                    bool write(std::ostream &os) const override;

                    void computeError() override
                    {
                        const shot_vertex *se3Vertex = static_cast<const shot_vertex *>(_vertices[0]);
                        auto pose_keyframe_SE3Quat = se3Vertex->estimate();
                        Vec3_t proj = cam_project(pose_keyframe_SE3Quat, _pos_w); // re-projected line function (ax+by+c=0)

                        Vec4_t obs(_measurement); // (xs,ys, xe, ye) the two endpoints of the detected line segment
                        _error(0) = (obs(0) * proj(0) + obs(1) * proj(1) + proj(2)) / sqrt(proj(0) * proj(0) + proj(1) * proj(1));
                        _error(1) = (obs(2) * proj(0) + obs(3) * proj(1) + proj(2)) / sqrt(proj(0) * proj(0) + proj(1) * proj(1));
                    }

                    // return the re-projected line function (ax+by+c=0)
                    inline Vec3_t cam_project(const ::g2o::SE3Quat &cam_pose_cw, const Vec6_t &plucker_coord)
                    {
                        Mat44_t pose_cw = util::converter::to_eigen_mat(cam_pose_cw);

                        const Mat33_t rot_cw = pose_cw.block<3, 3>(0, 0);
                        const Vec3_t trans_cw = pose_cw.block<3, 1>(0, 3);

                        Mat66_t transformation_line_cw = Eigen::Matrix<double, 6, 6>::Zero();
                        transformation_line_cw.block<3, 3>(0, 0) = rot_cw;
                        transformation_line_cw.block<3, 3>(3, 3) = rot_cw;
                        transformation_line_cw.block<3, 3>(0, 3) = skew(trans_cw) * rot_cw;

                        return _K * (transformation_line_cw * plucker_coord).block<3, 1>(0, 0);
                    }

                    inline Mat33_t skew(const Vec3_t &t) const
                    {
                        Mat33_t S;
                        S << 0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0;
                        return S;
                    }

                    Vec6_t _pos_w; // Plücker coordinates of the 3D line

                    number_t _fx, _fy, _cx, _cy;
                    Mat33_t _K;
                };

            } // namespace se3
        }     // namespace g2o
    }         // namespace optimize
} // namespace stella_vslam

#endif
