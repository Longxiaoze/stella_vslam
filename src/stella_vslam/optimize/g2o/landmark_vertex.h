#ifndef STELLA_VSLAM_OPTIMIZER_G2O_LANDMARK_VERTEX_H
#define STELLA_VSLAM_OPTIMIZER_G2O_LANDMARK_VERTEX_H

#include "stella_vslam/type.h"

#include <g2o/core/base_vertex.h>

namespace stella_vslam {
namespace optimize {
namespace g2o {

class landmark_vertex final : public ::g2o::BaseVertex<3, Vec3_t> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    landmark_vertex();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void setToOriginImpl() override {
        _estimate.fill(0);
    }

    void oplusImpl(const double* update) override {
        Eigen::Map<const Vec3_t> v(update);
        _estimate += v;
    }
};

} // namespace g2o
} // namespace optimize
} // namespace stella_vslam

#endif // STELLA_VSLAM_OPTIMIZER_G2O_LANDMARK_VERTEX_H
