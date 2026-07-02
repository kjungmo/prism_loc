#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc_fusion/so3.hpp"
using namespace prism_loc_fusion;

TEST(So3, SkewIsCross) {
  Eigen::Vector3d a(1, 2, 3), b(-4, 5, 6);
  EXPECT_TRUE((skew(a) * b).isApprox(a.cross(b), 1e-12));
}
TEST(So3, ExpZeroIsIdentity) {
  Eigen::Quaterniond q = so3Exp(Eigen::Vector3d::Zero());
  EXPECT_NEAR(q.angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-12);
}
TEST(So3, LogExpRoundTrip) {
  for (const Eigen::Vector3d phi : {Eigen::Vector3d(0.01, -0.02, 0.005),
                                    Eigen::Vector3d(0.3, 0.4, -0.5),
                                    Eigen::Vector3d(1.2, 0.0, 0.0)}) {
    Eigen::Vector3d out = so3Log(so3Exp(phi));
    EXPECT_TRUE(out.isApprox(phi, 1e-9)) << "phi=" << phi.transpose() << " out=" << out.transpose();
  }
}
TEST(So3, ExpMatchesAngleAxis) {
  Eigen::Vector3d phi(0.0, 0.0, M_PI / 3);
  Eigen::Matrix3d R = so3Exp(phi).toRotationMatrix();
  Eigen::Matrix3d Rref(Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(R.isApprox(Rref, 1e-9));
}
