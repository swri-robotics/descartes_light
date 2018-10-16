#ifndef HYBRID_PLANNING_PATH_VISUALIZATION_H
#define HYBRID_PLANNING_PATH_VISUALIZATION_H

#include <tesseract_ros/kdl/kdl_env.h>
#include <geometry_msgs/PoseArray.h>
#include "hybrid_planning_common/eigen_typedefs.h"

namespace hybrid_planning_common
{

std::vector<geometry_msgs::Pose> toPoses(const AlignedVector<Eigen::Isometry3d>& vs);

geometry_msgs::PoseArray toPoseArray(const AlignedVector<Eigen::Isometry3d>& vs,
                                     const std::string& ref_frame);

}

#endif
