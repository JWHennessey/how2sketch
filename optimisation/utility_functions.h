#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <Eigen/LU>
#include <Eigen/Eigenvalues>

namespace utility
{
    auto completeNormal(Eigen::Matrix<float, 4, 3> face1) ->  Eigen::Vector3f;
    auto completeCentroid(Eigen::Matrix<float, 4, 3> face1) ->  Eigen::Vector3f;
    auto hasCoPlanarRelation(Eigen::Matrix<float, 4, 3> face1, Eigen::Matrix<float, 4, 3> face2) -> bool;
    auto hasOrthogonalRelation(Eigen::Matrix<float, 4, 3> face1, Eigen::Matrix<float, 4, 3> face2) -> int;
    auto hasCoAxialRelation(int axis1, Eigen::Matrix<float, 4, 3> face1, int axis2, Eigen::Matrix<float, 4, 3> face2) -> bool;
    auto findMidPlane(int axisId, Eigen::Matrix<float,4,3> faceA, Eigen::Matrix<float,4,3> faceB) -> Eigen::Matrix<float,4,3>;
}
#endif // UTILITY_FUNCTIONS_H
