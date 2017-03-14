#ifndef UTILITY_FUNCTIONS_CPP
#define UTILITY_FUNCTIONS_CPP

#include <qdebug>
#include "optimisation/utility_functions.h"
#include <iostream>

namespace utility
{

auto findMidPlane(int axisId, Eigen::Matrix<float,4,3> faceA, Eigen::Matrix<float,4,3> faceB) -> Eigen::Matrix<float,4,3>
{
//    auto maxCoefs = (faceA.colwise().maxCoeff() + faceB.colwise().maxCoeff()) * 0.5;
//    auto minCoefs = (faceA.colwise().minCoeff() + faceB.colwise().minCoeff()) * 0.5;

//    auto maxCoefsA = faceA.colwise().maxCoeff();
//    auto maxCoefsB = faceB.colwise().maxCoeff();
//    auto minCoefsA = faceA.colwise().minCoeff();
//    auto minCoefsB = faceB.colwise().minCoeff();
    axisId--;
    auto meanA = faceA.colwise().mean();
    auto meanB = faceB.colwise().mean();

    auto planeIndex = 0;
    auto planeValue = 0.0f;
    for(auto i = 0; i < 3; i++)
        if(meanA(i) != meanB(i))
        {
            planeIndex = i;
            planeValue = (meanA(i) + meanB(i)) * 0.5;
            break;
        }

//    Eigen::Matrix<float,4,3> face(faceA);
//    for(auto i = 0; i < 4; i++)
//        face(i,planeIndex) = planeValue;




    Eigen::Matrix<float,4,3> face(faceA);
    for(auto i = 0; i < 4; i++)
        face(i,planeIndex) = planeValue;


//    std::cout << "planeIndex " << planeIndex << std::endl;
//    std::cout << "faceA" << faceA << std::endl;
//    std::cout << "faceB" << faceB << std::endl;
//    std::cout << "out" << face << std::endl;


    return face;
}

auto completeNormal(Eigen::Matrix<float, 4, 3> face1) ->  Eigen::Vector3f
{
//    Eigen::Matrix<float, 4, 3> centered = face1.rowwise() - face1.colwise().mean();
//    Eigen::Matrix<float,3,3> cov = (centered.adjoint() * centered);
//    cov /= 4.0;

//    Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float, 3, 3> > es;
//    es.compute( cov );
//    Eigen::Vector3f n = es.eigenvectors().col(0);
//    return n;

    //std::cout << "face1 " << face1 << std::endl;
//    for(auto i = 0; i < 4; i++)
//        qDebug() << QVector3D(face1(i,0), face1(i,1), face1(i,2));

    Eigen::Vector3f BA = face1.row(1) - face1.row(0);
    Eigen::Vector3f CA = face1.row(2) - face1.row(0);
    Eigen::Vector3f n1 = BA.cross(CA);
    n1.normalize();
//    qDebug() << "n1 norm " << n1.norm();

//    Eigen::Vector3f BD = face1.row(1) - face1.row(3);
//    Eigen::Vector3f AD = face1.row(0) - face1.row(3);
//    Eigen::Vector3f n2 = BD.cross(AD);
//    n2.normalize();
//    qDebug() << "n2 norm " << n2.norm();

//    qDebug() << "n1 " << n1(0) << " " << n1(1) << " " << n1(2);
//    qDebug() << "n2 " << n2(0) << " " << n2(1) << " " << n2(2);
//    qDebug() << "n1 dot n2 " << n1.dot(n2);
    return n1;
}

auto completeCentroid(Eigen::Matrix<float, 4, 3> face1) ->  Eigen::Vector3f
{
    Eigen::Vector3f centroid = face1.colwise().mean();
    return centroid;
}

auto small_threshold = 0.001f;

auto hasCoPlanarRelation(Eigen::Matrix<float, 4, 3> face1, Eigen::Matrix<float, 4, 3> face2) -> bool
{
      Eigen::Vector3f normal1 = completeNormal(face1);
      Eigen::Vector3f normal2 = completeNormal(face2);
      Eigen::Vector3f centroid1 = completeCentroid(face1);
      Eigen::Vector3f centroid2 = completeCentroid(face2);

//    centroid1.normalize();
//    centroid2.normalize();

//      qDebug() << "n1 " << normal1(0) << " " << normal1(1) << " " << normal1(2);
//      qDebug() << "n2 " << normal2(0) << " " << normal2(1) << " " << normal2(2);
//      qDebug() << "c1 " << centroid1(0) << " " << centroid1(1) << " " << centroid1(2);
//      qDebug() << "c2 " << centroid2(0) << " " << centroid2(1) << " " << centroid2(2);

      Eigen::Vector3f delta = centroid2 - centroid1;
      auto dist = std::fabs(normal1.dot(delta));

      Eigen::Vector3f cp = normal1.cross(normal2);
      auto norm = cp.norm();

//      qDebug() << "norm " << norm;
//      qDebug() << "planarDist " << dist;

      if(isnan(dist))
          return false;

      if(dist > small_threshold || norm > small_threshold)
          return false;
      else
          return true;
}

auto hasOrthogonalRelation(Eigen::Matrix<float, 4, 3> face1, Eigen::Matrix<float, 4, 3> face2) -> int
{
    Eigen::Vector3f normal1 = completeNormal(face1);
    Eigen::Vector3f normal2 = completeNormal(face2);
    auto angle = std::fabs(normal1.dot(normal2));
    Eigen::Vector3f cp = normal1.cross(normal2);
    auto norm = cp.norm();
    //qDebug() << "normalAngle " << angle;
    //qDebug() << "CrossProduct Norm " << norm;
    if(norm < small_threshold)
        return 1; // Parallel
    else if(angle < small_threshold)
        return 2; // Orthogonal
    else
        return -1; // Neither
}

auto hasCoAxialRelation(int axis1, Eigen::Matrix<float, 4, 3> face1, int axis2, Eigen::Matrix<float, 4, 3> face2) -> bool
{
        Eigen::Vector3f normal1 = completeNormal(face1);
        Eigen::Vector3f normal2 = completeNormal(face2);
        Eigen::Vector3f centroid1 = completeCentroid(face1);
        Eigen::Vector3f centroid2 = completeCentroid(face2);
        Eigen::Vector3f delta = centroid1 - centroid2;
        delta.normalize();
        Eigen::Vector3f cp1 = normal1.cross(normal2);
        Eigen::Vector3f cp2 = normal1.cross(delta);
        auto norm1 = std::fabs(cp1.norm());
        auto norm2 = std::fabs(cp2.norm());
        //qDebug() << "hasCoAxialRelation";
        //qDebug() << "norm1 " << norm1 << " norm2 " << norm2;
        if(norm1 < small_threshold && norm2 < 0.01)
        {
            return true;
        }
        else
        {
            return false;
        }

////    if(axis1 == axis2)
////    {
//        Eigen::Vector3f centroid1 = completeCentroid(face1);
//        Eigen::Vector3f centroid2 = completeCentroid(face2);
//        for(auto i = 0; i < 3; i++)
//        {
//            auto d = std::fabs(centroid1(i) - centroid2(i));
//            if(d < 0.001)
//            {
//                qDebug() << "axis1 " << axis1 << " index matched " << i << " d " << d;
//                return true;
//            }
//        }
////    }
//    return false;
}

}
#endif // UTILITY_FUNCTIONS_CPP
