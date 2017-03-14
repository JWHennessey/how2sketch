#include "meshsegmentcluster_old.h"

#include "matrix.h"
#include "mex.h"

#include <Eigen/Geometry>
#include "utils/matlabengine.h"
#define  BUFSIZE 256

MeshSegmentCluster::MeshSegmentCluster()
    : id(-1)
    , numberOfChildClusters(0)
    , hasParent(false)
    , hasGrandParent(false)
    , parentCluster(nullptr)
    , grandParentCluster(nullptr)
    , firstClusterIsParent(false)
    , firstClusterIsParentId(-1)
    , maxAreaFaceId(0)
{
    bbRotation = Eigen::Matrix3f::Identity();
    bbRotationInv = bbRotation.inverse();
}

MeshSegmentCluster::MeshSegmentCluster(MeshSegmentCluster* parent, MeshSegmentCluster* grandParent)
     : id(-1)
     , numberOfChildClusters(0)
     , parentCluster(parent)
     , hasParent(true)
     , grandParentCluster(grandParent)
     , firstClusterIsParent(false)
     , firstClusterIsParentId(-1)
     , maxAreaFaceId(0)
{
    bbRotation = Eigen::Matrix3f::Identity();
    bbRotationInv = bbRotation.inverse();

    hasGrandParent = (grandParent != nullptr);
}

MeshSegmentCluster::MeshSegmentCluster(bool selfParent, MeshSegmentCluster* grandParent)
    :  id(-1)
     , numberOfChildClusters(0)
     , parentCluster(this)
     , hasParent(selfParent)
     , grandParentCluster(grandParent)
    , firstClusterIsParent(selfParent)
    , firstClusterIsParentId(-1)
    , maxAreaFaceId(0)
{
    bbRotation = Eigen::Matrix3f::Identity();
    bbRotationInv = bbRotation.inverse();

    hasGrandParent = (grandParent != nullptr);
}

//When cluster is a group of segments
auto MeshSegmentCluster::create(QJsonArray jsonArray, trimesh::point mean, float max_coeff, int& counterRef) -> void
{
    counterRef++;
    id = counterRef;

    qDebug() << "MeshSegmentCluster::create(QJsonArray jsonArray, trimesh::point mean, float max_coeff)" << id;

    for(auto it = jsonArray.begin(); it != jsonArray.end(); it++)
    {
        if(it->isString())
        {
            clusters.push_back(new MeshSegmentCluster(this, parentCluster));
            auto meshSegmentCluster = clusters.back();
            meshSegmentCluster->create(it->toString(), mean, max_coeff, counterRef);
        }
        else if(it->isObject())
        {
            //qDebug() << "Is Object";

            auto object = it->toObject();
            //Is is an object
            if(object.find("object") != object.end())
            {
                auto objectVal = object["object"].toObject();
                auto parentVal = objectVal["parent"];
                auto childVal = objectVal["child"];
                clusters.push_back(new MeshSegmentCluster(this, parentCluster));
                auto meshSegmentCluster = clusters.back();
                meshSegmentCluster->create(parentVal.toString(), mean, max_coeff, counterRef);

                //meshSegmentCluster->createChildren(childVal.toArray(), mean, max_coeff, counterRef);

            }
            else if(object.find("cluster") != object.end()) // It is a cluster
            {
                clusters.push_back(new MeshSegmentCluster(true, parentCluster));
                auto arrayVal = object["cluster"];
                auto meshSegmentCluster = clusters.back();
                meshSegmentCluster->create(arrayVal.toArray(), mean, max_coeff, counterRef);
            }
        }
    }
    numberOfChildClusters = counterRef - id;
}


//When cluster is a group of segments
//auto MeshSegmentCluster::createChildren(QJsonArray jsonArray, trimesh::point mean, float max_coeff, int& counterRef) -> void
//{
//    counterRef++;
//    id = counterRef;

//    qDebug() << "MeshSegmentCluster::createChildren(QJsonArray jsonArray, trimesh::point mean, float max_coeff)" << id;

//    for(auto it = jsonArray.begin(); it != jsonArray.end(); it++)
//    {
//        if(it->isString())
//        {
//            childClusters.push_back(new MeshSegmentCluster(this, parentCluster));
//            auto meshSegmentCluster = childClusters.back();
//            meshSegmentCluster->create(it->toString(), mean, max_coeff, counterRef);
//        }
//        else if(it->isObject())
//        {
//            //qDebug() << "Is Object";

//            auto object = it->toObject();
//            //Is is an object
//            if(object.find("object") != object.end())
//            {
//                auto objectVal = object["object"].toObject();
//                auto parentVal = objectVal["parent"];
//                auto childVal = objectVal["child"];
//                childClusters.push_back(new MeshSegmentCluster(this, parentCluster));
//                auto meshSegmentCluster = childClusters.back();
//                meshSegmentCluster->create(parentVal.toString(), mean, max_coeff, counterRef);

//                meshSegmentCluster->createChildren(childVal.toArray(), mean, max_coeff, counterRef);

//            }
//            else if(object.find("cluster") != object.end()) // It is a cluster
//            {
//                childClusters.push_back(new MeshSegmentCluster(true, parentCluster));
//                auto arrayVal = object["cluster"];
//                auto meshSegmentCluster = childClusters.back();
//                meshSegmentCluster->create(arrayVal.toArray(), mean, max_coeff, counterRef);
//            }
//        }
//    }
//    numberOfChildClusters = counterRef - id;
//}

//When cluster is just a single segment
auto MeshSegmentCluster::create(QString meshFilename, trimesh::point mean, float max_coeff, int& counterRef) -> void
{
    counterRef++;
    id = counterRef;
    qDebug() << "MeshSegmentCluster::create(QString meshFilename, trimesh::point mean, float max_coeff)" << id;

    //qDebug() << "before meshSegment.size() " << meshSegment.size();
    //Load Mesh Segment
     auto trimesh = trimesh::TriMesh::read(meshFilename.toUtf8().constData());
     //Resize points and add points to mesh segment
     for(auto i = 0; i < (int)trimesh->vertices.size(); i++)
     {
         trimesh->vertices[i] -= mean;
         trimesh->vertices[i] /= max_coeff;
         auto p = trimesh->vertices[i];

         p[0] = trimesh->vertices[i][1];
         p[1] = trimesh->vertices[i][2];
         p[2] = trimesh->vertices[i][0];
         meshSegment.addPoint(p);
     }

     QString file("/Users/JamesHennessey/Dropbox/engd/multimodel_drawing_assistance/output_data/seg_");
     file.append(QString::number(id));
     file.append(QString(".ply"));
     //trimesh->write(file.toUtf8().constData());

     //Write Segments using IGL
     Eigen::MatrixXf segmentVertices(trimesh->vertices.size(), 3);
     Eigen::MatrixXi segmentFaces(trimesh->faces.size(), 3);
     for(auto i = 0; i < trimesh->vertices.size(); i++)
         segmentVertices.row(i) = Eigen::Vector3f(trimesh->vertices[i][1], trimesh->vertices[i][2], trimesh->vertices[i][0]);

     for(auto i = 0; i < trimesh->faces.size(); i++)
         segmentFaces.row(i) = Eigen::Vector3i(trimesh->faces[i][0], trimesh->faces[i][1], trimesh->faces[i][2]);

     igl::writePLY(file.toUtf8().constData(), segmentVertices, segmentFaces);

     meshSegment.addFaces(trimesh->faces);

     meshSegment.computeBoundingBox();
     //qDebug() << "after meshSegment.size() " << meshSegment.size();
}


auto MeshSegmentCluster::getAllSegments(std::vector<MeshSegment*>& segments) -> void
{
     //qDebug() << "segments.size() " << segments.size();
     if(clusters.size() > 0)
     {
         for(auto& cluster : clusters)
         {
             cluster->getAllSegments(segments);
         }
     }
     else
     {
         segments.push_back(&meshSegment);
//         for(auto& cluster : childClusters)
//         {
//             cluster->getAllSegments(segments);
//         }
         //qDebug() << "meshSegment.size() " << meshSegment.size();

     }
}

auto MeshSegmentCluster::writeBox() -> QString
{

   qDebug() << "MeshSegmentCluster::writeBox";
   if(clusters.size() == 0)
   {
       qDebug() << "if(clusters.size() == 0)";
       return meshSegment.writeBox(id);
   }
   else if(clusters.size() > 0)
   {
       qDebug() << "else if(clusters.size() > 1)";
       auto bb = getBBox();
       Eigen::MatrixXf iglVertices(8, 3);
       Eigen::MatrixXi iglFaces(12, 3);
       iglFaces << 0,7,1,
               0,6,7,
               0,2,1,
               2,1,3,
               0,6,2,
               6,4,2,
               2,4,3,
               4,5,3,
               7,5,3,
               7,3,1,
               6,7,5,
               4,6,5;
       for(auto j = 0; j < 8; j++)
       {
           auto p = bb[j];
           iglVertices.row(j) = Eigen::Vector3f(p[0], p[1], p[2]);
       }
       QString localFolderBox = "/Users/JamesHennessey/Dropbox/engd/multimodel_drawing_assistance/output_data/";

       QString filename = "box";
       filename.append(QString::number(id));
       filename.append(QString(".ply"));
       qDebug() << filename;
       localFolderBox.append(filename);
       qDebug() << "igl::writePLY(localFolderBox.toUtf8().constData(), iglVertices, iglFaces);";
       igl::writePLY(localFolderBox.toUtf8().constData(), iglVertices, iglFaces);
       return filename;
   }
   qDebug() << "Null";
   return QString("Null");
}

auto MeshSegmentCluster::writeDataAndGetJSON(int i) -> QJsonObject
{
     QJsonObject object;
    if(clusters.size() == 0)
    {
        object = meshSegment.writeDataAndGetJSON(i);
    }
    else if(clusters.size() > 1)
    {
        qDebug() << "write " << i;
        auto bb = getBBox();
        Eigen::MatrixXf iglVertices(8, 3);
        Eigen::MatrixXi iglFaces(12, 3);
        iglFaces << 0,7,1,
                0,6,7,
                0,2,1,
                2,1,3,
                0,6,2,
                6,4,2,
                2,4,3,
                4,5,3,
                7,5,3,
                7,3,1,
                6,7,5,
                4,6,5;
        for(auto j = 0; j < 8; j++)
        {
            auto p = bb[j];
            iglVertices.row(j) = Eigen::Vector3f(p[0], p[1], p[2]);
        }
        QString localFolderBox = "/Users/JamesHennessey/Dropbox/engd/multimodel_drawing_assistance/output_data/";

        QString filename = "box";
        filename.append(QString::number(i));
        filename.append(QString(".ply"));
        qDebug() << filename;
        localFolderBox.append(filename);
        igl::writePLY(localFolderBox.toUtf8().constData(), iglVertices, iglFaces);
        object["box"] = filename;
    }

//    if(guides.size() > 0)
//    {
//        QJsonArray jsonGuides;
//        for(auto& g : guides)
//        {

//            QJsonObject jsonFrom;
//            jsonFrom["x"] = g.first[0];
//            jsonFrom["y"] = g.first[1];
//            jsonFrom["z"] = g.first[2];
//            QJsonObject jsonTo;
//            jsonTo["x"] = g.second[0];
//            jsonTo["y"] = g.second[1];
//            jsonTo["z"] = g.second[2];
//            QJsonObject jsonGuidePair;
//            jsonGuidePair["from"] = jsonFrom;
//            jsonGuidePair["to"] = jsonTo;

//            jsonGuides.push_back(jsonGuidePair);

//        }
//        object["guides"] = jsonGuides;
//    }


    return object;
}

auto MeshSegmentCluster::clusterId() -> int
{
    //qDebug() << "MeshSegmentCluster::clusterId() " << id;
    return id;
}

auto MeshSegmentCluster::clusterSize() -> int
{
 //   return 1;
    if(clusters.size() == 0)
    {
        return 1;
    }
    auto val = clusters.size();
    if(hasParentCluster())
    {
        val++;
    }
    return val;
}

auto MeshSegmentCluster::boundingBoxesSize() -> int //Number of vertices for the bounding boxes
{
    return 8 * clusterSize();
}


auto MeshSegmentCluster::hasParentCluster() -> bool
{
    return (hasParent && clusters.size() != 0);
}

auto MeshSegmentCluster::boundingBoxes() -> std::vector<trimesh::point>
{
    //qDebug() << " MeshSegmentCluster::boundingBoxes()";
    std::vector<trimesh::point> boxesToReturn;
    if(hasParentCluster() )
    {
        //qDebug() << "parentCluster->getBBox()";
        //qDebug() << "parentClusterID " << parentCluster->clusterId() << " my id " << id;
        auto bBox = parentCluster->getBBox();
        boxesToReturn.insert(std::end(boxesToReturn), std::begin(bBox), std::end(bBox));
    }
    //qDebug() << "Parent bBox done";
    if(clusters.size() == 0)
    {
        //qDebug() << "meshSegment->getBBox()";
        auto bBox = meshSegment.getBBox();
        boxesToReturn.insert(std::end(boxesToReturn), std::begin(bBox), std::end(bBox));
    }
    else
    {
        for(auto& cluster : clusters)
        {
            //qDebug() << "cluster->getBBox()";
            auto bBox = cluster->getBBox();
            boxesToReturn.insert(std::end(boxesToReturn), std::begin(bBox), std::end(bBox));
        }
    }
    return boxesToReturn;
//    std::vector<trimesh::point> clusterBBoxes = getBBox();
//    qDebug() << "Next Cluster";
//    for(auto& point : clusterBBoxes)
//    {
//        qDebug() << "point " << point[0] << " " << point[1] << " " << point[2];
//    }
//    auto dataPoints = clusterBBoxes.data();
//    float* data = dataPoints[0];

//    return clusterBBoxes;
}


auto MeshSegmentCluster::getBBox() -> std::vector<trimesh::point>
{
   // qDebug() << "MeshSegmentCluster::getBBox()";
    if(adjustedBBox.size() > 0)
    {
        //qDebug() << "MeshSegmentCluster::getBBox() adjustedBBox";
        return adjustedBBox;
    }


    if(clusters.size() == 0)
    {
        //qDebug() << "clusters.size() == 0";
        return meshSegment.getBBox();
    }
    else
    {
        //qDebug() << "clusters.size() " << clusters.size();
        auto haveSeenEndNode = false;
        std::vector<trimesh::point> bBoxes;
        for(auto& cluster : clusters)
        {
            //qDebug() << "loop";
            //Only consider the next level of segments
            auto bBox = cluster->getBBox();
            auto isEndNode = cluster->clusterSize() == 1;
            if(!haveSeenEndNode && isEndNode)
            {
                haveSeenEndNode = true;
            }
            else if(haveSeenEndNode && !isEndNode)
            {
                continue;
            }
            bBoxes.insert(std::end(bBoxes), std::begin(bBox), std::end(bBox));
        }
        //qDebug() << "bBoxes.size() " << bBoxes.size();
        MeshSegment tempSegment;
        for(auto& point : bBoxes)
        {
            //qDebug() << "point " << point[0] << " " << point[1] << " " << point[2];
            tempSegment.addPoint(point);
        }

        tempSegment.computeBoundingBox(bbRotation, bbRotationInv);
        auto tempBBox = tempSegment.getBBox();
        //qDebug() << "tempBBox.size() " << tempBBox.size();
        return tempBBox;
    }
}

auto MeshSegmentCluster::boundingBoxesFaceData() -> unsigned short*
{
    unsigned short bbIndicesTemplate[] = {0,7,1,
                                          0,6,7,
                                          0,2,1,
                                          2,1,3,
                                          0,6,2,
                                          6,4,2,
                                          2,4,3,
                                          4,5,3,
                                          7,5,3,
                                          7,3,1,
                                          6,7,5,
                                          4,6,5};


    const static int bb_faces_count = 12;
    auto cluster_no = clusterSize();
    //qDebug() << "cluster_no " << cluster_no;
    auto boundingBoxFaces = (unsigned short*) std::malloc(cluster_no * bb_faces_count * 3 * sizeof(unsigned short));

    for(auto i = 0; i < cluster_no; i++)
    {
        //qDebug() << "Setup CLuster";
        auto step = i * 36;
        for(auto j = 0; j < bb_faces_count * 3; j++)
        {
            auto index = (j + step);
            boundingBoxFaces[index] = (unsigned short) bbIndicesTemplate[j] + (i * 8);
        }
    }

    return boundingBoxFaces;
}

//Depth first search to find cluster with correct id
auto MeshSegmentCluster::getCluster(const int id, int& counterRef) -> MeshSegmentCluster*
{
    counterRef++;
     //qDebug() << "MeshSegmentCluster::getCluster " << id << " " << counterRef;
     if(id == counterRef)
     {
        //qDebug() << "Return this";
        return this;
     }
     else
     {
         int childCount = 1;
         for(auto& cluster : clusters)
         {
             MeshSegmentCluster* returnedCluster = cluster->getCluster(id, counterRef);
             if(returnedCluster != nullptr && returnedCluster->clusterId() == counterRef)
             {
                 //qDebug() << "Return child cluster " << childCount;
                 return returnedCluster;
             }
             childCount++;
         }
//         for(auto& cluster : childClusters)
//         {
//             MeshSegmentCluster* returnedCluster = cluster->getCluster(id, counterRef);
//             if(returnedCluster != nullptr && returnedCluster->clusterId() == counterRef)
//             {
//                 //qDebug() << "Return child cluster " << childCount;
//                 return returnedCluster;
//             }
//             //childCount++;
//         }
     }
//     qDebug() << "return nullptr counterRef " << counterRef;
     return nullptr;
}

auto MeshSegmentCluster::getNoChildClusters() -> int
{
    return numberOfChildClusters;
}

auto MeshSegmentCluster::setBBoxRotations(Eigen::Matrix3f eigTrans, Eigen::Matrix3f eigTransInv) -> void
{
   bbRotation = eigTrans;
   bbRotationInv = eigTransInv;
   for(auto& cluster : clusters)
   {
       cluster->setBBoxRotations(bbRotation, bbRotationInv);
   }
}


auto computeFaceNormal(Eigen::MatrixXf plane) -> Eigen::Vector3f
{
    // 7,5,3,
    Eigen::Vector3f a = plane.row(0);
    Eigen::Vector3f b = plane.row(1);
    Eigen::Vector3f c = plane.row(2);
    Eigen::Vector3f ab = b - a;
    Eigen::Vector3f ac = c - a;
    Eigen::Vector3f dir = ab.cross(ac);
    return dir.normalized();
}

auto findTopRight(Eigen::MatrixXf input, std::pair<int, int> indexes) -> Eigen::Vector3f
{
    Eigen::Vector3f output = input.row(0);

    for(auto i = 1; i < input.rows(); i++)
    {
        if(input(i, indexes.first) > output(indexes.first))
            output(indexes.first) = input(i, indexes.first);

        if(input(i, indexes.second) > output(indexes.second))
            output(indexes.second) = input(i, indexes.second);
    }

    return output;

}


auto findBottomLeft(Eigen::MatrixXf input, std::pair<int, int> indexes) -> Eigen::Vector3f
{
    Eigen::Vector3f output = input.row(0);

    for(auto i = 1; i < input.rows(); i++)
    {
        if(input(i, indexes.first) < output(indexes.first))
            output(indexes.first) = input(i, indexes.first);

        if(input(i, indexes.second) < output(indexes.second))
            output(indexes.second) = input(i, indexes.second);
    }
    return output;
}

auto MeshSegmentCluster::getParentCluster() ->  MeshSegmentCluster*
{
        return parentCluster;
}



auto computeBoundingFace(std::vector<trimesh::point> boundingBox, std::function<bool(int)> functor) -> Eigen::Matrix<float, 4, 3>
{
    Eigen::MatrixXf boundingFace(4,3);
    int count = 0;
    int allCount = 1;
    for(auto& point : boundingBox)
    {
        //qDebug() << evenOddCount;
        //if(evenOddCount < 5)
        if(functor(allCount))
        {
            //qDebug() << "bounding box A point " << point[0] << " " << point[1] << " " << point[2];
            for(auto i = 0; i < 3; i++)
                boundingFace(count, i) = point[i];
            count++;
        }
        allCount++;
    }
    return boundingFace;
}

auto MeshSegmentCluster::getGrandParentBBox() -> std::vector<trimesh::point>
{
    if(hasParent)
    {
        return parentCluster->getBBox();
    }
    else
    {
        std::vector<trimesh::point> empty;
        return empty;
    }
}




auto front = [](int x) -> bool { return (x % 2 == 0); };
auto notFront = [](int x) -> bool { return (x % 2 != 0); };
auto frontIndexes = std::make_pair(0, 2);
int myFrontIndexOrder[] = {0,1,2,3};
auto frontIndexOrder = std::vector<int>(myFrontIndexOrder, myFrontIndexOrder + sizeof(myFrontIndexOrder) / sizeof(int));

auto leftLambda = [](int x) -> bool { return (x < 5); };
auto notLeftLambda = [](int x) -> bool { return (x >= 5); };
auto leftIndexes = std::make_pair(1, 2);
int myLeftIndexOrder[] = {0,1,2,3};
auto leftIndexOrder = std::vector<int>(myLeftIndexOrder, myLeftIndexOrder + sizeof(myLeftIndexOrder) / sizeof(int));

auto top = [](int x) -> bool { return (x > 2 && x < 7); };
auto notTop = [](int x) -> bool { return (x <= 2 && x >= 7); };
auto topIndexes = std::make_pair(0, 1);
int myTopIndexOrder[] = {0,1,3,2};
auto topIndexOrder = std::vector<int>(myTopIndexOrder, myTopIndexOrder + sizeof(myTopIndexOrder) / sizeof(int) );



auto  MeshSegmentCluster::calculateOptimisationBoxFace() -> void
{

    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel ;
    using Iso_rectangle_2 = CGAL::Iso_rectangle_2<Kernel>;
    using CGAL_Point = Kernel::Point_2;

    std::vector<std::function<bool(int)>> functors;
    functors.push_back(front);
    functors.push_back(leftLambda);
    functors.push_back(top);
    functors.push_back(notFront);
    functors.push_back(notLeftLambda);
    functors.push_back(notTop);

    std::vector<std::pair<int, int>> planeIndexes;
    planeIndexes.push_back(frontIndexes);
    planeIndexes.push_back(leftIndexes);
    planeIndexes.push_back(topIndexes);

    for(auto& cluster : clusters)
    {
        int maxIndex = 0;
        float maxArea = 0.0;

        for(auto i = 0; i < 3; i++)
        {

            auto boundingBox = parentCluster->getBBox();
            Eigen::MatrixXf boundingFace = computeBoundingFace(boundingBox, functors[i]);


            boundingFace = (bbRotationInv * boundingFace.transpose()).transpose();

            Eigen::Vector3f bottom_left = findBottomLeft(boundingFace, planeIndexes[i]);
            Eigen::Vector3f top_right = findTopRight(boundingFace, planeIndexes[i]);

            Eigen::Vector4f parentFace(bottom_left(planeIndexes[i].first), bottom_left(planeIndexes[i].second), top_right(planeIndexes[i].first), top_right(planeIndexes[i].second));

            auto bBox = cluster->getBBox();
            int allCount = 1;
            int count = 0;
            Eigen::MatrixXf innerBox(4, 3);
            for(auto& point : bBox)
            {

                if(functors[i](allCount))
                {
                    for(auto i = 0; i < 3; i++)
                        innerBox(count, i) = point[i];
                    //qDebug() << "cluster " << count << " " << point[0] << " " << point[1] << " " << point[2];
                    count++;
                }

                allCount++;
            }

            innerBox = (bbRotationInv * innerBox.transpose()).transpose();

            bottom_left = findBottomLeft(innerBox, planeIndexes[i]);
            top_right = findTopRight(innerBox, planeIndexes[i]);

            Eigen::Vector4f innerFace(bottom_left(planeIndexes[i].first), bottom_left(planeIndexes[i].second), top_right(planeIndexes[i].first), top_right(planeIndexes[i].second));

            Iso_rectangle_2 input_child(CGAL_Point(innerFace(0),innerFace(1)),CGAL_Point(innerFace(2),innerFace(3)));
            Iso_rectangle_2 input_parent(CGAL_Point(parentFace(0),parentFace(1)),CGAL_Point(parentFace(2),parentFace(3)));

            auto inters = CGAL::intersection(input_child,input_parent);
            Iso_rectangle_2 intersection;
            CGAL::assign(intersection,inters);
            auto area = intersection.area() / input_child.area();
            //qDebug() << "i " << i << " area " << area;
            if(i == 0 || area > maxArea && area <= 1.0)
            {
                maxArea = area;
                maxIndex = i;
            }


            Eigen::Vector4f diff = parentFace - innerFace;
            qDebug() << "diff.norm() " << diff.norm();
            if(diff.norm() < 1.0e-04)//
            {
                firstClusterIsParent = true;
                firstClusterIsParentId = cluster->clusterId();
                qDebug() << "firstClusterIsParentId " << firstClusterIsParentId;
            }


        }
        //maxIndex
        cluster->setMaxAreaFaceId(maxIndex);

    }

}

auto MeshSegmentCluster::setMaxAreaFaceId(int i) -> void
{
    maxAreaFaceId = i;
}
auto MeshSegmentCluster::getMaxAreaFaceId() -> int { return maxAreaFaceId;}


auto computeAlternativeBoundingFace(std::vector<trimesh::point> boundingBox, std::function<bool(int)> functor) -> Eigen::Matrix<float, 4, 3>
{
    Eigen::MatrixXf boundingFace(4,3);
    int count = 0;
    int allCount = 1;
    for(auto& point : boundingBox)
    {
        //qDebug() << evenOddCount;
        //if(evenOddCount < 5)
        if(!functor(allCount))
        {
            //qDebug() << "bounding box A point " << point[0] << " " << point[1] << " " << point[2];
            for(auto i = 0; i < 3; i++)
                boundingFace(count, i) = point[i];
            count++;
        }
        allCount++;
    }
    return boundingFace;
}

auto snapClosestFaces(std::vector<trimesh::point> parent, std::vector<trimesh::point> child, std::function<bool(int)> functor, std::pair<int, int> planeIndexes, int functorIndex) -> std::vector<trimesh::point>
{

    //qDebug() << "snapClosestFaces";

    float parentFace[2];
    float childFace[2];
    float childMid = 0.0;

    int allCount = 1;
    for(auto j = 0; j < 8; j++)
    {
        auto parentPoint = parent[j];
        auto childPoint = child[j];
        if(functor(allCount))
        {
            for(auto i = 0; i < 3; i++)
            {
                if(i != planeIndexes.first && i != planeIndexes.second)
                {
                    parentFace[0] = parentPoint[i];
                    childFace[0] = childPoint[i];
                }
            }
        }
        else
        {
            for(auto i = 0; i < 3; i++)
            {
                if(i != planeIndexes.first && i != planeIndexes.second)
                {
                    parentFace[1] = parentPoint[i];
                    childFace[1] = childPoint[i];
                }
            }
        }

        for(auto i = 0; i < 3; i++)
        {
            if(i != planeIndexes.first && i != planeIndexes.second)
            {
                childMid += childPoint[i];
            }
        }

        allCount++;
    }


    childMid /= 8.0;

    auto minDist = 100000.0;
    auto parentIndex = 0;
    for(auto i = 0; i < 2; i++)
    {

        //qDebug() << "parentFace[i] " << parentFace[i];
        auto dist = std::fabs(parentFace[i] - childMid);
        if(dist < minDist)
        {
            parentIndex = i;
            minDist = dist;
        }

    }


    minDist = 100000.0;
    auto childIndex = 0;
    for(auto i = 0; i < 2; i++)
    {

        //qDebug() << "parentFace[i] " << parentFace[i];
        auto dist = std::fabs(parentFace[parentIndex] - childFace[i]);
        if(dist < minDist)
        {
            childIndex = i;
            minDist = dist;
        }

    }

    //qDebug() << "minDist " << minDist;

    for(auto j = 0; j < 8; j++)
    {
        auto& childPoint = child[j];
        for(auto i = 0; i < 3; i++)
        {
            if(i != planeIndexes.first && i != planeIndexes.second)
            {
                if(childPoint[i] == childFace[childIndex])
                {
                    //qDebug() << "childPoint[i] before " << childPoint[i];
                    childPoint[i] = parentFace[parentIndex];
                    //qDebug() << "childPoint[i] before " << childPoint[i];
                }
            }
        }

    }

    return child;
}
//Front = evenxx
//Left = < 5
//Top = > 2 && < 7
auto MeshSegmentCluster::runOptimisation(Eigen::Matrix<float, 200, 4>& drawnBoxes, int& drawnBoxCount, MeshSegmentCluster* cluster39) -> void
{

    std::vector<std::function<bool(int)>> functors;
    functors.push_back(front);
    functors.push_back(leftLambda);
    functors.push_back(top);
    functors.push_back(notFront);
    functors.push_back(notLeftLambda);
    functors.push_back(notTop);

    std::vector<std::pair<int, int>> planeIndexes;
    planeIndexes.push_back(frontIndexes);
    planeIndexes.push_back(leftIndexes);
    planeIndexes.push_back(topIndexes);

    std::vector<std::vector<int>> planeVertexOrder;
    planeVertexOrder.push_back(frontIndexOrder);
    planeVertexOrder.push_back(leftIndexOrder);
    planeVertexOrder.push_back(topIndexOrder);

    calculateOptimisationBoxFace();

    auto engine = MatlabEngine::getInstance();
    engEvalString(engine->getEngine(), "cd /Users/JamesHennessey/Documents/MATLAB/selection_ordering");
    engEvalString(engine->getEngine(), "close all;");

    auto previousBoxWithDiffFunctorCount = 0;


    for(auto funcorIndex = 0; funcorIndex < 3; funcorIndex++)
    {
        std::vector<MeshSegmentCluster*> candidateClusters;
        for(auto& cluster : clusters)
        {
            // cluster->clusterId() != 5 %%
            if(cluster->getMaxAreaFaceId() == funcorIndex && cluster->clusterId() != firstClusterIsParentId )
                candidateClusters.push_back(cluster);
        }

        if(candidateClusters.size() == 0)
            continue;

        qDebug() << "funcorIndex " << funcorIndex;

        auto functor = functors[funcorIndex];
        auto planeIndex = planeIndexes[funcorIndex];

        auto boundingBox = parentCluster->getBBox();
        Eigen::MatrixXf boundingFace = computeBoundingFace(boundingBox, functor);
        Eigen::MatrixXf boundingFaceAlt = computeAlternativeBoundingFace(boundingBox, functor);

        boundingFace = (bbRotationInv * boundingFace.transpose()).transpose();
        boundingFaceAlt = (bbRotationInv * boundingFaceAlt.transpose()).transpose();

        std::cout << "boundingFace " << boundingFace << std::endl;
        std::cout << "boundingFaceAlt " << boundingFaceAlt << std::endl;

        Eigen::Vector3f bottom_left = findBottomLeft(boundingFace, planeIndex);
        Eigen::Vector3f top_right = findTopRight(boundingFace, planeIndex);
        Eigen::Vector4f boundingFaceTwoPoints(bottom_left(planeIndex.first), bottom_left(planeIndex.second), top_right(planeIndex.first), top_right(planeIndex.second));

        //qDebug() << "boundingFaceTwoPoints " << boundingFaceTwoPoints(0) << " " << boundingFaceTwoPoints(1) << " " << boundingFaceTwoPoints(2) << " " << boundingFaceTwoPoints(3);

        drawnBoxes.row(0) = boundingFaceTwoPoints;
        drawnBoxCount = 1;
        if(hasGrandParent)
        {
            //qDebug() << "--- parentCluster->hasParentCluster()";
            //MeshSegmentCluster* grandParentCluster = parentCluster->getParentCluster();
            auto grandParentBoundingBox = grandParentCluster->getGrandParentBBox();

            if(grandParentBoundingBox.size() > 0)
            {
                Eigen::MatrixXf grandParentFace = computeBoundingFace(grandParentBoundingBox, functor);
                Eigen::MatrixXf grandParentAltFace = computeAlternativeBoundingFace(grandParentBoundingBox, functor);
                grandParentFace = (bbRotationInv * grandParentFace.transpose()).transpose();
                Eigen::Vector3f gp_bottom_left = findBottomLeft(grandParentFace, planeIndex);
                Eigen::Vector3f gp_top_right = findTopRight(grandParentFace, planeIndex);
                Eigen::Vector4f gp_boundingFaceTwoPoints(gp_bottom_left(planeIndex.first), gp_bottom_left(planeIndex.second), gp_top_right(planeIndex.first), gp_top_right(planeIndex.second));

                std::cout << "grandParentFace " << grandParentFace << std::endl;
                std::cout << "grandParentAltFace " << grandParentAltFace << std::endl;

                //Test to see if the grandparnt can be an anchor
                auto face1Match = false;
                //auto face2Match = true;
                for(auto j = 0; j < 4; j++)
                {
                    Eigen::Vector3f pt1 = grandParentFace.row(j);
                    Eigen::Vector3f pt2 = grandParentAltFace.row(j);
                    for(auto k = 0; k < 3; k++)
                    {
                        if(k != planeIndex.first && k != planeIndex.second)
                        {

                            auto e1 = std::fabs(pt1(k) - boundingFace(j,k));
                            qDebug() << "error1 " << e1;
                            if(e1 < 0.01)
                            {
                                face1Match = true;
                            }
                            auto e2 = std::fabs(pt2(k) - boundingFace(j, k));
                            qDebug() << "error2 " << e2;
                            if(e2 < 0.01)
                            {
                                 face1Match = true;
                            }


                        }
                    }
                }

                //qDebug() << "gp_boundingFaceTwoPoints " << gp_boundingFaceTwoPoints(0) << " " << gp_boundingFaceTwoPoints(1) << " " << gp_boundingFaceTwoPoints(2) << " " << gp_boundingFaceTwoPoints(3);
                if(face1Match)
                {
                     drawnBoxes.row(1) = gp_boundingFaceTwoPoints;
                     drawnBoxCount = 2;
                }
            }
        }

        auto isHack = false;
        if(id == 8)
        {
            auto grandParentBoundingBox = cluster39->getGrandParentBBox();

            if(grandParentBoundingBox.size() > 0)
            {
                //isHack = true;
                Eigen::MatrixXf grandParentFace = computeBoundingFace(grandParentBoundingBox, functor);
                //Eigen::MatrixXf grandParentAltFace = computeAlternativeBoundingFace(grandParentBoundingBox, functor);
                grandParentFace = (bbRotationInv * grandParentFace.transpose()).transpose();
                Eigen::Vector3f gp_bottom_left = findBottomLeft(grandParentFace, planeIndex);
                Eigen::Vector3f gp_top_right = findTopRight(grandParentFace, planeIndex);
                Eigen::Vector4f gp_boundingFaceTwoPoints(gp_bottom_left(planeIndex.first), gp_bottom_left(planeIndex.second), gp_top_right(planeIndex.first), gp_top_right(planeIndex.second));
                drawnBoxes.row(2) = gp_boundingFaceTwoPoints;
                drawnBoxCount = 3;
            }
        }

        int innerClusterSize = candidateClusters.size();
        if(firstClusterIsParent)
        {
            qDebug() << "firstClusterIsParent firstClusterIsParentId " << firstClusterIsParentId;
            for(auto& cluster : candidateClusters)
            {
                qDebug() << "cluster->clusterId() " << cluster->clusterId();
                if(firstClusterIsParentId == cluster->clusterId())
                {
                    innerClusterSize--;
                }
            }
        }

        Eigen::MatrixXf innerBoxes(4 * innerClusterSize, 3);
        Eigen::MatrixXf innerBoxesAllPoints(8 * innerClusterSize, 3);

        qDebug() << "innerClusterSize " << innerClusterSize;

        auto count = 0;
        auto allPointsCount = 0;
        for(auto& cluster : candidateClusters)
        {
            if(firstClusterIsParent && firstClusterIsParentId == cluster->clusterId())
            {
                continue;
            }
            auto bBox = cluster->getBBox();
            int allCount = 1;
            for(auto& point : bBox)
            {
                for(auto i = 0; i < 3; i++)
                    innerBoxesAllPoints(allPointsCount, i) = point[i];

                allPointsCount++;

                if(functor(allCount))
                {
                    for(auto i = 0; i < 3; i++)
                        innerBoxes(count, i) = point[i];

                    count++;
                }
                allCount++;
            }

        }

        //std::cout << "innner initial " << innerBoxes << std::endl;

        innerBoxes = (bbRotationInv * innerBoxes.transpose()).transpose();
        innerBoxesAllPoints = (bbRotationInv * innerBoxesAllPoints.transpose()).transpose();

        //    for(auto i = 0; i < 4 * clusters.size(); i++)
        //    {
        //        qDebug() << "innerBoxes " << innerBoxes(i, 0) << " " << innerBoxes(i, 1) << " " << innerBoxes(i, 2);
        //    }


        //Eigen::Vector4f boxAEigen(boundingFace(0,0), boundingFace(0,2), boundingFace(2,0), boundingFace(2,2));

        //qDebug() << "boxAEigen " << boxAEigen(0) << " " << boxAEigen(1) << " " << boxAEigen(2) << " " << boxAEigen(3);


        Eigen::Matrix2f drawnBoxCountMatrix = Eigen::Matrix2f::Zero();
        drawnBoxCountMatrix(0,0) = (float) drawnBoxCount;


        //engEvalString(engine->getEngine(), "figure, hold on; rectangle('Position', convertRectangleFromTwoPoints(boxA));");

        Eigen::MatrixXf boxesEigen(innerClusterSize, 4);
        for(auto i = 0; i < innerClusterSize; i++)
        {
            auto index = 4 * i;
            Eigen::MatrixXf face(4, 3);
            for(auto j = 0; j < 4; j++)
            {
                face.row(j) = innerBoxes.row(index+j);
            }

            Eigen::Vector3f bottom_left = findBottomLeft(face, planeIndex);
            Eigen::Vector3f top_right = findTopRight(face, planeIndex);

            boxesEigen.row(i) = Eigen::Vector4f(bottom_left(planeIndex.first), bottom_left(planeIndex.second), top_right(planeIndex.first), top_right(planeIndex.second));
        }


        //qDebug() << "drawnBoxes.rows() " << drawnBoxes.rows() << " drawnBoxes.cols() " << drawnBoxes.cols();
        mxArray *parentBoxMX = mxCreateNumericMatrix(1, 4, mxSINGLE_CLASS, mxREAL);
        mxArray *drawnBoxesMX = mxCreateNumericMatrix(drawnBoxes.rows(), 4, mxSINGLE_CLASS, mxREAL);
        mxArray *boxesMX = mxCreateNumericMatrix(boxesEigen.rows(), 4, mxSINGLE_CLASS, mxREAL);
        mxArray *drawnBoxCountMX = mxCreateNumericMatrix(2, 2, mxSINGLE_CLASS, mxREAL);

        memcpy(mxGetPr(drawnBoxesMX), drawnBoxes.data(), sizeof(float) * 4 * drawnBoxes.rows());
        memcpy(mxGetPr(boxesMX), boxesEigen.data(), sizeof(float) * 4 * boxesEigen.rows());
        memcpy(mxGetPr(drawnBoxCountMX), drawnBoxCountMatrix.data(), sizeof(float) * 4);
        memcpy(mxGetPr(parentBoxMX), boundingFaceTwoPoints.data(), sizeof(float) * 4);

        //qDebug() << "clusters.size() " << clusters.size();


//        for(auto i = 0; i < drawnBoxCount; i++)
//        {
//            std::cout << "drawnBoxes row " << i << " " << drawnBoxes.row(i) << std::endl;
//        }

//        std::cout << "boxesEigen " << boxesEigen << std::endl;
//        std::cout << "boundingFaceTwoPoints " << boundingFaceTwoPoints << std::endl;

        engPutVariable(engine->getEngine(), "drawnBoxes", drawnBoxesMX);
        engPutVariable(engine->getEngine(), "boxes", boxesMX);
        engPutVariable(engine->getEngine(), "drawnBoxesCount", drawnBoxCountMX);
        engPutVariable(engine->getEngine(), "parentBox", parentBoxMX);


        engEvalString(engine->getEngine(), "noDrawnBoxes = drawnBoxesCount(1,1);");

        //    engEvalString(engine->getEngine(), "openvar('parentBox');");
        //    engEvalString(engine->getEngine(), "openvar('drawnBoxes');");
        //    engEvalString(engine->getEngine(), "openvar('boxes');");

        engEvalString(engine->getEngine(), "[parentBoxNormalized, drawnBoxesNormalized, boxesNormalized] = normaliseBoxes(parentBox, drawnBoxes, boxes);");

        //    engEvalString(engine->getEngine(), "openvar('parentBoxNormalized');");
        //    engEvalString(engine->getEngine(), "openvar('drawnBoxesNormalized');");
        //    engEvalString(engine->getEngine(), "openvar('boxesNormalized');");

        //engEvalString(engine->getEngine(), "parentBoxNormalized = parentBox; drawnBoxesNormalized = drawnBoxes; boxesNormalized = boxes;");
        engEvalString(engine->getEngine(), "displayBoxes(drawnBoxesNormalized, noDrawnBoxes, boxesNormalized, [], 'Before');");

        //function [boxes_new_positions, box_ordering, all_lines, box_lines_size, all_lines_left, box_lines_left_size, all_lines_right, box_lines_right_size]
        engEvalString(engine->getEngine(), "[boxes_new_positions, box_ordering_x, all_lines_x, box_lines_size_x, all_lines_left_x, box_lines_left_size_x, all_lines_right_x, box_lines_right_size_x] = runOptimisation(parentBox, parentBoxNormalized, drawnBoxesNormalized, boxesNormalized, noDrawnBoxes, false);");
        //engEvalString(engine->getEngine(), "openvar('temp_mat');");
        engEvalString(engine->getEngine(), "displayBoxes(drawnBoxesNormalized, noDrawnBoxes, boxes_new_positions, all_lines_left_x, 'After Left X');");

        engEvalString(engine->getEngine(), "displayBoxes(drawnBoxesNormalized, noDrawnBoxes, boxes_new_positions, all_lines_x, 'After X');");

        engEvalString(engine->getEngine(), "swappedParentBox = swapXY(parentBox); swappedParentBoxNormalized= swapXY(parentBoxNormalized); swappedDrawnBoxesNormalized = swapXY(drawnBoxesNormalized); swappedBoxes = swapXY(boxes_new_positions);");
        if(isHack)
             engEvalString(engine->getEngine(), "[boxes_new_positions, box_ordering_y, all_lines_y, box_lines_size_y, all_lines_left_y, box_lines_left_size_y, all_lines_right_y, box_lines_right_size_y] = runOptimisation(swappedParentBox, swappedParentBoxNormalized, swappedDrawnBoxesNormalized, swappedBoxes, noDrawnBoxes, true);");
        else
            engEvalString(engine->getEngine(), "[boxes_new_positions, box_ordering_y, all_lines_y, box_lines_size_y, all_lines_left_y, box_lines_left_size_y, all_lines_right_y, box_lines_right_size_y] = runOptimisation(swappedParentBox, swappedParentBoxNormalized, swappedDrawnBoxesNormalized, swappedBoxes, noDrawnBoxes, false);");

        engEvalString(engine->getEngine(), "boxes_new_positions_all = swapXY(boxes_new_positions);");
        engEvalString(engine->getEngine(), "all_lines_y = swapXYLines(all_lines_y); all_lines_left_y = swapXYLines(all_lines_left_y); all_lines_right_y = swapXYLines(all_lines_right_y); all_lines = [all_lines_x; all_lines_y];");

        engEvalString(engine->getEngine(), "displayBoxes(drawnBoxesNormalized, noDrawnBoxes, boxes_new_positions_all, all_lines_left_y, 'After Left Y');");
        engEvalString(engine->getEngine(), "displayBoxes(drawnBoxesNormalized, noDrawnBoxes, boxes_new_positions_all, all_lines_right_y, 'After Right Y');");
        engEvalString(engine->getEngine(), "displayBoxes(drawnBoxesNormalized, noDrawnBoxes, boxes_new_positions_all, all_lines_y, 'After Y');");
        engEvalString(engine->getEngine(), "displayBoxes(drawnBoxesNormalized, noDrawnBoxes, boxes_new_positions_all, all_lines, 'After All');");
        //engEvalString(engine->getEngine(), "displayBoxes(drawnBoxesNormalized * 1.2, noDrawnBoxes, [], all_lines, 'Only Guides');");

        engEvalString(engine->getEngine(), "[boxes_new_positions_final] = deNormaliseBoxes(parentBox, boxes_new_positions_all);");
        engEvalString(engine->getEngine(), "all_lines_y = deNormaliseLines(parentBox, all_lines_y);");
        engEvalString(engine->getEngine(), "all_lines_x = deNormaliseLines(parentBox, all_lines_x);");
        engEvalString(engine->getEngine(), "all_lines_left_x = deNormaliseLines(parentBox, all_lines_left_x);");
        engEvalString(engine->getEngine(), "all_lines_left_y = deNormaliseLines(parentBox, all_lines_left_y);");
        engEvalString(engine->getEngine(), "all_lines_right_x = deNormaliseLines(parentBox, all_lines_right_x);");
        engEvalString(engine->getEngine(), "all_lines_right_y = deNormaliseLines(parentBox, all_lines_right_y);");

        //engEvalString(engine->getEngine(), "openvar('all_lines_right_y');");

        mxArray *newBoxes = engGetVariable(engine->getEngine(), "boxes_new_positions_final");
        mxArray *boxOrderingX = engGetVariable(engine->getEngine(), "box_ordering_x");
        mxArray *boxOrderingY = engGetVariable(engine->getEngine(), "box_ordering_y");
        mxArray *numberOfLinesX = engGetVariable(engine->getEngine(), "box_lines_size_x");
        mxArray *numberOfLinesY = engGetVariable(engine->getEngine(), "box_lines_size_y");
        mxArray *all_lines_x = engGetVariable(engine->getEngine(), "all_lines_x");
        mxArray *all_lines_y = engGetVariable(engine->getEngine(), "all_lines_y");
        mxArray *numberOfLinesTop = engGetVariable(engine->getEngine(), "box_lines_left_size_y");
        mxArray *numberOfLinesBottom = engGetVariable(engine->getEngine(), "box_lines_right_size_y");
        mxArray *numberOfLinesLeft = engGetVariable(engine->getEngine(), "box_lines_left_size_x");
        mxArray *numberOfLinesRight = engGetVariable(engine->getEngine(), "box_lines_right_size_x");
        mxArray *all_lines_top = engGetVariable(engine->getEngine(), "all_lines_left_y");
        mxArray *all_lines_bottom = engGetVariable(engine->getEngine(), "all_lines_right_y");
        mxArray *all_lines_left = engGetVariable(engine->getEngine(), "all_lines_left_x");
        mxArray *all_lines_right = engGetVariable(engine->getEngine(), "all_lines_right_x");

        double* dat;
        dat = mxGetPr(newBoxes);
        Eigen::Map<Eigen::MatrixXd > newBoxesEigen(dat, boxesEigen.rows(), 4);

        dat = mxGetPr(boxOrderingX);
        Eigen::Map<Eigen::VectorXd > boxOrderingXEigen(dat, boxesEigen.rows(), 1);

        dat = mxGetPr(boxOrderingY);
        Eigen::Map<Eigen::VectorXd > boxOrderingYEigen(dat, boxesEigen.rows(), 1);

        dat = mxGetPr(numberOfLinesX);
        Eigen::Map<Eigen::VectorXd > numberOfLinesXEigen(dat, boxesEigen.rows(), 1);

        dat = mxGetPr(numberOfLinesY);
        Eigen::Map<Eigen::VectorXd > numberOfLinesYEigen(dat, boxesEigen.rows(), 1);

        dat = mxGetPr(numberOfLinesTop);
        Eigen::Map<Eigen::VectorXd > numberOfLinesTopEigen(dat, boxesEigen.rows(), 1);

        dat = mxGetPr(numberOfLinesBottom);
        Eigen::Map<Eigen::VectorXd > numberOfLinesBottomEigen(dat, boxesEigen.rows(), 1);

        dat = mxGetPr(numberOfLinesLeft);
        Eigen::Map<Eigen::VectorXd > numberOfLinesLeftEigen(dat, boxesEigen.rows(), 1);

        dat = mxGetPr(numberOfLinesRight);
        Eigen::Map<Eigen::VectorXd > numberOfLinesRightEigen(dat, boxesEigen.rows(), 1);

        //auto total_no_lines = numberOfLinesXEigen.sum() + numberOfLinesYEigen.sum();

        dat = mxGetPr(all_lines_y);
        Eigen::Map<Eigen::MatrixXd > allLinesYEigen(dat, numberOfLinesYEigen.sum(), 4);

        dat = mxGetPr(all_lines_x);
        Eigen::Map<Eigen::MatrixXd > allLinesXEigen(dat, numberOfLinesXEigen.sum(), 4);

        dat = mxGetPr(all_lines_top);
        Eigen::Map<Eigen::MatrixXd > allLinesTopEigen(dat, numberOfLinesTopEigen.sum(), 4);

        dat = mxGetPr(all_lines_bottom);
        Eigen::Map<Eigen::MatrixXd > allLinesBottomEigen(dat, numberOfLinesBottomEigen.sum(), 4);

        dat = mxGetPr(all_lines_left);
        Eigen::Map<Eigen::MatrixXd > allLinesLeftEigen(dat, numberOfLinesLeftEigen.sum(), 4);



        dat = mxGetPr(all_lines_right);
        Eigen::Map<Eigen::MatrixXd > allLinesRightEigen(dat, numberOfLinesRightEigen.sum(), 4);

//        std::cout << "allLinesLeftEigen " << allLinesLeftEigen << std::endl;
//        std::cout << "allLinesRightEigen " << allLinesRightEigen << std::endl;
//        std::cout << "allLinesBottomEigen numberOfLinesBottomEigen.sum() " << numberOfLinesBottomEigen.sum() << " " << allLinesBottomEigen << std::endl;
//        std::cout << "allLinesTopEigen " << allLinesTopEigen << std::endl;

        //    std::cout << "boxOrderingXEigen " << boxOrderingXEigen << std::endl;
        //    std::cout << "boxOrderingYEigen " << boxOrderingYEigen << std::endl;
        //    std::cout << "numberOfLinesXEigen " << numberOfLinesXEigen << std::endl;
        //    std::cout << "numberOfLinesYEigen " << numberOfLinesYEigen << std::endl;
        //    std::cout << "allLinesEigen " << allLinesEigen << std::endl;

        for(auto i = 0; i < boxesEigen.rows(); i++)
        {
            clusterOrderingX.push_back(previousBoxWithDiffFunctorCount + (int) boxOrderingXEigen(i));
            clusterOrderingY.push_back(previousBoxWithDiffFunctorCount + (int) boxOrderingYEigen(i));
            guideCountX.push_back((int) numberOfLinesXEigen(i));
            guideCountY.push_back((int) numberOfLinesYEigen(i));


            guideCountLeft.push_back((int) numberOfLinesLeftEigen(i));
            guideCountRight.push_back((int) numberOfLinesRightEigen(i));
            guideCountTop.push_back((int) numberOfLinesTopEigen(i));
            guideCountBottom.push_back((int) numberOfLinesBottomEigen(i));
        }

        //This could be source of bug!!!!
        previousBoxWithDiffFunctorCount += boxesEigen.rows();

        for(auto i = 0; i < boxesEigen.rows(); i++)
        {
            if(funcorIndex == 0)
            {
                auto index = 8 * i;
                if(id == 7)
                      index = 8 * (boxesEigen.rows() - i - 1);

                for(auto j = 0; j < 2; j++)
                {
                    innerBoxesAllPoints(index+j,0) = newBoxesEigen(i, 0);
                    innerBoxesAllPoints(index+j,2) = newBoxesEigen(i, 1);
                }

                index += 2;
                for(auto j = 0; j < 2; j++)
                {
                    innerBoxesAllPoints(index+j,0) = newBoxesEigen(i, 0);
                    innerBoxesAllPoints(index+j,2) = newBoxesEigen(i, 3);
                }

                index += 2;
                for(auto j = 0; j < 2; j++)
                {
                    innerBoxesAllPoints(index+j,0) = newBoxesEigen(i, 2);
                    innerBoxesAllPoints(index+j,2) = newBoxesEigen(i, 3);
                }

                index += 2;
                for(auto j = 0; j < 2; j++)
                {
                    innerBoxesAllPoints(index+j,0) = newBoxesEigen(i, 2);
                    innerBoxesAllPoints(index+j,2) = newBoxesEigen(i, 1);
                }

            }
            else if(funcorIndex == 1)
            {
                auto index = 8 * i;
                //auto index = 8 * (boxesEigen.rows() - i - 1);
                //Eigen::Vector4f(bottom_left(planeIndex.first), bottom_left(planeIndex.second), top_right(planeIndex.first), top_right(planeIndex.second));

//                //Left = < 5
//                std::cout << "innerBoxesAllPoints " << innerBoxesAllPoints << std::endl;
//                std::cout << "boxesEigen " << boxesEigen << std::endl;
//                std::cout << "funcorIndex 1 not tested" << std::endl;

                innerBoxesAllPoints(index,1) = newBoxesEigen(i, 0);
                innerBoxesAllPoints(index+4,1) = newBoxesEigen(i, 0);
                innerBoxesAllPoints(index,2) = newBoxesEigen(i, 1);
                innerBoxesAllPoints(index+6,2) = newBoxesEigen(i, 1);
                index++;

                innerBoxesAllPoints(index,1) = newBoxesEigen(i, 2);
                innerBoxesAllPoints(index+4,1) = newBoxesEigen(i, 2);
                innerBoxesAllPoints(index,2) = newBoxesEigen(i, 1);
                innerBoxesAllPoints(index+6,2) = newBoxesEigen(i, 1);
                index++;

                innerBoxesAllPoints(index,1) = newBoxesEigen(i, 0);
                innerBoxesAllPoints(index+4,1) = newBoxesEigen(i, 0);
                innerBoxesAllPoints(index,2) = newBoxesEigen(i, 3);
                innerBoxesAllPoints(index+2,2) = newBoxesEigen(i, 3);
                index++;

                innerBoxesAllPoints(index,1) = newBoxesEigen(i, 2);
                innerBoxesAllPoints(index+4,1) = newBoxesEigen(i, 2);
                innerBoxesAllPoints(index,2) = newBoxesEigen(i, 3);
                innerBoxesAllPoints(index+2,2) = newBoxesEigen(i, 3);

            }
            else if(funcorIndex == 2)
            {
                //indexes 0 and 1
                //auto index = 8 * (boxesEigen.rows() - i - 1);
                auto index = 8 * i;

                innerBoxesAllPoints(index,0) = newBoxesEigen(i, 0);
                innerBoxesAllPoints(index,1) = newBoxesEigen(i, 1);

                innerBoxesAllPoints(index+2,0) = newBoxesEigen(i, 0);
                innerBoxesAllPoints(index+2,1) = newBoxesEigen(i, 1);

                innerBoxesAllPoints(index+1,0) = newBoxesEigen(i, 0);
                innerBoxesAllPoints(index+1,1) = newBoxesEigen(i, 3);

                innerBoxesAllPoints(index+3,0) = newBoxesEigen(i, 0);
                innerBoxesAllPoints(index+3,1) = newBoxesEigen(i, 3);

                innerBoxesAllPoints(index+6,0) = newBoxesEigen(i, 2);
                innerBoxesAllPoints(index+6,1) = newBoxesEigen(i, 1);

                innerBoxesAllPoints(index+4,0) = newBoxesEigen(i, 2);
                innerBoxesAllPoints(index+4,1) = newBoxesEigen(i, 1);

                innerBoxesAllPoints(index+7,0) = newBoxesEigen(i, 2);
                innerBoxesAllPoints(index+7,1) = newBoxesEigen(i, 3);

                innerBoxesAllPoints(index+5,0) = newBoxesEigen(i, 2);
                innerBoxesAllPoints(index+5,1) = newBoxesEigen(i, 3);
             }

            //qDebug() << "funcorIndex " << funcorIndex;

        }

        Eigen::MatrixXf alternativeBoundingFace = computeAlternativeBoundingFace(boundingBox, functor);

        Eigen::Vector3f innerBoxMean = innerBoxesAllPoints.colwise().mean();
//        Eigen::Vector3f currentMean = boundingFace.colwise().mean();
//        Eigen::Vector3f alternativeMean = alternativeBoundingFace.colwise().mean();

        //qDebug() << "innerBoxMean";
        //faceToHighlight.clear();

        bool isAlternativeFace = false;

        adjustedBBox = getBBox();
        auto diffToMoveChildren = 0.0;
        Eigen::Vector3f startValue(-1,-1,-1);
        for(auto i = 0; i < 3; i++)
        {
            if(i != planeIndex.first && i != planeIndex.second)
            {
                auto altDiff = std::fabs(innerBoxMean(i) - alternativeBoundingFace(0, i));
                auto currentDiff = std::fabs(innerBoxMean(i) - boundingFace(0, i));
                if(altDiff > currentDiff)
                {
                   qDebug() << "normal ";

                   startValue = boundingFace.row(0);
                   for(auto n : candidateClusters)
                   {
                       for(auto j : planeVertexOrder[funcorIndex])
                       {
                           Eigen::Vector3f pointOnPlane = (bbRotation * boundingFace.row(j).transpose()).transpose();
                           faceToHighlight.push_back(pointOnPlane);
                       }
                   }

                   //Snap to the top...
//                   for(auto k = 0; k < boxesEigen.rows(); k++)
//                   {
//                       auto index = k * 8;

//                       auto allCount = 1;
//                       for(auto j = 0; j < 8; j++)
//                       {
//                           if(!functor(allCount))
//                           {
//                               auto original = innerBoxesAllPoints(index+j, i);
//                               innerBoxesAllPoints(index+j, i) = startValue(i) ;
//                               diffToMoveChildren = innerBoxesAllPoints(index+j, i) - startValue(i);
//                           }
//                           allCount++;
//                       }
//                   }
                }
                else
                {
                    startValue = alternativeBoundingFace.row(0);
                    isAlternativeFace = true;
                    for(auto n : candidateClusters)
                    {
                        for(auto j : planeVertexOrder[funcorIndex])
                        {
                            Eigen::Vector3f pointOnPlane = (bbRotation * alternativeBoundingFace.row(j).transpose()).transpose();
                            faceToHighlight.push_back(pointOnPlane);
                        }
                    }
//                    //Snap to the bottom...
//                    for(auto k = 0; k < boxesEigen.rows(); k++)
//                    {
//                        auto index = k * 8;

//                        auto allCount = 1;
//                        for(auto j = 0; j < 8; j++)
//                        {
//                            if(functor(allCount))
//                            {
//                                auto original = innerBoxesAllPoints(index+j, i);
//                                innerBoxesAllPoints(index+j, i) = startValue(i) ;
//                                diffToMoveChildren = innerBoxesAllPoints(index+j, i) - startValue(i);
//                            }
//                            allCount++;
//                        }
//                    }
                }
            }
        }




        //qDebug() << "faceToHighlight.size() xx " << faceToHighlight.size();


        //ALL X GUIDES
        for(auto i = 0; i < numberOfLinesXEigen.sum(); i++)
        {
            Eigen::Vector4d row = allLinesXEigen.row(i);
            Eigen::Vector3f from = startValue;
            Eigen::Vector3f to = startValue;

            from(planeIndex.first) = row(0);
            from(planeIndex.second) = row(2);
            to(planeIndex.first) = row(1);
            to(planeIndex.second) = row(3);

            Eigen::Vector3f  from2 = (bbRotation * from);
            Eigen::Vector3f  to2 = (bbRotation * to);
            auto pair = std::make_pair(from2, to2);
            guidesX.push_back(pair);
        }

        //ALL Y GUIDES
        for(auto i = 0; i < numberOfLinesYEigen.sum(); i++)
        {
            Eigen::Vector4d row = allLinesYEigen.row(i);
            Eigen::Vector3f from = startValue;
            Eigen::Vector3f to = startValue;

            from(planeIndex.first) = row(0);
            from(planeIndex.second) = row(2);
            to(planeIndex.first) = row(1);
            to(planeIndex.second) = row(3);

            Eigen::Vector3f  from2 = (bbRotation * from);
            Eigen::Vector3f  to2 = (bbRotation * to);
            auto pair = std::make_pair(from2, to2);
            guidesY.push_back(pair);
        }

        //ALL Y GUIDES
        //qDebug() << "ALL Y GUIDES";
        for(auto i = 0; i < numberOfLinesYEigen.sum(); i++)
        {

            Eigen::Vector4d row = allLinesYEigen.row(i);

            Eigen::Vector3f from = startValue;
            Eigen::Vector3f to = startValue;

            from(planeIndex.first) = row(0);
            from(planeIndex.second) = row(2);
            to(planeIndex.first) = row(1);
            to(planeIndex.second) = row(3);

            Eigen::Vector3f  from2 = (bbRotation * from);
            Eigen::Vector3f  to2 = (bbRotation * to);
            auto pair = std::make_pair(from2, to2);
            guidesY.push_back(pair);
        }

        //ALL Left
        //qDebug() << "All Left";
        for(auto i = 0; i < numberOfLinesLeftEigen.sum(); i++)
        {
            Eigen::Vector4d row = allLinesLeftEigen.row(i);
            //qDebug() << row[0] << " " << row[1] << " "  << row[2] << " " << row[3];
            Eigen::Vector3f from = startValue;
            Eigen::Vector3f to = startValue;

            from(planeIndex.first) = row(0);
            from(planeIndex.second) = row(2);
            to(planeIndex.first) = row(1);
            to(planeIndex.second) = row(3);

            Eigen::Vector3f  from2 = (bbRotation * from);
            Eigen::Vector3f  to2 = (bbRotation * to);
            auto pair = std::make_pair(from2, to2);
            guidesLeft.push_back(pair);
        }

        //ALL Right
        //qDebug() << "All Right";
        for(auto i = 0; i < numberOfLinesRightEigen.sum(); i++)
        {
            Eigen::Vector4d row = allLinesRightEigen.row(i);
            //qDebug() << row[0] << " " << row[1] << " "  << row[2] << " " << row[3];
            Eigen::Vector3f from = startValue;
            Eigen::Vector3f to = startValue;

            from(planeIndex.first) = row(0);
            from(planeIndex.second) = row(2);
            to(planeIndex.first) = row(1);
            to(planeIndex.second) = row(3);

            Eigen::Vector3f  from2 = (bbRotation * from);
            Eigen::Vector3f  to2 = (bbRotation * to);
            auto pair = std::make_pair(from2, to2);
            guidesRight.push_back(pair);
        }

        //ALL Top
        //qDebug() << "All Top";
        for(auto i = 0; i < numberOfLinesTopEigen.sum(); i++)
        {
            Eigen::Vector4d row = allLinesTopEigen.row(i);
            //qDebug() << row[0] << " " << row[1] << " "  << row[2] << " " << row[3];
            Eigen::Vector3f from = startValue;
            Eigen::Vector3f to = startValue;

            from(planeIndex.first) = row(0);
            from(planeIndex.second) = row(2);
            to(planeIndex.first) = row(1);
            to(planeIndex.second) = row(3);

            Eigen::Vector3f  from2 = (bbRotation * from);
            Eigen::Vector3f  to2 = (bbRotation * to);
            auto pair = std::make_pair(from2, to2);
            guidesTop.push_back(pair);
        }

        //ALL Bottom
        //qDebug() << "All Bottom";
        for(auto i = 0; i < numberOfLinesBottomEigen.sum(); i++)
        {
            Eigen::Vector4d row = allLinesBottomEigen.row(i);
            //qDebug() << row[0] << " " << row[1] << " "  << row[2] << " " << row[3];
            Eigen::Vector3f from = startValue;
            Eigen::Vector3f to = startValue;

            from(planeIndex.first) = row(0);
            from(planeIndex.second) = row(2);
            to(planeIndex.first) = row(1);
            to(planeIndex.second) = row(3);

            Eigen::Vector3f  from2 = (bbRotation * from);
            Eigen::Vector3f  to2 = (bbRotation * to);
            auto pair = std::make_pair(from2, to2);
            guidesBottom.push_back(pair);
        }

        innerBoxes = (bbRotation * innerBoxes.transpose()).transpose();
        innerBoxesAllPoints = (bbRotation * innerBoxesAllPoints.transpose()).transpose();

//        count = 0;
//        for(auto& cluster : candidateClusters)
//        {

//            if(firstClusterIsParent && firstClusterIsParentId == cluster->clusterId())
//            {
//                continue;
//            }


//            auto bBox = cluster->getBBox();
//            for(auto& point : bBox)
//            {

//                for(auto i = 0; i < 3; i++)
//                    point[i] = innerBoxesAllPoints(count, i);

//                count++;
//            }

//            if(id == 39)
//                cluster->setBoundingBox(bBox);
//            else
//                cluster->setBoundingBox(bBox, functor, planeIndex, funcorIndex, false);

////            auto clusterBBox = cluster->getBBox();
////            //auto parentBBox = getBBox();
////            auto newClusterBBox = snapClosestFaces(boundingBox, clusterBBox, functor, planeIndex, funcorIndex);
////            cluster->setBoundingBox(newClusterBBox);
//        }
   }
}




auto MeshSegmentCluster::getLeftGuides(int i) -> std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >
{
//    qDebug() << "getLeftGuides "<< i;
//    qDebug() << "guideCountX.size() " << guideCountX.size();
    if(guideCountLeft.size() == 0)
    {
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > > returnVector;
        return returnVector;
    }

    auto startIndex = 0;
    for(auto j = 0; j < i; j++)
    {
        startIndex += guideCountLeft[j];
    }
//    qDebug() << "startIndex " << startIndex;
//    qDebug() << "guideCountX[i] " << guideCountX[i];

    auto endIndex = startIndex + guideCountLeft[i];
 //   qDebug() << "endIndex " << endIndex;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > > returnVector(guidesLeft.begin() + startIndex, guidesLeft.begin() + endIndex);
    return returnVector;
}

auto MeshSegmentCluster::getRightGuides(int i) -> std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >
{
    if(guideCountRight.size() == 0)
    {
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > > returnVector;
        return returnVector;
    }

    auto startIndex = 0;
    for(auto j = 0; j < i; j++)
    {
        startIndex += guideCountRight[j];
    }
    auto endIndex = startIndex + guideCountRight[i];
    auto first = guidesRight.begin() + startIndex;
    auto last = guidesRight.begin() + endIndex;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > > returnVector(first, last);
    return returnVector;
}

auto MeshSegmentCluster::getTopGuides(int i) -> std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >
{
    if(guideCountTop.size() == 0)
    {
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > > returnVector;
        return returnVector;
    }

    auto startIndex = 0;
    for(auto j = 0; j < i; j++)
    {
        startIndex += guideCountTop[j];
    }
    auto endIndex = startIndex + guideCountTop[i];
    auto first = guidesTop.begin() + startIndex;
    auto last = guidesTop.begin() + endIndex;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > > returnVector(first, last);
    return returnVector;
}



auto MeshSegmentCluster::getBottomGuides(int i) -> std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >
{
    if(guideCountBottom.size() == 0)
    {
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > > returnVector;
        return returnVector;
    }

    auto startIndex = 0;
    for(auto j = 0; j < i; j++)
    {
        startIndex += guideCountBottom[j];
    }
    auto endIndex = startIndex + guideCountBottom[i];
    auto first = guidesBottom.begin() + startIndex;
    auto last = guidesBottom.begin() + endIndex;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > > returnVector(first, last);
    return returnVector;
}

auto MeshSegmentCluster::getChildClusterId(int i) -> int
{
    auto index = clusterOrderingX[i] - 1; //Matlab to c++ indexing
//    if(firstClusterIsParent)
//    {
//        for(auto j = 0; j < clusters.size(); j++)
//        {
//            auto cluster = clusters[j];
//            if(cluster->clusterId() == firstClusterIsParentId && index >= j)
//            {
//                index++;
//            }
//        }
//    }

    auto cluster = clusters[index];
    return cluster->clusterId();
}

auto MeshSegmentCluster::getBoxFilename(int i) -> QString
{
    //qDebug() << "getBoxFilename " << i;
    auto index = clusterOrderingX[i] - 1; //Matlab to c++ indexing
   // qDebug() << "index " << index;
    if(firstClusterIsParent)
    {

        for(auto j = 0; j < clusters.size(); j++)
        {
            auto cluster = clusters[j];
            if(cluster->clusterId() == firstClusterIsParentId && index >= j)
            {
                index++;
            }
        }
    }
    //qDebug() << "index after " << index;
    auto cluster = clusters[index];
    auto filename = cluster->writeBox();
    return filename;
}

auto MeshSegmentCluster::getFaceToHighlight(int i) -> std::vector<Eigen::Vector3f>
{
    auto startIndex = i * 4;
    auto endIndex = startIndex + 4;
    std::vector<Eigen::Vector3f> sub_vec;
    std::copy(faceToHighlight.begin() + startIndex, faceToHighlight.begin() + endIndex, std::back_inserter(sub_vec));
    //qDebug() << "sub_vec.size() " << sub_vec.size();
    return sub_vec;
}



auto  MeshSegmentCluster::getChildFaceToHighlight(int i) -> std::vector<Eigen::Vector3f>
{
    auto index = clusterOrderingX[i] - 1; //Matlab to c++ indexing
    //qDebug() << "index " << index;
    if(firstClusterIsParent)
    {
       // qDebug() << "firstClusterIsParent firstClusterIsParentId " << firstClusterIsParentId;
        for(auto j = 0; j < clusters.size(); j++)
        {
            auto cluster = clusters[j];
            if(cluster->clusterId() == firstClusterIsParentId && index >= j)
            {
                index++;
            }
        }
    }
   // qDebug() << "index after " << index;
    auto cluster = clusters[index];
    auto plane = cluster->getFaceToHighlight(i);
    return plane;
}

auto MeshSegmentCluster::getChildFaceToDraw(int i) -> std::vector<Eigen::Vector3f>
{
    qDebug() << "getChildFaceToDraw " << i;
    qDebug() << "clusters.size() " << clusters.size();
    qDebug() << "noBoxesToDraw() " << noBoxesToDraw();
    for(auto x = 0; x < clusterOrderingX.size(); x++)
    {
        qDebug() << clusterOrderingX[x];
    }

    auto index = clusterOrderingX[i] - 1; //Matlab to c++ indexing

    qDebug() << "index " << index;
    if(firstClusterIsParent)
    {
        qDebug() << "firstClusterIsParent firstClusterIsParentId " << firstClusterIsParentId;
        for(auto j = 0; j < clusters.size(); j++)
        {
            auto cluster = clusters[j];
            if(cluster->clusterId() == firstClusterIsParentId && index >= j)
            {
                index++;
            }
        }
    }
    else
    {
       for(auto c : clusters)
       {
          qDebug() << "cluster id " << c->clusterId();
       }
       assert(noBoxesToDraw() == clusters.size());
    }
    qDebug() << "index after " << index;
    auto cluster = clusters[index];
    auto plane = cluster->getFaceToDraw();
    return plane;
}

auto MeshSegmentCluster::adjustBBox(std::vector<trimesh::point> currentBBox, std::vector<trimesh::point> newBBox, std::vector<trimesh::point> inner, std::function<bool(int)> functor, std::pair<int, int> planeIndexes, int functorIndex) -> std::vector<trimesh::point>
{
    //qDebug() << "MeshSegmentCluster::adjustBBox";
    Eigen::MatrixXf originalBoundingFace(4,3);
    Eigen::MatrixXf newBoundingFace(4,3);
    Eigen::MatrixXf innerBox(4,3);

    Eigen::MatrixXf allInnerBox(8,3);

    int count = 0;
    int allCount = 1;

    for(auto j = 0; j < 8; j++)
    {

        auto iP = inner[j];
        for(auto i = 0; i < 3; i++)
        {
            allInnerBox(j, i) = iP[i];
        }

        if(functor(allCount))
        {
            auto oP = currentBBox[j];
            auto nP = newBBox[j];

            for(auto i = 0; i < 3; i++)
            {
                originalBoundingFace(count, i) = oP[i];
                newBoundingFace(count, i) = nP[i];
                innerBox(count, i) = iP[i];
            }
            count++;
        }
        allCount++;
    }

    originalBoundingFace = (bbRotationInv * originalBoundingFace.transpose()).transpose();
    newBoundingFace = (bbRotationInv * newBoundingFace.transpose()).transpose();
    innerBox = (bbRotationInv * innerBox.transpose()).transpose();
    allInnerBox = (bbRotationInv * allInnerBox.transpose()).transpose();

//    std::cout << "originalBoundingFace " << originalBoundingFace << std::endl;
//    std::cout << "newBoundingFace " << newBoundingFace << std::endl;
//    std::cout << "innerBox " << innerBox << std::endl;

    Eigen::Vector3f bottom_left = findBottomLeft(originalBoundingFace, planeIndexes);
    Eigen::Vector3f top_right =   findTopRight(originalBoundingFace, planeIndexes);

    Eigen::Vector3f new_bottom_left = findBottomLeft(newBoundingFace, planeIndexes);
    Eigen::Vector3f new_top_right = findTopRight(newBoundingFace, planeIndexes);

    qDebug() << "bottom_left " << bottom_left(planeIndexes.first) << " " << bottom_left(planeIndexes.first) << " " << bottom_left(planeIndexes.second);
    qDebug() << "top_right " << top_right(planeIndexes.first) << " " << top_right(planeIndexes.first) << " " << top_right(planeIndexes.second);

    auto new_width = (new_top_right(planeIndexes.first) - new_bottom_left(planeIndexes.first));
    auto width = (top_right(planeIndexes.first) - bottom_left(planeIndexes.first));

    qDebug() << "new_width " << new_width << " old_width " << width;

    auto new_height = (new_top_right(planeIndexes.second) - new_bottom_left(planeIndexes.second));
    auto height = (top_right(planeIndexes.second) - bottom_left(planeIndexes.second));

    qDebug() << "new_height " << new_height << " old_height " << height;

    for(auto i = 0; i < 4; i++)
    {

        auto dist_from_left = (innerBox(i,planeIndexes.first) - bottom_left(planeIndexes.first));
        auto width_percentage =  dist_from_left / width;

        qDebug() << "width " << width << " dist_from_left " << dist_from_left << " percentage " << width_percentage;
        innerBox(i,planeIndexes.first) = new_bottom_left(planeIndexes.first) + new_width * width_percentage;

        auto dist_from_bottom = (innerBox(i,planeIndexes.second) - bottom_left(planeIndexes.second));
        auto height_percentage =  dist_from_bottom / height;

        qDebug() << "height " << height << " dist_from_bottom " << dist_from_bottom << " percentage " << height_percentage;


        innerBox(i,planeIndexes.second) = new_bottom_left(planeIndexes.second) + new_height * height_percentage;

    }


    if(functorIndex == 0)
    {

        allCount = 1;
        count = 0;
        for(auto i = 0; i < 8; i++)
        {
            allInnerBox(i, planeIndexes.first) = innerBox(count, planeIndexes.first);
            allInnerBox(i, planeIndexes.second) = innerBox(count, planeIndexes.second);
            if(functor(allCount))
            {
                count++;
            }
            allCount++;
        }

    }
    else if(functorIndex == 1)
    {

        auto index = 0;

//        std::cout << "innerBoxesAllPoints " << allInnerBox << std::endl;
//        std::cout << "boxesEigen " << boxesEigen << std::endl;
//        std::cout << "funcorIndex 1 not tested" << std::endl;

        allInnerBox(index,1) = innerBox(0, planeIndexes.first);;
        allInnerBox(index+4,1) = innerBox(0, planeIndexes.first);;
        allInnerBox(index,2) = innerBox(0, planeIndexes.second);
        allInnerBox(index+6,2) = innerBox(0, planeIndexes.second);
        index++;

        allInnerBox(index,1) = innerBox(1, planeIndexes.first);
        allInnerBox(index+4,1) = innerBox(1, planeIndexes.first);
        allInnerBox(index,2) = innerBox(1, planeIndexes.second);
        allInnerBox(index+6,2) = innerBox(1, planeIndexes.second);
        index++;

        allInnerBox(index,1) = innerBox(2, planeIndexes.first);
        allInnerBox(index+4,1) = innerBox(2, planeIndexes.first);
        allInnerBox(index,2) = innerBox(2, planeIndexes.second);
        allInnerBox(index+2,2) = innerBox(2, planeIndexes.second);
        index++;

        allInnerBox(index,1) = innerBox(3, planeIndexes.first);
        allInnerBox(index+4,1) = innerBox(3, planeIndexes.first);
        allInnerBox(index,2) = innerBox(3, planeIndexes.second);
        allInnerBox(index+2,2) = innerBox(3, planeIndexes.second);

    }
    else if(functorIndex == 2)
    {
        allInnerBox(0, planeIndexes.first) = innerBox(0, planeIndexes.first);
        allInnerBox(0, planeIndexes.second) = innerBox(0, planeIndexes.second);

        allInnerBox(2, planeIndexes.first) = innerBox(0, planeIndexes.first);
        allInnerBox(2, planeIndexes.second) = innerBox(0, planeIndexes.second);

        allInnerBox(1, planeIndexes.first) = innerBox(1, planeIndexes.first);
        allInnerBox(1, planeIndexes.second) = innerBox(1, planeIndexes.second);

        allInnerBox(3, planeIndexes.first) = innerBox(1, planeIndexes.first);
        allInnerBox(3, planeIndexes.second) = innerBox(1, planeIndexes.second);

        allInnerBox(6, planeIndexes.first) = innerBox(2, planeIndexes.first);
        allInnerBox(6, planeIndexes.second) = innerBox(2, planeIndexes.second);

        allInnerBox(4, planeIndexes.first) = innerBox(2, planeIndexes.first);
        allInnerBox(4, planeIndexes.second) = innerBox(2, planeIndexes.second);

        allInnerBox(7, planeIndexes.first) = innerBox(3, planeIndexes.first);
        allInnerBox(7, planeIndexes.second) = innerBox(3, planeIndexes.second);

        allInnerBox(5, planeIndexes.first) = innerBox(3, planeIndexes.first);
        allInnerBox(5, planeIndexes.second) = innerBox(3, planeIndexes.second);
     }


    allInnerBox = (bbRotation * allInnerBox.transpose()).transpose();

    count = 0;
    for(auto& p : inner)
    {
        p[0] = allInnerBox(count, 0);
        p[1] = allInnerBox(count, 1);
        p[2] = allInnerBox(count, 2);
        count++;
    }

    return inner;
}

auto MeshSegmentCluster::setBoundingBox(std::vector<trimesh::point> bBox) -> void
{
    adjustedBBox = bBox;
}


auto MeshSegmentCluster::writeAllMeshes() -> void
{
    if(meshSegment.size() != 0 && clusters.size() == 0)
    {
        meshSegment.writeMesh(id);
    }
    for(auto& cluster : clusters)
    {
        cluster->writeAllMeshes();
    }
}

auto MeshSegmentCluster::setBoundingBox(std::vector<trimesh::point> bBox, std::function<bool(int)> functor, std::pair<int, int> planeIndexes, int functorIndex, bool translateOnly) -> void
{

    auto currentBBox = getBBox();
    if(meshSegment.size() != 0 && clusters.size() == 0)
    {
        auto innerBoundingBox = meshSegment.getBBox();
        auto newInnerBoundingBox = adjustBBox(currentBBox, bBox, innerBoundingBox, functor, planeIndexes, functorIndex);
        meshSegment.setBoundingBox(newInnerBoundingBox, translateOnly, id);
    }

    for(auto& cluster : clusters)
    {
        auto innerBoundingBox = cluster->getBBox();
        auto newInnerBoundingBox = adjustBBox(currentBBox, bBox, innerBoundingBox, functor, planeIndexes, functorIndex);
        cluster->setBoundingBox(newInnerBoundingBox, functor, planeIndexes, functorIndex, true);

    }

    adjustedBBox = bBox;
//    std::vector<Eigen::Vector3f> faceToDrawTemp;
//    faceToDraw.clear();
//    int allCount = 1;
//    for(auto j = 0; j < 8; j++)
//    {
//        if(!functor(allCount))
//        {
//            auto p = adjustedBBox[j];
//            Eigen::Vector3f facePoint(p[0], p[1], p[2]);
//            faceToDrawTemp.push_back(facePoint);
//            //qDebug() << p[0] << " " << p[1] << " " << p[2];
//        }
//        allCount++;
//    }

//    faceToDraw.clear();
//    std::vector<std::vector<int>> planeVertexOrder;
//    planeVertexOrder.push_back(frontIndexOrder);
//    planeVertexOrder.push_back(leftIndexOrder);
//    planeVertexOrder.push_back(topIndexOrder);
//    //qDebug() << "functorIndex " << functorIndex;
//    for(auto j : planeVertexOrder[functorIndex])
//    {
//        faceToDraw.push_back(faceToDrawTemp[j]);
//        //qDebug() << "faceToDraw " << faceToDrawTemp[j][0] << " " << faceToDrawTemp[j][1] << " " << faceToDrawTemp[j][2];
//    }
    //qDebug() << "create faceToDraw.size() " << faceToDraw.size();

    //qDebug() << "adjustedBBox.size() " << adjustedBBox.size();
}

