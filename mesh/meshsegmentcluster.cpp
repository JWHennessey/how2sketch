#ifndef MESHSEGMENTCLUSTER_CPP
#define MESHSEGMENTCLUSTER_CPP

#include "meshsegmentcluster.h"

MeshSegmentCluster::MeshSegmentCluster()
    : MeshSegmentCluster(nullptr)
{

}

MeshSegmentCluster::MeshSegmentCluster(MeshSegmentCluster* parent)
    : _clusterId(-1)
    , parentCluster(parent)
    , hasParent(true)
    , numberOfChildClusters(0)
{
    if(parentCluster == nullptr) hasParent = false;

}


auto MeshSegmentCluster::create(QString directory,
                                QJsonArray jsonArray,
                                trimesh::point mean,
                                float max_coeff,
                                int& counterRef) -> void
{
//    counterRef++;
//    _clusterId = counterRef;

    for(auto it = jsonArray.begin(); it != jsonArray.end(); it++)
    {

       if(it->isObject())
       {
           auto object = it->toObject();
           if(object.find("object") != object.end())
           {
               auto objectIsString = object["object"].isString();
               if(objectIsString)
               {
                   clusters.push_back(new MeshSegmentCluster(this));
                   auto meshSegmentCluster = clusters.back();
                   meshSegmentCluster->create(directory, object["object"].toString(), mean, max_coeff, counterRef);

                   if(object.find("cluster") != object.end())
                   {
                       auto clusterArray = object["cluster"].toArray();
                       meshSegmentCluster->addClusters(directory, clusterArray, mean, max_coeff, counterRef);
                   }
               }
               else // Object is a an abstract cluster
               {
                   qDebug() << "Object is abstract group, need to implement";
               }
           }
       }
    }
    numberOfChildClusters = counterRef - _clusterId;

}

auto MeshSegmentCluster::create(QString directory, QString filename, trimesh::point mean,
            float max_coeff, int& counterRef) -> void
{
   counterRef++;
   _clusterId = counterRef;
   qDebug() << "MeshSegmentCluster::create(QString meshFilename, trimesh::point mean, float max_coeff)" << _clusterId;

   auto meshFilename = directory.append(filename);

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
    file.append(QString::number(_clusterId));
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

}

auto MeshSegmentCluster::addClusters(QString directory,
                                QJsonArray jsonArray,
                                trimesh::point mean,
                                float max_coeff,
                                int& counterRef) -> void
{
//    counterRef++;
//    id = counterRef;

    for(auto it = jsonArray.begin(); it != jsonArray.end(); it++)
    {

       if(it->isObject())
       {
           auto object = it->toObject();
           if(object.find("object") != object.end())
           {
               auto objectIsString = object["object"].isString();
               if(objectIsString)
               {
                   clusters.push_back(new MeshSegmentCluster(this));
                   auto meshSegmentCluster = clusters.back();
                   meshSegmentCluster->create(directory, object["object"].toString(), mean, max_coeff, counterRef);

                   if(object.find("cluster") != object.end())
                   {
                       auto clusterArray = object["cluster"].toArray();
                       meshSegmentCluster->addClusters(directory, clusterArray, mean, max_coeff, counterRef);
                   }
               }
               else // Object is a an abstract cluster
               {
                   qDebug() << "AddClusters: Object is abstract group, need to implement";
               }
           }
       }

    }
    numberOfChildClusters = counterRef - _clusterId;
}


auto MeshSegmentCluster::getAllSegments(std::vector<MeshSegment*>& segments) -> void
{
    if(meshSegment.size() > 0)
        segments.push_back(&meshSegment);

    for(auto& cluster : clusters)
    {
        cluster->getAllSegments(segments);
    }
}

auto MeshSegmentCluster::getCluster(const int id, int& counterRef) -> MeshSegmentCluster*
{
     counterRef++;
     qDebug() << "MeshSegmentCluster::getCluster " << id;
     qDebug() << "_clusterId " << _clusterId;
     if(_clusterId == id)
     {
        qDebug() << "Return this";
        return this;
     }
     else
     {
         //int childCount = 1;
         for(auto& cluster : clusters)
         {
             MeshSegmentCluster* returnedCluster = cluster->getCluster(id, counterRef);
             if(returnedCluster)
             {
                 qDebug() << "Not nullptr";
                 if(returnedCluster->clusterId() == id)
                 {
                    qDebug() << "Return child cluster ";
                    return returnedCluster;
                 }
             }

             //childCount++;
         }
     }
     qDebug() << "return nullptr counterRef " << counterRef;
     return nullptr;
}

auto MeshSegmentCluster::clusterId() -> int
{
    return _clusterId;
}

auto MeshSegmentCluster::clusterSize() -> int
{
    if(clusters.size() == 0)
    {
        return 1;
    }
    auto val = clusters.size() + 1;
//    if(hasParentCluster())
//    {
//        val++;
//    }
    return val;
}


auto MeshSegmentCluster::boundingBoxesSize() -> int
{
    return 8 * clusterSize();
}

auto MeshSegmentCluster::boundingBoxes() -> std::vector<trimesh::point>
{
    std::vector<trimesh::point> boxesToReturn;
//    if(hasParentCluster())
//    {
//        auto bBox = parentCluster->getBBox();
//        boxesToReturn.insert(std::end(boxesToReturn), std::begin(bBox), std::end(bBox));
//    }

    auto bBox = meshSegment.getBBox();
    boxesToReturn.insert(std::end(boxesToReturn), std::begin(bBox), std::end(bBox));

    for(auto& cluster : clusters)
    {
        auto bBox = cluster->getBBox();
        boxesToReturn.insert(std::end(boxesToReturn), std::begin(bBox), std::end(bBox));
    }

    return boxesToReturn;
}

auto MeshSegmentCluster::getBBox() -> std::vector<trimesh::point>
{
    qDebug() << "MeshSegmentCluster::getBBox()";
    if(adjustedBBox.size() > 0)
    {
        qDebug() << "MeshSegmentCluster::getBBox() adjustedBBox";
        return adjustedBBox;
    }
    return meshSegment.getBBox();
//    if(clusters.size() == 0)
//    {
//        qDebug() << "clusters.size() == 0";
//        return meshSegment.getBBox();
//    }
//    else
//    {
//        qDebug() << "clusters.size() " << clusters.size();
//        auto haveSeenEndNode = false;
//        std::vector<trimesh::point> bBoxes;
//        for(auto& cluster : clusters)
//        {
//            qDebug() << "loop";
//            //Only consider the next level of segments
//            auto bBox = cluster->getBBox();
//            auto isEndNode = cluster->clusterSize() == 1;
//            if(!haveSeenEndNode && isEndNode)
//            {
//                haveSeenEndNode = true;
//            }
//            else if(haveSeenEndNode && !isEndNode)
//            {
//                continue;
//            }
//            bBoxes.insert(std::end(bBoxes), std::begin(bBox), std::end(bBox));
//        }
//        qDebug() << "bBoxes.size() " << bBoxes.size();
//        MeshSegment tempSegment;
//        for(auto& point : bBoxes)
//        {
//            qDebug() << "point " << point[0] << " " << point[1] << " " << point[2];
//            tempSegment.addPoint(point);
//        }

//        tempSegment.computeBoundingBox(bbRotation, bbRotationInv);
//        auto tempBBox = tempSegment.getBBox();
//        qDebug() << "tempBBox.size() " << tempBBox.size();
//        return tempBBox;
//    }
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

auto MeshSegmentCluster::setBBoxRotations(Eigen::Matrix3f eigTrans, Eigen::Matrix3f eigTransInv) -> void
{
    bbRotation = eigTrans;
    bbRotationInv = eigTransInv;
    for(auto& cluster : clusters)
    {
        cluster->setBBoxRotations(bbRotation, bbRotationInv);
    }
}

auto MeshSegmentCluster::hasParentCluster() -> bool
{
    //qDebug() << "Possible bug - hasParentCluster()" << hasParent;
    //return (hasParent && clusters.size() != 0);
    return hasParent;
    //return false;
}

auto MeshSegmentCluster::getParentCluster() ->  MeshSegmentCluster*
{
    return parentCluster;
}

auto MeshSegmentCluster::runOptimisation(Eigen::Matrix<float, 200, 4>& drawnBoxes,
                     int& drawnBoxCount, MeshSegmentCluster* cluster39) -> void
{

}

auto MeshSegmentCluster::getLeftGuides(int i) -> std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >
{
    auto empty = std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >();
    return empty;
}

auto MeshSegmentCluster::getRightGuides(int i) -> std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >
{
    auto empty = std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >();
    return empty;
}

auto MeshSegmentCluster::getTopGuides(int i) -> std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >
{
    auto empty = std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >();
    return empty;
}

auto MeshSegmentCluster::getBottomGuides(int i) -> std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >
{
    auto empty = std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >();
    return empty;
}

auto MeshSegmentCluster::getBoxFilename(int i) -> QString
{
    return QString("Null");
}

auto MeshSegmentCluster::writeAllMeshes() -> void
{

}

auto MeshSegmentCluster::writeBox() -> QString
{
    return QString("Null");
}

auto MeshSegmentCluster::noBoxesToDraw() -> int
{
   return 0;
}

auto MeshSegmentCluster::getFaceToHighlight(int i) -> std::vector<Eigen::Vector3f>
{
    auto empty = std::vector<Eigen::Vector3f>();
    return empty;
}

auto MeshSegmentCluster::getNoChildClusters() -> int
{
    return numberOfChildClusters;
}


#endif
