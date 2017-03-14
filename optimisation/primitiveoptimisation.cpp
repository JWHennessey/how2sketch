#include "primitiveoptimisation.h"
#include <qdebug>
#include <set>
#include <cmath>
#include "gurobi_c++.h"
#include "optimisation/utility_functions.h"
#include <algorithm>
#include <QFileDialog>
#include <CGAL/ch_graham_andrew.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <list>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Point_2<K> Point;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef Polygon_2::Vertex_iterator VertexIterator;
typedef Polygon_2::Edge_const_iterator EdgeIterator;

using namespace utility;


typedef std::tuple<float, float, float, float, float, float> Vector6f;

float error_threshold = 0.10;

PrimitiveOptimisation::PrimitiveOptimisation(Mesh* m)
    : mesh(m)
{

}


auto makeHalfGuides(int axis, Eigen::Vector3f minCoeffs, Eigen::Vector3f maxCoeffs) -> std::vector<Guide*>
{
    std::vector<Guide*> guides;
    guides.push_back(new Guide(minCoeffs, maxCoeffs));

    Eigen::Vector3f from2;
    for(auto i = 0; i < 3; i++)
    {
        if(i == axis)
            from2(axis) = minCoeffs(axis);
        else
            from2(i) = maxCoeffs(i);
    }

    Eigen::Vector3f to2;
    for(auto i = 0; i < 3; i++)
    {
        if(i == axis)
            to2(axis) = maxCoeffs(axis);
        else
            to2(i) = minCoeffs(i);
    }
    guides.push_back(new Guide(from2, to2));

    return guides;
}

auto makeHorizontalGuide(int axis, Eigen::Vector3f minCoeffs, Eigen::Vector3f maxCoeffs) -> Guide*
{

    Eigen::Vector3f from;
    for(auto i = 0; i < 3; i++)
    {
        if(i == axis)
            from(axis) = (minCoeffs(i) + maxCoeffs(i)) * 0.5;
        else
            from(i) = minCoeffs(i);
    }

    Eigen::Vector3f to;
    for(auto i = 0; i < 3; i++)
    {
        if(i == axis)
            to(axis) = (minCoeffs(i) + maxCoeffs(i)) * 0.5;
        else
            to(i) = maxCoeffs(i);
    }
    auto g = new Guide(from, to);

    return g;
}

auto makeVerticalGuide(int axis, Eigen::Vector3f minCoeffs, Eigen::Vector3f maxCoeffs) -> Guide*
{

    Eigen::Vector3f from;
    for(auto i = 0; i < 3; i++)
    {
        if(i == axis)
            from(axis) = minCoeffs(i);
        else
            from(i) = (minCoeffs(i) + maxCoeffs(i)) * 0.5;
    }

    Eigen::Vector3f to;
    for(auto i = 0; i < 3; i++)
    {
        if(i == axis)
            to(axis) = maxCoeffs(i);
        else
            to(i) = (minCoeffs(i) + maxCoeffs(i)) * 0.5;
    }
    auto g = new Guide(from, to);

    return g;
}

const static float anchors[] = {-2.0, -1.0, -0.5, 0.0, 0.25, 0.333333, 0.5, 0.66666, 0.75, 1.0, 1.5, 2.0, 3.0};
const static int anchorsSize = 13;

auto makeGuides(int axis, Eigen::Matrix<float, 4, 3> parentFace, int guideType, Eigen::Matrix<float, 4, 3> childFace) -> Step*
{
    Eigen::Vector3f maxCoeffs = parentFace.colwise().maxCoeff();
    Eigen::Vector3f minCoeffs = parentFace.colwise().minCoeff();

    Eigen::Vector3f childMaxCoeffs = childFace.colwise().maxCoeff();
    Eigen::Vector3f childMinCoeffs = childFace.colwise().minCoeff();


    auto delta = maxCoeffs(axis) - minCoeffs(axis);
    auto candidate_point = minCoeffs(axis) + delta * anchors[guideType];

    auto step = new Step(guideType);

    if(guideType == 0) // -2x
    {

    }
    else if(guideType == 1) // -1x
    {




        auto halfGuides = makeHalfGuides(axis, minCoeffs, maxCoeffs);
        for(auto g : halfGuides)
            step->addGuide(g);




        Eigen::Vector3f new_start(minCoeffs);
        new_start(axis) = candidate_point - 0.05;
        auto verticalGuide = makeVerticalGuide(axis, new_start, maxCoeffs);
        step->addGuide(verticalGuide);


        Eigen::Vector3f new_start2(new_start);
        Eigen::Vector3f new_end2(minCoeffs);
        new_start2(0) = new_end2(0);
        new_end2(1) = maxCoeffs(1);

        step->addGuide(new Guide(new_start2, new_end2));

        new_start2(2) = maxCoeffs(2);
        new_end2(2) = maxCoeffs(2);


        step->addGuide(new Guide(new_start2, new_end2));

        new_start(axis) += 0.05;
        auto halfGuides2 = makeHalfGuides(axis, new_start, maxCoeffs);
        for(auto g : halfGuides2)
            step->addGuide(g);
    }
    else if(guideType == 2) // -0.5x
    {
        auto halfGuides1 = makeHalfGuides(axis, minCoeffs, maxCoeffs);
        for(auto g : halfGuides1)
            step->addGuide(g);

        auto horizontalGuide1 = makeHorizontalGuide(axis, minCoeffs, maxCoeffs);
        step->addGuide(horizontalGuide1);

        Eigen::Vector3f new_end(maxCoeffs);
        for(auto i = 0; i < 3; i++)
            if(i == axis)
                new_end(i) = (minCoeffs(i) + maxCoeffs(i)) * 0.5;

        Eigen::Vector3f new_start(minCoeffs);
        for(auto i = 0; i < 3; i++)
            if(i == axis)
                new_start(i) = minCoeffs(i) - (maxCoeffs(i) - minCoeffs(i)) * 0.5;



        Eigen::Vector3f new_start_appox(new_start);

        auto verticalGuide1 = makeVerticalGuide(axis, new_start_appox, maxCoeffs);
        step->addGuide(verticalGuide1);

        Eigen::Vector3f new_end2(maxCoeffs);
        for(auto i = 0; i < 3; i++)
            if(i != axis)
                new_end2(i) = minCoeffs(i);


        step->addGuide(new Guide(new_start_appox, new_end2));

        Eigen::Vector3f new_start2(maxCoeffs);
        for(auto i = 0; i < 3; i++)
            if(i == axis)
                new_start2(i) = minCoeffs(i) - (maxCoeffs(i) - minCoeffs(i)) * 0.5;


        Eigen::Vector3f new_start2_appox(new_start2);

        step->addGuide(new Guide(maxCoeffs, new_start2_appox));

        auto halfReflection = makeHalfGuides(axis, new_start, new_end);
        for(auto g : halfReflection)
            step->addGuide(g);


        step->addGuide(new Guide(new_start2, new_start));
    }
    else if(guideType == 3) //Left edge
    {

    }
    else if(guideType == 4) // 1/4
    {
        Eigen::Vector3f new_end0(maxCoeffs);
        new_end0(0) = new_end0(0) + (new_end0(0) - minCoeffs(0)) ;

        auto horizontalGuide1 = makeHorizontalGuide(axis, minCoeffs, new_end0);
        step->addGuide(horizontalGuide1);

        Eigen::Vector3f new_end(maxCoeffs);
        for(auto i = 0; i < 3; i++)
            if(i == axis)
                new_end(i) = (minCoeffs(i) + maxCoeffs(i)) * 0.5;

        auto halfGuides = makeHalfGuides(axis, minCoeffs, new_end);
        for(auto g : halfGuides)
            step->addGuide(g);


        Eigen::Vector3f new_end2(maxCoeffs);
        Eigen::Vector3f new_start2(minCoeffs);
        for(auto i = 0; i < 3; i++)
            if(i == axis)
            {
                new_end2(i) = candidate_point;
                new_start2(i) = candidate_point;
            }


        new_end2(0) = new_end2(0) + (new_end2(0) - new_start2(0)) ;





    }
    else if(guideType == 5) // 1/3
    {

    }
    else if(guideType == 6) // 1/2
    {
        auto halfGuides = makeHalfGuides(axis, minCoeffs, maxCoeffs);
        for(auto g : halfGuides)
            step->addGuide(g);

        auto horizontalGuide = makeHorizontalGuide(axis, minCoeffs, maxCoeffs);
        step->addGuide(horizontalGuide);

    }
    else if(guideType == 7) // 2/3
    {
        auto halfGuides1 = makeHalfGuides(axis, minCoeffs, maxCoeffs);
        for(auto g : halfGuides1)
            step->addGuide(g);


        Eigen::Vector3f newstart(minCoeffs);
        newstart(axis) = (minCoeffs(axis) + maxCoeffs(axis)) * 0.5f;

        step->addGuide(new Guide(newstart, maxCoeffs));


        Eigen::Vector3f newEnd(maxCoeffs);
        newstart(axis) = candidate_point;
        newEnd(axis) = candidate_point;
        step->addGuide(new Guide(newstart, newEnd));


    }
    else if(guideType == 8) // 3/4
    {
        auto halfGuides1 = makeHalfGuides(axis, minCoeffs, maxCoeffs);
        for(auto g : halfGuides1)
            step->addGuide(g);

        auto horizontalGuide1 = makeHorizontalGuide(axis, minCoeffs, maxCoeffs);
        step->addGuide(horizontalGuide1);

        Eigen::Vector3f new_start(maxCoeffs);
        for(auto i = 0; i < 3; i++)
            if(i == axis)
                new_start(i) = (minCoeffs(i) + maxCoeffs(i)) * 0.5;

        auto halfGuides = makeHalfGuides(axis, new_start, maxCoeffs);
        for(auto g : halfGuides)
            step->addGuide(g);


        Eigen::Vector3f new_end2(maxCoeffs);
        Eigen::Vector3f new_start2(minCoeffs);
        for(auto i = 0; i < 3; i++)
            if(i == axis)
            {
                new_end2(i) = candidate_point;
                new_start2(i) = candidate_point;
            }


        new_start2(1) =  new_start2(1)  - (new_end2(1) - new_start2(1)) * 0.5;

        auto horizontalGuide = makeHorizontalGuide(axis, new_start2, new_end2);
        step->addGuide(horizontalGuide);

    }
    else if(guideType == 9) // Right edge
    {

    }
    else if(guideType == 10) // 0.5x
    {




    }
    else if(guideType == 11) // 1x
    {
        auto halfGuides = makeHalfGuides(axis, minCoeffs, maxCoeffs);
        for(auto g : halfGuides)
            step->addGuide(g);

        Eigen::Vector3f new_end(maxCoeffs);
        new_end(axis) = candidate_point;
        auto verticalGuide = makeVerticalGuide(axis, minCoeffs, new_end);
        step->addGuide(verticalGuide);

        auto halfGuides2 = makeHalfGuides(axis, minCoeffs, new_end);
        for(auto g : halfGuides2)
            step->addGuide(g);

        Eigen::Vector3f new_end_child3(childMinCoeffs);
                Eigen::Vector3f new_start_child3(childMaxCoeffs);
                new_end_child3(2) = (childMaxCoeffs(2) + childMinCoeffs(2)) * 0.5f;

        auto halfGuidesMid = makeHalfGuides(axis, new_end_child3, new_start_child3);
        for(auto g : halfGuidesMid)
            step->addGuide(g);

        Eigen::Vector3f new_start_child4(childMinCoeffs);

        auto verticalGuide2 = makeHorizontalGuide(axis, new_start_child4, childMaxCoeffs);
        step->addGuide(verticalGuide2);

        auto halfGuides3 = makeHalfGuides(axis, childMinCoeffs, childMaxCoeffs);
        for(auto g : halfGuides3)
            step->addGuide(g);


    }
    else if(guideType == 12) // 2x
    {

    }
    else if(guideType == 13)
    {

        Eigen::Vector3f fl(0.25565463304519653, -0.36276164650917053, 0.33699986338615417);
        Eigen::Vector3f fr(-0.2586311399936676, -0.36276164650917053, 0.33699986338615417);
        Eigen::Vector3f bl(0.25565463304519653, -0.36276164650917053, -0.62257999181747437);
        Eigen::Vector3f br(-0.2586311399936676, -0.36276164650917053, -0.62257999181747437);

        auto backMid = (bl + br) * 0.5;
        auto frontMid = (fl + fr) * 0.5;

        auto top_height = 0.59;

        step->addGuide(new Guide(bl, fr));
        step->addGuide(new Guide(br, fl));
        step->addGuide(new Guide(backMid, frontMid));

        auto backMidTop = Eigen::Vector3f(backMid);
        backMidTop(1) = top_height;
        step->addGuide(new Guide(backMid, backMidTop));

        Eigen::Vector3f frontBottom(frontMid(0),-0.36276164650917053,0.240463);
        Eigen::Vector3f frontTop(frontMid(0), top_height, 0.240463);
        step->addGuide(new Guide(frontBottom, frontTop));

        step->addGuide(new Guide(frontTop, backMidTop));
        frontTop(1) = 0.23;
        backMidTop(1) = 0.23;

        step->addGuide(new Guide(frontTop, backMidTop));
    }


    return step;
}


auto computeProjectedSurfaceArea(Eigen::Matrix4f viewport, Eigen::Matrix4f mvp, Eigen::Matrix<float, 4,3> face) -> float
{
    //qDebug() << "Polygon points----------";
    std::vector<Point> p;
    for(auto i = 0; i < 4; i++)
    {
        Eigen::Vector4f proj = mvp * Eigen::Vector4f(face(i,0), face(i,1), face(i,2), 1);
        proj /= proj(3);
        proj = viewport * proj;
        p.push_back(Point(proj(0),proj(1)));
    }
    std::vector<Point> out;
    CGAL::ch_graham_andrew(p.begin() , p.end(), std::back_inserter(out));
    Polygon_2 ch;
    for(auto point : out)
    {
        ch.push_back(point);
    }
    auto area = ch.area();
    const static Eigen::Vector3f camera(camera_pos[0], camera_pos[1], camera_pos[2]);
    Eigen::Vector3f  mean = face.colwise().mean();
    Eigen::Vector3f delta = mean - camera;
    auto dist = std::fabs(delta.norm()) * 10.0;
    //qDebug() << "dist " << dist;
    return area - dist;
}

auto generatePartialPrimitives(PlanarRelation* relation, QMatrix4x4 viewport, QMatrix4x4 mvp, bool isMixerHack = false) -> std::vector<PlanePartialPrimitive*>
{

    auto partialPrimitves = std::vector<PlanePartialPrimitive*>();
    qDebug() << "relation->getFromSegment()";
    auto parentMeshSegment = relation->getFromSegment();
    qDebug() << "relation->getFace()";
    auto parentFace = parentMeshSegment->getFace(relation->getFromAxisId(), relation->getFromFaceId());

    //parentFace = reOrderPoints(parentFace);

    auto childMeshSegment = relation->getToSegment();
    auto childFace = childMeshSegment->getFace(relation->getToAxisId(), relation->getToFaceId());
    //childFace = reOrderPoints(childFace);

    qDebug() << "Partial Primitives " << parentMeshSegment->getName() << " to " << childMeshSegment->getName();

    Eigen::Vector3f parentMaxCoeffs = parentFace.colwise().maxCoeff();
    Eigen::Vector3f parentMinCoeffs = parentFace.colwise().minCoeff();

    Eigen::Vector3f childMaxCoeffs = childFace.colwise().maxCoeff();
    Eigen::Vector3f childMinCoeffs = childFace.colwise().minCoeff();


    Eigen::Matrix4f eigenMVP(mvp.data());
    Eigen::Matrix4f eigenViewport(viewport.data());
    auto projectedSurfaceArea = computeProjectedSurfaceArea(eigenViewport, eigenMVP, parentFace);

//    std::cout << parentFace << std::endl;
//    std::cout << parentMaxCoeffs << std::endl;
//    std::cout << parentMinCoeffs << std::endl;

    //qDebug() << "--------------------";
    auto planeIndex = -1;
    auto candidatePositions = std::vector<std::vector<float> >();
    auto candidatePositionsDuplicates = std::vector<std::set<float> >();
    //Generate candidate min/max positions for each axis
    for(auto i = 0; i < 3; i++)
    {
        candidatePositions.push_back(std::vector<float>());
        candidatePositionsDuplicates.push_back(std::set<float>());
        if(parentMinCoeffs(i) == parentMaxCoeffs(i))
        {
            //qDebug() << "parentMinCoeffs(i) == parentMaxCoeffs(i) " << parentMinCoeffs(i);
            planeIndex = i;
            continue;
        }

        for(auto j = 0; j < anchorsSize; j++)
        {
            auto delta = parentMaxCoeffs(i) - parentMinCoeffs(i);
            auto candidate_point = parentMinCoeffs(i) + delta * anchors[j];
            auto ret = candidatePositionsDuplicates[i].insert(candidate_point);

            if(ret.second) //Item newly inserted
                candidatePositions[i].push_back(candidate_point);
            else
                candidatePositions[i].push_back(-100.0f);
        }
    }

    //combine candidate min/max into axis partial primitives
    auto candidateAxisPartialPrimitives = std::vector<std::vector<AxisPartialPrimitive*>* >();
    for(auto i = 0; i < 3; i++)
    {

        auto partialPrimitives = new std::vector<AxisPartialPrimitive*>();

        auto unguidedAxis =
            new AxisPartialPrimitive(i, childMinCoeffs(i), childMaxCoeffs(i), childMeshSegment->getName());

        partialPrimitives->push_back(unguidedAxis);
        candidateAxisPartialPrimitives.push_back(partialPrimitives);

        if(planeIndex == i)
            continue;

        auto cPos = candidatePositions[i];

        //qDebug() << "cPos.size()  " << cPos.size();
        for(auto j = 0; j < cPos.size(); j++)
        {
            if(cPos[j] == -100.0f)
                continue;

            //qDebug() << cPos[j];
            for(auto k = (j + 1); k < cPos.size(); k++)
            {
                if(cPos[k] == -100.0f)
                    continue;

                //qDebug() << cPos[j] << " " << cPos[k];
                auto oldWidth = std::fabs(childMaxCoeffs(i) - childMinCoeffs(i));
                auto oldMid = (childMaxCoeffs(i) + childMinCoeffs(i)) * 0.5;

                auto newWidth = std::fabs(cPos[j] - cPos[k]);
                auto newMid = (cPos[j] + cPos[k]) * 0.5;

                auto midThreshold = oldWidth * error_threshold;
                auto widthThreshold = oldWidth *error_threshold;

                auto midDelta = std::fabs(oldMid - newMid);
                auto widthDelta = std::fabs(oldWidth - newWidth);


                if(midDelta > midThreshold || widthDelta > widthThreshold)
                    continue;

                auto min = 0.0f;
                auto max = 0.0f;
                if(cPos[j] > cPos[k])
                {
                    max = cPos[j];
                    min = cPos[k];
                }
                else
                {
                    max = cPos[k];
                    min = cPos[j];
                }

                auto minStep = makeGuides(i, parentFace, j, childFace);
                auto maxStep = makeGuides(i, parentFace, k, childFace);

                if(projectedSurfaceArea <= 0.0)
                    continue;

                //qDebug() << projectedSurfaceArea;
                //qDebug() << "AxisPartialPrimitive " << i << " min " << min << " max " << max;
                auto axisPP =
                        new AxisPartialPrimitive(i, min, max, j, k, minStep, maxStep, parentMeshSegment->getName(), relation->getFromAxisId(), relation->getFromFaceId(), projectedSurfaceArea, childMeshSegment->getName(), relation->getToAxisId(), relation->getToFaceId(), midDelta / midThreshold, widthDelta / widthThreshold);

                candidateAxisPartialPrimitives[i]->push_back(axisPP);
            }
        }
    }

    //combime the axis partial primitives to become partial plane primitives
    for(auto axis1Index = 0; axis1Index < 3; axis1Index++)
    {
        if(axis1Index == planeIndex)
            continue;

        for(auto axis2Index = (axis1Index + 1); axis2Index < 3; axis2Index++)
        {
            if(axis2Index == planeIndex)
                continue;

            auto axis1Candidates = candidateAxisPartialPrimitives[axis1Index];
            auto axis2Candidates = candidateAxisPartialPrimitives[axis2Index];


//            qDebug() << "axis1Candidates.size() " << axis1Candidates->size();
//            qDebug() << "axis2Candidates.size() " << axis2Candidates->size();
            for(auto axis1Candidate : *axis1Candidates)
            {
                for(auto axis2Candidate : *axis2Candidates)
                {

                    if(axis1Candidate->isUnguided() && axis2Candidate->isUnguided())
                        continue;

                    if(axis1Candidate->getAxis() != axis2Candidate->getAxis() && axis1Candidate->getCandidateTypeName() == axis2Candidate->getCandidateTypeName())
                    {

                        //qDebug() << "parentMinCoeffs(planeIndex) " << parentMinCoeffs(planeIndex) << " parentMaxCoeffs(planeIndex) " << parentMaxCoeffs(planeIndex);
                        auto ppp = new PlanePartialPrimitive(axis1Candidate, axis2Candidate, planeIndex, parentMinCoeffs(planeIndex), relation->isMidPlane());
                        //qDebug() << "ppp plane value " << ppp->getPlaneValue();
                        partialPrimitves.push_back(ppp);
                    }
                }
            }
        }
    }

//    qDebug() << parentMeshSegment->getName() << " from axis " << relation->getFromAxisId() << " from face " << relation->getFromFaceId();
//    qDebug() << childMeshSegment->getName() << " to axis " << relation->getToAxisId() << " to face " << relation->getToFaceId();

//    for(auto i = 0; i < 4; i++)
//        qDebug() << "parent " << parentFace(i,0) << " " << parentFace(i,1) << " " << parentFace(i,2) << " child " << childFace(i,0) << " " << childFace(i,1) << " " << childFace(i,2);

    return partialPrimitves;
}

auto createTruncatedPyramidPoints(int planeIndex, PlanePartialPrimitive* a, PlanePartialPrimitive* b) -> std::vector<trimesh::point>
{




    auto pvA = a->getPlaneValue();
    auto pvB = b->getPlaneValue();

    auto smallPoints = std::vector<trimesh::point>();
    auto largePoints = std::vector<trimesh::point>();

    auto min = 0;
    auto max = 0;
    if(pvA < pvB)
    {
         min = pvA;
         max = pvB;
         smallPoints = a->getPoints();
         largePoints = b->getPoints();
    }
    else
    {
        min = pvB;
        max = pvA;
        smallPoints = b->getPoints();
        largePoints = a->getPoints();
    }

    auto outputPoints = std::vector<trimesh::point>();

    for(auto i = 0; i < 4; i++)
    {
        outputPoints.push_back(smallPoints[i]);
        outputPoints.push_back(largePoints[i]);
    }

    return outputPoints;

}

auto createCompleteCandidate(MeshSegment* ms, PlanePartialPrimitive* a, PlanePartialPrimitive* b) -> CandidatePrimitive*
{

    //qDebug() << "createCompleteCandidate " << ms->getName();

    AxisPartialPrimitive* x = nullptr;
    AxisPartialPrimitive* y = nullptr;
    AxisPartialPrimitive* z = nullptr;

    std::vector<AxisPartialPrimitive*> axisPrimitvesA = {a->getAxisPrimitiveA(), a->getAxisPrimitiveB()};
    std::vector<AxisPartialPrimitive*> axisPrimitvesB = {b->getAxisPrimitiveA(), b->getAxisPrimitiveB()};

    for(auto apA : axisPrimitvesA)
    {
        for(auto apB : axisPrimitvesB)
        {
              if(apA->getAxis() == 0 && apA->getAxis() == apB->getAxis())
              {
                    x = apA;
                    x->merge(apB);
              }
              else if(apA->getAxis() == 1 && apA->getAxis() == apB->getAxis())
              {
                  y = apB;
                  y->merge(apA);
              }
              else if(apA->getAxis() == 2 && apA->getAxis() == apB->getAxis())
              {
                  z = apA;
                  z->merge(apB);
              }
        }
    }

    auto pvA = a->getPlaneValue();
    auto pvB = b->getPlaneValue();
    auto planeIndex = a->getPlaneIndex();
    auto min = 0;
    auto max = 0;
    if(pvA < pvB)
    {
         min = pvA;
         max = pvB;
    }
    else
    {
        min = pvB;
        max = pvA;
    }

    auto unguidedAxis = new AxisPartialPrimitive(planeIndex, min, max, ms->getName());

    auto points = createTruncatedPyramidPoints(planeIndex, a, b);

    if(planeIndex == 0)
    {
        x = unguidedAxis;
        //points = createTruncatedPyramidPoints(planeIndex, y, z);
    }
    else if(planeIndex == 1)
    {
        y = unguidedAxis;
        //points = createTruncatedPyramidPoints(planeIndex, x, z);
    }
    else if(planeIndex == 2)
    {
        z = unguidedAxis;
        //points = createTruncatedPyramidPoints(planeIndex, x, y);
    }

    assert(x != nullptr && y != nullptr && z != nullptr );


    auto completeCandidate = new CandidatePrimitive(ms, x, y, z);
    completeCandidate->setPoints(points);
    return completeCandidate;
}


auto createCompleteCandidate(MeshSegment* ms, AxisPartialPrimitive* a, AxisPartialPrimitive* b, AxisPartialPrimitive* c) -> CandidatePrimitive*
{

    //qDebug() << "createCompleteCandidate " << ms->getName();

    AxisPartialPrimitive* x = nullptr;
    AxisPartialPrimitive* y = nullptr;
    AxisPartialPrimitive* z = nullptr;

    std::vector<AxisPartialPrimitive*> axisPrimitves = {a,b,c};
    for(auto ap : axisPrimitves)
    {
        if(ap == nullptr)
        {
            qDebug() << "Nullptr";
        }

        //qDebug() << "loop";
        assert(ap != nullptr);
        //qDebug() << "ap->getAxis() " << ap->getAxis();

        if(ap->getAxis() == 0)
            x = ap;
        else if(ap->getAxis() == 1)
            y = ap;
        else if(ap->getAxis() == 2)
            z = ap;
    }

    assert(x != nullptr && y != nullptr && z != nullptr );

    auto completeCandidate = new CandidatePrimitive(ms, x,y,z);
    return completeCandidate;
}

auto isValidCocplanarRelation(PlanarRelation* relation, CandidatePrimitive* cp1, CandidatePrimitive* cp2) -> bool
{
    auto fromAxisId = relation->getFromAxisId();
    auto fromFaceId = relation->getFromFaceId();
    auto toAxisId = relation->getToAxisId();
    auto toFaceId = relation->getToFaceId();
    auto face1 = cp1->getFace(fromAxisId, fromFaceId);
    auto face2 = cp2->getFace(toAxisId, toFaceId);
    auto valid = hasCoPlanarRelation(face1, face2);

    return valid;
}

auto isValidCocplanarRelation(PlanarRelation* relation, MeshSegment* parent, CandidatePrimitive* cp2) -> bool
{
    auto fromAxisId = relation->getFromAxisId();
    auto fromFaceId = relation->getFromFaceId();
    auto toAxisId = relation->getToAxisId();
    auto toFaceId = relation->getToFaceId();
    auto face1 = parent->getFace(fromAxisId, fromFaceId);
    auto face2 = cp2->getFace(toAxisId, toFaceId);
    return hasCoPlanarRelation(face1, face2);
}


auto createCompletePrimitives(Mesh* mesh, std::map<QString, std::vector<PlanePartialPrimitive*>> planePrimitivesMap, std::map<QString, std::vector<AxisPartialPrimitive*>> unguidedAxisMap, std::map<QString, MeshSegment*> nameToMeshSegment) -> std::map<QString, std::vector<CandidatePrimitive*> >
{
    qDebug() << "createCompletePrimitives";
    auto candidatePrimitivesMap = std::map<QString, std::vector<CandidatePrimitive*> >();
    for(auto planePrimitives : planePrimitivesMap)
    {
        qDebug() << planePrimitives.first;
         auto completeCandidates = std::vector<CandidatePrimitive*>();
         //Add original primitive


         auto meshSegment = nameToMeshSegment[planePrimitives.first];

         if(!meshSegment->getIsTruncatedPyramid()) // Is plane or cuboid
         {

                 AxisPartialPrimitive* unguidedX = unguidedAxisMap[planePrimitives.first][0];
                 AxisPartialPrimitive* unguidedY = unguidedAxisMap[planePrimitives.first][1];
                 AxisPartialPrimitive* unguidedZ = unguidedAxisMap[planePrimitives.first][2];
                 auto originalPrimitive = createCompleteCandidate(meshSegment, unguidedX, unguidedY, unguidedZ);
                 completeCandidates.push_back(originalPrimitive);


                 qDebug() << planePrimitives.first << " maps to " << meshSegment->getName();

                 //qDebug() << "originalPrimitive added";

                 for(auto plane1 : planePrimitives.second)
                 {
                     for(auto plane2 : planePrimitives.second)
                     {
                         if(plane1->getPlaneIndex() == plane2->getPlaneIndex())
                                continue;


                         if(plane1->isMidPlane() && plane2->isMidPlane())
                                continue;

                         auto plane2A = plane2->getAxisPrimitiveA();
                         auto plane2B = plane2->getAxisPrimitiveB();
                         AxisPartialPrimitive* missingAxis = nullptr;
                         if(plane2A->getAxis() == plane1->getPlaneIndex())
                         {
                             missingAxis = plane2A;
                         }
                         else if(plane2B->getAxis() == plane1->getPlaneIndex())
                         {
                             missingAxis = plane2B;
                         }
                         else
                         {
                             continue;
                         }

        //                 if(missingAxis == nullptr)
        //                 {
        //                     qDebug() << "missingAxis == nullptr";
        //                 }
        //                 qDebug() << "Add missing axis to make primitive";

                         try
                         {
                            auto candidate = createCompleteCandidate(meshSegment, plane1->getAxisPrimitiveA(), plane1->getAxisPrimitiveB(), missingAxis);
                            completeCandidates.push_back(candidate);
                         }
                         catch(...)
                         {
                             qDebug() << "Error with creating complete candidate;";
                         }
                     }
                     //Add unguided missing axis
                     auto missingAxisIndex = plane1->getPlaneIndex();
                     auto unguidedAxis = unguidedAxisMap[planePrimitives.first][missingAxisIndex];

                     auto candidate = createCompleteCandidate(meshSegment, plane1->getAxisPrimitiveA(), plane1->getAxisPrimitiveB(), unguidedAxis);
                     completeCandidates.push_back(candidate);
                 }
                 //

         }
//         if(meshSegment->getIsPlane())
//         {
//             qDebug() << "Is Plane!!!";
//             assert(false);
//         }
         else // Is Truncated pyramid
         {
             qDebug() << "Is Truncated pyramid " << meshSegment->getName();
             for(auto plane1 : planePrimitives.second)
             {
                 for(auto plane2 : planePrimitives.second)
                 {
                     if(plane1->getPlaneIndex() == plane2->getPlaneIndex() && plane1->getPlaneValue() != plane2->getPlaneValue())
                     {

                         auto candidate = createCompleteCandidate(meshSegment, plane1, plane2);
                         completeCandidates.push_back(candidate);
                     }
                 }
             }
         }
         candidatePrimitivesMap.insert(std::make_pair(planePrimitives.first, completeCandidates));


    }


    std::map<QString, MeshSegment*> candidateToOriginal;

    for(auto candidatePair : candidatePrimitivesMap)
    {

        auto segments = mesh->getMeshSegments();
        for(auto i = size_t(0); i < segments.size(); i++)
        {
            if(segments[i]->getName() == candidatePair.first)
            {
                candidateToOriginal.insert(std::make_pair(candidatePair.first, segments[i]));
                //qDebug() << candidatePair.first << "Found parent";
                break; //Out of for loop
            }
        }
    }

     auto finalCandidatePrimitivesMap = std::map<QString, std::vector<CandidatePrimitive*> >();
    //Make sure candidates satify constraints with parents
    for(auto candidatePair : candidatePrimitivesMap)
    {
        auto completeCandidates = std::vector<CandidatePrimitive*>();
        for(auto candidate : candidatePair.second)
        {

            completeCandidates.push_back(candidate);
            continue;

        }
        qDebug() << "candidatePair.first " << candidatePair.first;
        finalCandidatePrimitivesMap.insert(std::make_pair(candidatePair.first, completeCandidates));
    }

    return finalCandidatePrimitivesMap;
}


auto PrimitiveOptimisation::generateCandidates(QMatrix4x4 viewport, QMatrix4x4 mvp) -> void
{
    qDebug() << "PrimitiveOptimisation::generateCandidates()";


    partialPrimitivesMap.clear();
    unguidedAxisMap.clear();
    candidatePrimitivesMap.clear();

    int secondLevelParentCount = 0;

    std::map<QString, MeshSegment*> nameToMeshSegment;

    auto segments = mesh->getMeshSegments();
    for(auto i = size_t(0); i < segments.size(); i++)
    {
        auto fromSegment = segments[i];
        auto fromPoints = fromSegment->getBBox();

        nameToMeshSegment.insert(std::make_pair(fromSegment->getName(), fromSegment));

        //The each of the unquided axis from the input
        Eigen::Matrix<float, 8, 3> eigenFromPoints;
        for(auto k = 0; k < fromPoints.size(); k++)
            for(auto l = 0; l < 3; l++)
                eigenFromPoints(k,l) = fromPoints[k][l];

        Eigen::Vector3f fromMaxCoeff = eigenFromPoints.colwise().maxCoeff();
        Eigen::Vector3f fromMinCoeff = eigenFromPoints.colwise().minCoeff();

        std::vector<AxisPartialPrimitive*> unguidedAxis;

        auto planeIndex = 0;
        for(auto a = size_t(0); a < 3; a++)
        {
            auto axis =
                new AxisPartialPrimitive(a, fromMinCoeff(a), fromMaxCoeff(a), fromSegment->getName());

            if(fromMinCoeff(a) == fromMaxCoeff(a))
                planeIndex = a;

            unguidedAxis.push_back(axis);
        }
        unguidedAxisMap.insert(std::make_pair(fromSegment->getName(), unguidedAxis));


        for(auto j = size_t(0); j < segments.size(); j++)
        {
            if(i == j)
                continue;

            auto toSegment = segments[j];

            auto coPlanarRelations = fromSegment->findCoPlanarRelations(toSegment);
            for(auto relation : coPlanarRelations)
            {
//                qDebug() << "from axis " << relation->getFromAxisId() << " from face " << relation->getFromFaceId();
//                qDebug() << "to axis " << relation->getToAxisId() << " to face " << relation->getToFaceId();
                  auto partialPrimitives = generatePartialPrimitives(relation, viewport, mvp);
                  auto it = partialPrimitivesMap.find(toSegment->getName());
                  if(it == partialPrimitivesMap.end()) //not seen
                  {
                       partialPrimitivesMap.insert(std::make_pair(toSegment->getName(), partialPrimitives));
                  }
                  else // seen
                  {
                       it->second.insert(std::end(it->second), std::begin(partialPrimitives), std::end(partialPrimitives));
                  }
            }
        }
        if(fromSegment->hasNoRelations())
        {
            std::vector<PlanePartialPrimitive*> partialPrimitives;
            if(planeIndex == 0)
            {
                auto ppp = new PlanePartialPrimitive(unguidedAxis[1], unguidedAxis[2], planeIndex, fromMinCoeff(planeIndex), false);
                partialPrimitives.push_back(ppp);
            }
            else if(planeIndex == 1)
            {
                auto ppp = new PlanePartialPrimitive(unguidedAxis[0], unguidedAxis[2], planeIndex, fromMinCoeff(planeIndex), false);
                partialPrimitives.push_back(ppp);
            }
            else if(planeIndex == 2)
            {
                auto ppp = new PlanePartialPrimitive(unguidedAxis[0], unguidedAxis[1], planeIndex, fromMinCoeff(planeIndex), false);
                partialPrimitives.push_back(ppp);
            }

            partialPrimitivesMap.insert(std::make_pair(fromSegment->getName(), partialPrimitives));
        }

    }
    //std::map<QString, std::vector<CandidatePrimitive*> >
    auto firstLevelandidatePrimitivesMap = createCompletePrimitives(mesh, partialPrimitivesMap, unguidedAxisMap, nameToMeshSegment);

    //partialPrimitivesMap.clear();

    for(auto candidatePair : firstLevelandidatePrimitivesMap)
    {
        auto candidateCount = 0;
        for(auto candidate : candidatePair.second)
        {
            for(auto j = size_t(0); j < segments.size(); j++)
            {
                auto toSegment = segments[j];
                if(toSegment->getName() == candidatePair.first) //Same Primitive
                    continue;

                auto candidateParents = candidate->getParentPrimitives();
                auto it = candidateParents.find(toSegment->getName());
                if(it != candidateParents.end()) //Is parent primitive
                    continue;

                if(candidate->isInputPrimitive())
                    continue;


                auto candidateName = QString(candidate->getPrimitiveName()).append(QString::number(candidateCount));
                candidate->setName(candidateName);
                candidate->createNewMeshSegment();
                auto coPlanarRelations = candidate->findCoPlanarRelations(toSegment);


                for(auto relation : coPlanarRelations)
                {

                      auto partialPrimitives = generatePartialPrimitives(relation, viewport, mvp);

                      for(auto pp : partialPrimitives)
                      {
                          auto points = pp->getPoints();
                          qDebug() << "pp plane index " << pp->getPlaneIndex();
                          for(auto pt : points)
                          {
                              qDebug() << pt[0] << " " << pt[1] << " " << pt[2];
                          }
                      }

                    auto parentMeshSegment = relation->getFromSegment();
                    auto parentFace = parentMeshSegment->getFace(relation->getFromAxisId(), relation->getFromFaceId());

                    auto childMeshSegment = relation->getToSegment();
                    auto childFace = childMeshSegment->getFace(relation->getToAxisId(), relation->getToFaceId());

                    std::cout << "parentFace " << parentFace << std::endl;
                    std::cout << "childFace " << childFace << std::endl;

                      qDebug() << "2nd Level Relation from " << relation->getFromSegment()->getName() << " to " << relation->getToSegment()->getName();

                      qDebug() << "2nd level primitives for " << toSegment->getName() << " " << partialPrimitives.size();

                      auto it = partialPrimitivesMap.find(toSegment->getName());
                      if(it == partialPrimitivesMap.end()) //not seen
                      {
                           partialPrimitivesMap.insert(std::make_pair(toSegment->getName(), partialPrimitives));
                      }
                      else // seen
                      {
                           it->second.insert(std::end(it->second), std::begin(partialPrimitives), std::end(partialPrimitives));
                      }
                 }

            }
            candidateCount++;
        }
    }
    candidatePrimitivesMap = createCompletePrimitives(mesh, partialPrimitivesMap, unguidedAxisMap, nameToMeshSegment);

}

//const static float anchors[] = {-2.0, -1.0, -0.5, 0.0, 0.25, 0.333333, 0.5, 0.66666, 0.75, 1.0, 1.5, 2.0, 3.0};
auto guideIdToText(int id) -> QString
{
    switch(id)
    {
        case 0: return QString("Extend 2X");
        case 1: return QString("Extend 1X");
        case 2: return QString("Extend 1/2X");
        case 3: return QString("Left Edge");
        case 4: return QString("Divide 1/4");
        case 5: return QString("Divide 1/3");
        case 6: return QString("Divide 1/2");
        case 7: return QString("Divide 2/3");
        case 8: return QString("Divide 3/4");
        case 9: return QString("Right Edge");
        case 10: return QString("Extend 1/2X");
        case 11: return QString("Extend 1X");
        case 12: return QString("Extend 2X");
        case 13: return QString("Align Midplanes");
        default: return QString("Error");
    }
}

std::set<Vector6f> previous_guides_u0;
std::set<Vector6f> previous_guides_u1;
std::set<Vector6f> previous_guides_u2;

auto createIndividualGuide(QJsonArray& json_guides, std::vector<Guide*> guides, int guideId, std::map<Vector6f, int>& pointToId, std::map<Vector6f, int>& pointToLastSeenIndex, AxisPartialPrimitive* axis, std::map<QString, CandidatePrimitive*> finalPrimitivesMap) -> QJsonObject
{
    int guideCount = json_guides.size();
//    if(guideCount > 0)
//        guideCount--;


    QJsonObject guide_sequence;
    guide_sequence["type"] = guideId;
    guide_sequence["type_text"] = guideIdToText(guideId);

    auto parentName = axis->getParentPrimitiveName();
    auto stringList = parentName.split(".ply");
    auto originalParentName = stringList[0].append(".ply");
    qDebug() << "originalParentName " << originalParentName;
    auto parentFaceId = axis->getParentPrimitiveFaceId();
    auto parentAxisId = axis->getParentPrimitiveAxisId();
    auto parentCandidate =  finalPrimitivesMap[originalParentName];
    auto parentFace = parentCandidate->getFace(parentAxisId, parentFaceId);
    QJsonArray faceToHighlight;
    std::vector<int> faceOrder = {0,1,3,2};

    for(auto i : faceOrder)
    {
        QJsonObject point;
        point["x"] = parentFace(i,1);
        point["y"] = parentFace(i,2);
        point["z"] = parentFace(i,0);
        faceToHighlight.push_back(point);
    }

    guide_sequence["face_to_highlight"] = faceToHighlight;


    auto childName = axis->getCandidateTypeName();
    stringList = childName.split(".ply");
    auto originalChildName = stringList[0].append(".ply");
    qDebug() << "originalChildName " << originalChildName;
    auto childFaceId = axis->getGuidesFaceId();
    auto childAxisId = axis->getGuidesAxisId();
    auto childCandidate =  finalPrimitivesMap[originalChildName];
    auto childFace = childCandidate->getFace(childAxisId, childFaceId);
    QJsonArray faceToDraw;
    for(auto i : faceOrder)
    {
        QJsonObject point;
        point["x"] = childFace(i,1);
        point["y"] = childFace(i,2);
        point["z"] = childFace(i,0);
        faceToDraw.push_back(point);
    }

    guide_sequence["face_to_draw"] = faceToDraw;

    QJsonArray u0;
    QJsonArray u1;
    QJsonArray u2;

    QJsonArray pg_u0;
    QJsonArray pg_u1;
    QJsonArray pg_u2;
    for(auto g : previous_guides_u0)
    {
        auto gId = pointToId[g];
        pg_u0.push_back(gId);
    }
    for(auto g : previous_guides_u1)
    {
        auto gId = pointToId[g];
        pg_u1.push_back(gId);
    }
    for(auto g : previous_guides_u2)
    {
        auto gId = pointToId[g];
        pg_u2.push_back(gId);
    }

    for(auto g : guides)
    {
        //qDebug() << " guideCount " << guideCount;
        auto to = g->getToPoint();
        auto from = g->getFromPoint();
        QJsonObject jsonFrom;
        jsonFrom["x"] = to[1];
        jsonFrom["y"] = to[2];
        jsonFrom["z"] = to[0];
        QJsonObject jsonTo;
        jsonTo["x"] = from[1];
        jsonTo["y"] = from[2];
        jsonTo["z"] = from[0];
        QJsonObject jsonGuidePair;
        jsonGuidePair["guide_id"] = guideCount;
        jsonGuidePair["from"] = jsonFrom;
        jsonGuidePair["to"] = jsonTo;
        json_guides.push_back(jsonGuidePair);

        Vector6f pair(to[1], to[2], to[0], from[1], from[2], from[0]);
        auto guideId = 0;
        auto it1 = pointToLastSeenIndex.find(pair);
        if(it1 == pointToLastSeenIndex.end())
        {
            Vector6f pair2(from[1], from[2], from[0], to[1], to[2], to[0]);
            guideId = pointToId[pair2];
        }
        else
        {
            guideId = pointToId[pair];
        }


        auto lastSeenId = pointToLastSeenIndex[pair];

        qDebug() << "guideID " <<  guideId << " guideCount " << guideCount << " lastSeen " << lastSeenId;

        if(guideId == guideCount)
        {
            auto difficulty = g->getUserDifficulty();
            if(difficulty >= 2)
            {
                u2.push_back(guideId);
                if(lastSeenId > guideCount)
                    previous_guides_u2.insert(pair);
            }

            if(difficulty >= 1)
            {
                u1.push_back(guideId);
                if(lastSeenId > guideCount)
                    previous_guides_u1.insert(pair);
            }

            if(difficulty >= 0)
            {
                u0.push_back(guideId);
                if(lastSeenId > guideCount)
                    previous_guides_u0.insert(pair);
            }
      }

         guideCount++;

    }


    QJsonObject U0;
    U0["previous_guides"] = pg_u0;
    U0["indexes"] = u0;
    QJsonObject U1;
    U1["previous_guides"] = pg_u1;
    U1["indexes"] = u1;
    QJsonObject U2;
    U2["previous_guides"] = pg_u2;
    U2["indexes"] = u2;

    guide_sequence["user0"] = U0;
    guide_sequence["user1"] = U1;
    guide_sequence["user2"] = U2;

    std::vector<Vector6f> toRemove;
    for(auto g : previous_guides_u0)
    {
        auto lastSeen = pointToLastSeenIndex[g];
        //qDebug() << "lastSeen " << lastSeen << " guideCount " << guideCount;
        if(lastSeen < guideCount)
        {
            toRemove.push_back(g);
        }
    }

    for(auto r : toRemove)
    {
        auto it = previous_guides_u0.find(r);
        previous_guides_u0.erase(it);
    }

    toRemove.clear();
    for(auto g : previous_guides_u1)
    {
        auto lastSeen = pointToLastSeenIndex[g];
        if(lastSeen < guideCount)
        {
            toRemove.push_back(g);
        }
    }

    for(auto r : toRemove)
    {
        auto it = previous_guides_u1.find(r);
        previous_guides_u1.erase(it);
    }

    toRemove.clear();
    for(auto g : previous_guides_u2)
    {
        auto lastSeen = pointToLastSeenIndex[g];
        if(lastSeen < guideCount)
        {
            toRemove.push_back(g);
        }
    }

    for(auto r : toRemove)
    {
        auto it = previous_guides_u2.find(r);
        previous_guides_u2.erase(it);
    }

    return guide_sequence;
}

auto guidesFromAxis(AxisPartialPrimitive* axis, QJsonArray& guides, QJsonArray& sequence, std::map<Vector6f, int>& pointToId, std::map<Vector6f, int>& pointToLastSeenIndex, std::map<QString, CandidatePrimitive*> finalPrimitivesMap) -> void
{
    if(axis->isUnguided())
        return;



    auto minGuides = axis->getMinGuides();
    if(minGuides.size() > 0)
    {
        auto s1 = createIndividualGuide(guides, minGuides, axis->getMinGuide(), pointToId, pointToLastSeenIndex, axis, finalPrimitivesMap);
        sequence.push_back(s1);
    }
    auto maxGuides = axis->getMaxGuides();
    if(maxGuides.size() > 0)
    {
        auto s2 = createIndividualGuide(guides, maxGuides, axis->getMaxGuide(), pointToId, pointToLastSeenIndex, axis, finalPrimitivesMap);
        sequence.push_back(s2);
    }

}


auto makeGuidesAndSequences(AxisPartialPrimitive* xAxis, AxisPartialPrimitive* yAxis, AxisPartialPrimitive* zAxis, std::map<QString, int> primitivesNameToId, QJsonArray& guides, std::map<Vector6f, int>& pointToId, std::map<Vector6f, int>& pointToLastSeenIndex,  std::map<QString, CandidatePrimitive*> finalPrimitivesMap) -> QJsonObject
{
    QJsonObject object;
    QJsonArray sequence;

    guidesFromAxis(xAxis, guides, sequence, pointToId, pointToLastSeenIndex, finalPrimitivesMap);
    guidesFromAxis(yAxis, guides, sequence, pointToId, pointToLastSeenIndex, finalPrimitivesMap);
    guidesFromAxis(zAxis, guides, sequence, pointToId, pointToLastSeenIndex, finalPrimitivesMap);

    object["guide_sequence"] = sequence;

    return object;
}


auto createGuideMaps(int& id, std::vector<Guide*> guides, std::map<Vector6f, int>& pointToId, std::map<Vector6f, int>& pointToLastSeenIndex) -> void
{
    for(auto g : guides)
    {
        auto to = g->getToPoint();
        auto from = g->getFromPoint();
        Vector6f pair1(to[1], to[2], to[0], from[1], from[2], from[0]);
        auto it1 = pointToId.find(pair1);
        Vector6f pair2(from[1], from[2], from[0],to[1], to[2], to[0]);
        auto it2 = pointToId.find(pair2);
        if(it1 == pointToId.end() && it2 == pointToId.end()) //First time it is seen
        {
            pointToId.insert(std::make_pair(pair1, id));
            pointToLastSeenIndex.insert(std::make_pair(pair1, id));
        }
        else if(it1 != pointToId.end())
        {
            pointToLastSeenIndex[pair1] = id;
        }
        else if(it2 != pointToId.end())
        {
            pointToLastSeenIndex[pair2] = id;
        }

        id++;
   }
}


auto createGuideMaps(int& id, AxisPartialPrimitive* axis, std::map<Vector6f, int>& pointToId, std::map<Vector6f, int>& pointToLastSeenIndex) -> void
{
    if(axis->isUnguided())
        return;

    auto minGuides = axis->getMinGuides();
    createGuideMaps(id, minGuides, pointToId, pointToLastSeenIndex);

    auto maxGuides = axis->getMaxGuides();
    createGuideMaps(id, maxGuides, pointToId, pointToLastSeenIndex);
}



auto createGuideMaps(int& id, AxisPartialPrimitive* xAxis, AxisPartialPrimitive* yAxis, AxisPartialPrimitive* zAxis, std::map<Vector6f, int>& pointToId, std::map<Vector6f, int>& pointToLastSeenIndex) -> void
{
    createGuideMaps(id, xAxis, pointToId, pointToLastSeenIndex);
    createGuideMaps(id, yAxis, pointToId, pointToLastSeenIndex);
    createGuideMaps(id, zAxis, pointToId, pointToLastSeenIndex);
}



auto PrimitiveOptimisation::findOrdering() -> void
{

    std::map<int, CandidatePrimitive*> primitivesMap;
    std::map<QString, int> primitivesNameToId;
    auto candidateCount = 0;
    for(auto pair : finalPrimitivesMap)
    {
        primitivesMap.insert(std::make_pair(candidateCount, pair.second));
        primitivesNameToId.insert(std::make_pair(pair.first, candidateCount));
        candidateCount++;
    }

    double  primitiveConnectivity[candidateCount][candidateCount]; //Hacky quadratic constraint matrix
    std::fill(primitiveConnectivity[0], primitiveConnectivity[0] + candidateCount * candidateCount, 0);


    for(auto child_id = 0; child_id < primitivesMap.size(); child_id++)
    {
        auto cp = primitivesMap[child_id];
        auto parentNames = cp->getParentPrimitives();
        for(auto parent : parentNames)
        {
            auto parentId = primitivesNameToId[parent];
            primitiveConnectivity[parentId][child_id] = 1;
        }
    }

    int drawnPrimitiveCount = 0;
    std::vector<int> sequence;
    while(drawnPrimitiveCount < candidateCount)
    {
        //primitiveConnectivity[parent][child]
        std::vector<int> candidateToDraw;
        for(auto child_id = 0; child_id < candidateCount; child_id++)
        {
            int countNoneDrawnParents = 0;
            for(auto parent_id = 0; parent_id < candidateCount; parent_id++)
            {
                //std::cout << primitiveConnectivity[parent_id][child_id] << " ";

                if(parent_id == child_id)
                    continue;

                if(primitiveConnectivity[parent_id][child_id] == 1)
                {
                    auto it = std::find(sequence.begin(), sequence.end(), parent_id);
                    if(it == sequence.end()) //Parent is not drawn
                        countNoneDrawnParents++;
                }

            }
            //std::cout << std::endl;
            auto it = std::find(sequence.begin(), sequence.end(), child_id);
            if(countNoneDrawnParents == 0 && it == sequence.end())
                candidateToDraw.push_back(child_id);
        }

        //NEED TO SETTLE TIES


        sequence.insert(std::end(sequence), std::begin(candidateToDraw), std::end(candidateToDraw));

        drawnPrimitiveCount+=candidateToDraw.size();


    }

    QString dir = QFileDialog::getExistingDirectory(nullptr, tr("Open Directory"),
                                                    "/Users/JamesHennessey/Dropbox/engd/multimodel_drawing_assistance/output_sequences/",
                                                    QFileDialog::ShowDirsOnly
                                                    | QFileDialog::DontResolveSymlinks);
    std::map<Vector6f, int> pointToId;
    std::map<Vector6f, int> pointToLastSeenIndex;
    int guideId = 0;


    for(auto i : sequence)
    {
        qDebug() << "Sequence i " << i;
        auto p = primitivesMap[i];
        createGuideMaps(guideId, p->getXAxis(), p->getYAxis(), p->getZAxis(), pointToId, pointToLastSeenIndex);
    }

    QJsonArray jsonSequence;
    QJsonArray jsonGuides;

    int sequenceCount = 0;
    for(auto i : sequence)
    {
        auto p = primitivesMap[i];
        qDebug() << i << " "<< p->getPrimitiveName();
        auto meshSegment = p->getMeshSegment();
        auto primitiveType = meshSegment->getPrimitiveType();

        QJsonObject step;

        if(sequenceCount == 0)
        {
            QJsonObject initialPlaneStep;
            QJsonObject emptyArray;
            initialPlaneStep["id"] = -1;
            initialPlaneStep["type"] = "Plane";
            initialPlaneStep["primitive"] = "StartPlane.ply";
            initialPlaneStep["segment"] = "StartPlane.obj";
            initialPlaneStep["ellipses"] = emptyArray;
            initialPlaneStep["sequence"] = emptyArray;
            auto startName = QString(dir).append(QString("/StartPlane"));


            auto isMidPlane = false;
            if(p->getPrimitiveName() == QString("handle_bigger.ply"))
            {
                isMidPlane = true;
            }
            auto planeIndex = 0;
            if(p->getPrimitiveName() == QString("body.ply") || p->getPrimitiveName() == QString("lense.ply"))
            {
                planeIndex = 2;
            }

             p->writeStartPlane(startName, isMidPlane, planeIndex);
            jsonSequence.push_back(initialPlaneStep);

            step["arrows"] = p->getStartArrows(isMidPlane, planeIndex);
        }


        auto boxName = QString("Primitive_").append(QString::number(sequenceCount)).append(QString(".ply"));
        auto boxFilename = QString(dir).append(QString("/")).append(boxName);
        p->writeBox(boxFilename);

        auto segmentName = QString("Segment_").append(QString::number(sequenceCount)).append(QString(".obj"));
        auto segmentFilename = QString(dir).append(QString("/")).append(segmentName);
        auto newBBox = p->getPoints();
        meshSegment->writeMesh(segmentFilename, newBBox);
        //auto segmentFilename = dir.append(segmentName);

//        QJsonArray jsonEllipses;
//        for(auto ellipse : ellipses)
//        {
//            QJsonObject jsonEllipse;
//            QJsonArray jsonTranslation;

        QJsonArray jsonEllipses;
        auto ellipsesArray = meshSegment->getJsonEllipses();
        for(auto j = 0; j < ellipsesArray.size(); j++)
        {
            QJsonObject jsonEllipse;
            QJsonArray jsonTranslation;
            QJsonArray jsonNormal;

            auto ellipse = ellipsesArray[j].toObject();
            auto normal = ellipse["normal"].toArray();
            jsonNormal.push_back(normal[1]);
            jsonNormal.push_back(normal[2]);
            jsonNormal.push_back(normal[0]);
            jsonEllipse["normal"] = jsonNormal;

            auto translation = ellipse["translation"].toArray();
            jsonTranslation.push_back(translation[1]);
            jsonTranslation.push_back(translation[2]);
            jsonTranslation.push_back(translation[0]);
            jsonEllipse["translation"] = jsonTranslation;

            jsonEllipse["diameter"] = ellipse["diameter"].toDouble();

            jsonEllipses.push_back(jsonEllipse);
        }


        step["id"] = sequenceCount;
        step["type"] = primitiveType;
        step["primitive"] = boxName;
        step["segment"] = segmentName;
        step["ellipses"] = jsonEllipses;

        QJsonArray pg_u0;
        QJsonArray pg_u1;
        QJsonArray pg_u2;
        for(auto g : previous_guides_u0)
        {
            auto gId = pointToId[g];
            pg_u0.push_back(gId);
        }
        for(auto g : previous_guides_u1)
        {
            auto gId = pointToId[g];
            pg_u1.push_back(gId);
        }
        for(auto g : previous_guides_u2)
        {
            auto gId = pointToId[g];
            pg_u2.push_back(gId);
        }

        QJsonObject primitvePreviousGuides;
        primitvePreviousGuides["user0"] = pg_u0;
        primitvePreviousGuides["user1"] = pg_u1;
        primitvePreviousGuides["user2"] = pg_u2;
        step["previous_guides"] = primitvePreviousGuides;

        QJsonObject guidesAndSequences = makeGuidesAndSequences(p->getXAxis(), p->getYAxis(), p->getZAxis(), primitivesNameToId, jsonGuides, pointToId, pointToLastSeenIndex, finalPrimitivesMap);
        auto stepSequence = guidesAndSequences["guide_sequence"].toArray();
        step["sequence"] = stepSequence;

        jsonSequence.push_back(step);
        sequenceCount++;
    }

    QJsonObject jsonOutput;
    jsonOutput["sequence"] = jsonSequence;
    jsonOutput["guides"] = jsonGuides;

    dir.append(QString("/sequence.json"));
    QFile saveFile(dir);
    if (!saveFile.open(QIODevice::WriteOnly)) {
            qWarning("Couldn't open save file.");
    }
    QJsonDocument saveDoc(jsonOutput);
    saveFile.write(saveDoc.toJson());

}


auto PrimitiveOptimisation::runOptimisation() -> void
{
    try
    {


        finalPrimitivesMap.clear();

        qDebug() << "PrimitiveOptimisation::runOptimisation()";

        qDebug() << "candidatePrimitivesMap " << candidatePrimitivesMap.size();


        GRBEnv* env = new GRBEnv();

        std::map<QString, int> parentMapIndex;
        std::map<QString, MeshSegment*> candidateToOriginal;
        int noPrimitives = 0;
        int totalNoCandidates = 0;
        for(auto candidatePair : candidatePrimitivesMap)
        {
            qDebug() << candidatePair.first;
            auto segments = mesh->getMeshSegments();
            for(auto i = size_t(0); i < segments.size(); i++)
            {
                if(segments[i]->getName() == candidatePair.first)
                {
                    candidateToOriginal.insert(std::make_pair(candidatePair.first, segments[i]));
                    qDebug() << candidatePair.first << "Found parent";
                    break; //Out of for loop
                }
            }

            for(auto candidate : candidatePair.second)
            {
                //qDebug() << "if(candidate->isInputPrimitive())";
                if(candidate->isInputPrimitive())
                {
                    auto it = parentMapIndex.find(candidatePair.first);
                    if(it != parentMapIndex.end())
                    {
                        qDebug() << "Error multiple original primitives " << totalNoCandidates << " " << candidate->getError();
                    }
                    else
                    {
                        parentMapIndex.insert(std::make_pair(candidatePair.first, totalNoCandidates));
                        qDebug() << "Parent Map " << candidatePair.first << " " << totalNoCandidates;
                    }
                }

                //qDebug() << "if(candidate->isInputPrimitive()) after";
                totalNoCandidates++;
            }
            noPrimitives++;
        }

        qDebug() << "candidateToOriginal.size() " << candidateToOriginal.size();
        qDebug() << "parentMapIndex.size() " << parentMapIndex.size();

         qDebug() << "totalNoCandidates " << totalNoCandidates << " noPrimitives " << noPrimitives << " totalNoCandidates * totalNoCandidates " << totalNoCandidates * totalNoCandidates ;

        int totalSquared = totalNoCandidates * totalNoCandidates;
        qDebug() << "totalSquared " << totalSquared;


        double  solution[totalNoCandidates]; //Solution vector
        //Model varaiables
        double  c[totalNoCandidates]; //Linear coefficients
        char    type[totalNoCandidates]; //Type of solution vars e.g binary, continuous
        double  ub[totalNoCandidates]; //Solution upper bound
        double  lb[totalNoCandidates]; //Solution lower bound
        //Linear constraint vars
        double  A[noPrimitives][totalNoCandidates]; //Linear constraint matrix
        std::fill(A[0], A[0] + noPrimitives * totalNoCandidates, 0);
        char    linearSense[noPrimitives]; //Linear sense constraint
        double  linearRhs[noPrimitives]; //Linear constraint RHS
        //Quatratic constraint vars
        double  Q[totalNoCandidates][totalNoCandidates]; //Hacky quadratic constraint matrix
        std::fill(Q[0], Q[0] + totalNoCandidates * totalNoCandidates, 0);
        char    quadSense[totalNoCandidates]; //Quadratic sense constraint
        double  quadRhs[totalNoCandidates]; //Quadratic constraint RHS

        qDebug() << "totalNoCandidates " << totalNoCandidates << " noPrimitives " << noPrimitives;

        int primitiveCount = 0;
        int candidateCount = 0;
        for(auto candidatePair : candidatePrimitivesMap)
        {
            qDebug() << "primitiveCount " << primitiveCount << " candidateCount " << candidateCount;
            for(auto candidate : candidatePair.second)
            {

                if(candidate == nullptr)
                    continue;

                auto val = candidate->getError();
                //qDebug() << "Error " << val;
                if(!std::isfinite(val))
                {
                    qDebug() << "Nan or inf error " << candidateCount << " val " << val;
                    return;
                }

                c[candidateCount] = val;
                type[candidateCount] = GRB_BINARY;
                ub[candidateCount] = 1;
                lb[candidateCount] = 0;

                A[primitiveCount][candidateCount] = 1;

                auto parents = candidate->getParentPrimitives();
                for(auto parent : parents)
                {
                    //qDebug() << candidateCount << " " << parent << " " << parentMapIndex[parent];
                    Q[candidateCount][parentMapIndex[parent]] = 1;
                }

                candidateCount++;
            }
            linearSense[primitiveCount] = '=';
            linearRhs[primitiveCount] = 1;
            primitiveCount++;
        }


        qDebug() << "GRBModel model";

        GRBModel model = GRBModel(*env);

//        // Add variables to the model
        GRBVar* vars = model.addVars(lb, ub, c, type, NULL, totalNoCandidates);
        model.update();

        //Linear Constraints for selecting
        double* Apt = &A[0][0];
        for (auto i = 0; i < noPrimitives; i++)
        {
            GRBLinExpr lhs = 0;
            for (auto j = 0; j < totalNoCandidates; j++)
              if (Apt[i*totalNoCandidates+j] != 0)
                 lhs += Apt[i*totalNoCandidates+j]*vars[j];
            model.addConstr(lhs, linearSense[i], linearRhs[i]);
        }
        model.update();

//        qDebug() << "Linear Constraints ";

//        //Quadratic Constraints for selecting parent
        for (auto i = 0; i < totalNoCandidates; i++)
        {
            for (auto j = 0; j < totalNoCandidates; j++)
            {
                //qDebug() << "Q[i][j] " << i << " " << j;
                if (Q[i][j] != 0)
                {
                   GRBQuadExpr lhs = 0;
                   //qDebug() << i << " * " << j << " - " << i;
                   //std::cout <<  i << " * " << j << " - " << i << std::endl;
                   lhs += vars[i]*vars[j] - vars[i]; // (child * parent) - child
                   model.addQConstr(lhs, '>=', 0);
                }
            }
        }
        model.update();

//        qDebug() << "Quadratic Constraints 1";

        //Quadratic constraint for ensuring relations are kept
        int aCandidateCount = 0;

        for(auto candidatePairA = candidatePrimitivesMap.begin(); candidatePairA != candidatePrimitivesMap.end(); candidatePairA++)
        {
            qDebug() << "segmentA before";
            auto segmentA = candidateToOriginal[candidatePairA->first];
            qDebug() << "segmentA after";
            for(auto candidateA : candidatePairA->second)
            {
                int bCandidateCount = 0;
                for(auto candidatePairB = candidatePrimitivesMap.begin(); candidatePairB != candidatePrimitivesMap.end(); candidatePairB++)
                {
                    if(candidatePairA->first == candidatePairB->first)
                    {
                        bCandidateCount += candidatePairB->second.size();
                        continue;
                    }
                    //qDebug() << "segmentB before";
                    auto segmentB = candidateToOriginal[candidatePairB->first];
                    //qDebug() << "segmentB after";
                    //qDebug() << "A " << candidatePairA->first << " B " << candidatePairB->first;
                    auto coplanarRelations = segmentA->findCoPlanarRelations(segmentB);
                    for(auto candidateB : candidatePairB->second)
                    {

                        for(auto coplanarRelation : coplanarRelations)
                        {
                            //qDebug() << "coplanarRelation";

                            //&& !coplanarRelation->isMidPlane()
                            if(!isValidCocplanarRelation(coplanarRelation, candidateA, candidateB) ) // Relation between pair is broken
                            {
                                qDebug() << "A " << candidatePairA->first << " B " << candidatePairB->first;
                                qDebug() << "Invalid relation possible, constraint added " << aCandidateCount << " " << bCandidateCount;
                                GRBQuadExpr lhs = 0;
                                lhs += vars[aCandidateCount]*vars[bCandidateCount];
                                model.addQConstr(lhs, '=', 0);
//                                GRBLinExpr lhs = 0;
//                                lhs += vars[aCandidateCount]+vars[bCandidateCount];
//                                model.addConstr(lhs, '<=', 1);
                            }


                        }

                        bCandidateCount++;
                    }
                 }
                aCandidateCount++;
            }
        }
        model.update();
        qDebug() << "Quadratic Constraints 2";

        model.optimize();

        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            //*objval1 = model.get(GRB_DoubleAttr_ObjVal);
            for (auto i = 0; i < totalNoCandidates; i++)
              solution[i] = vars[i].get(GRB_DoubleAttr_X);
        }
        else
        {
            std::cout << "Not optimal solution" << std::endl;
            return;
        }

        //finalPrimitivesMap
        primitiveCount = 0;
        candidateCount = 0;
        for(auto candidatePair : candidatePrimitivesMap)
        {
            for(auto candidate : candidatePair.second)
            {
                if(solution[candidateCount] == 1)
                {
                    qDebug() << candidate->getPrimitiveName() << "total " << candidate->getError() << " guide " << candidate->getGuideError() << " geometry " << candidate->getGeometryError();
                    finalPrimitivesMap.insert(std::make_pair(candidatePair.first, candidate));
                }

                candidateCount++;
            }
            primitiveCount++;
        }
        findOrdering();

    } catch(GRBException e) {
       std::cout << "Error code = " << e.getErrorCode() << std::endl;
       std::cout << e.getMessage() << std::endl;
     } catch (...) {
       std::cout << "Error during optimization" << std::endl;
     }

}
