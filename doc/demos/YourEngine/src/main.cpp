#include "YourEnginePrePathfindingPhysic.h"
#include "YourEnginePathData.h"
#include "Pre/PathfindingPreCompute.h"
#include "PathfindingAStar.h"
#include "Path/PathfindingPathCardinalSpline.h"

int main()
{
    // The zone where the pathfinding will be calculated
    Pathfinding::Math::AxisAlignedBox zone( -1000, -100, -1200, 1300, 500, 1000 );

    Pathfinding::Pre::Compute compute;
    YourEnginePath::PrePathfindingPhysic phys;
    YourEnginePath::PathData* pathData = new YourEnginePath::PathData();
    Pathfinding::Actor* pathActor = new Pathfinding::Actor(pathData, Pathfinding::Math::AxisAlignedBox( 0.f, 0.f, 0.f, 30.f, 50.f, 30.f));

    // Calculate all sectors and portals.
    compute.generateSectorsAndPortals(&phys, pathActor, &zone, 0 );

    // Then you can search a path.  In this demo, it's empty, so the A* will return nada.
    Pathfinding::Math::Vector3 m_startPos( 0.f, 0.f, 0.f);
    Pathfinding::Math::Vector3 m_endPos( 10.f, 0.f, 10.f);

    Pathfinding::AStar AStar;

    std::vector<Pathfinding::Math::Vector3> pathPoints;
    std::vector<Pathfinding::AStarNode> AStarNodes;

    AStarNodes = AStar.getPath(m_startPos, m_endPos, pathActor->getData(), 0 );

    // Make a cardinal spline out of the points returned by the A*
    Pathfinding::Path::InterpolatePathData interpolateData;
    interpolateData.waypoints = &AStarNodes;
    interpolateData.nbPoints  = 15;
    interpolateData.data      = pathActor->getData();
    interpolateData.hLevel    = 0;

    Pathfinding::Path::CardinalSpline cardinalPath;
    pathPoints = cardinalPath.interpolateFromWaypoints( &interpolateData );

    delete pathData;
    delete pathActor;
}