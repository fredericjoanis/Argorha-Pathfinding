#include "YourEnginePathData.h"
#include "PathfindingPortal.h"

namespace YourEnginePath
{
    PathData::PathData()
    {

    }

    void PathData::addSector( Pathfinding::Sector * io_sector, int in_hLevel )
    {
        listSector.push_back(io_sector);
    }

    void PathData::addPortal( Pathfinding::Portal * io_portal, int in_hLevel )
    {
        listPortal.push_back(io_portal);
    }

    bool PathData::suppressSector( Pathfinding::Sector * io_sector, int in_hLevel )
    {
        return true;
    }

    bool PathData::suppressPortal( Pathfinding::Portal * io_portal, int in_hLevel )
    {
        return true;
    }

    std::list<const Pathfinding::Portal*> PathData::getPortalsAtPos( const Pathfinding::Math::Vector3 & pos, int in_hLevel )
    {
        std::list<const Pathfinding::Portal*> ret;

        for( std::list<Pathfinding::Portal*>::iterator iter = listPortal.begin(); iter != listPortal.end(); iter++ )
        {
            if( (*iter)->getBox().intersects( pos ) )
            {
                ret.push_back(*iter);
            }
        }

        return ret;
    }

    Pathfinding::Sector* PathData::getSectorAtPos( const Pathfinding::Math::Vector3 & pos, int in_hLevel )
    {
        Pathfinding::Sector* ret = 0;

        for( std::list<Pathfinding::Sector*>::iterator iter = listSector.begin(); iter != listSector.end() && ret == 0; iter++ )
        {
            if( (*iter)->getBox().intersects( pos ) )
            {
                ret = *iter;
            }
        }

        return ret;
    }

    std::vector<const Pathfinding::Portal*> PathData::getPortalsInZone( const Pathfinding::Math::AxisAlignedBox & in_box, int in_hLevel )
    {
        std::vector<const Pathfinding::Portal*> ret;

        for( std::list<Pathfinding::Portal*>::iterator iter = listPortal.begin(); iter != listPortal.end(); iter++ )
        {
            if( (*iter)->getBox().intersects( in_box ) )
            {
                ret.push_back(*iter);
            }
        }

        return ret;
    }
}