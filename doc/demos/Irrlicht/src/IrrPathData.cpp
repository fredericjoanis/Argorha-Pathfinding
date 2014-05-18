#include "IrrPathData.h"
#include "PathfindingPortal.h"

namespace PathIrr
{
    IrrPathData::IrrPathData()
    {

    }

    void IrrPathData::addSector( Pathfinding::Sector * io_sector, int in_hLevel )
    {
        mListSector.push_back(io_sector);
    }

    void IrrPathData::addPortal( Pathfinding::Portal * io_portal, int in_hLevel )
    {
        mListPortal.push_back(io_portal);
    }

    bool IrrPathData::suppressSector( Pathfinding::Sector * io_sector, int in_hLevel )
    {
        return true;
    }

    bool IrrPathData::suppressPortal( Pathfinding::Portal * io_portal, int in_hLevel )
    {
        return true;
    }

    std::list<const Pathfinding::Portal*> IrrPathData::getPortalsAtPos( const Pathfinding::Math::Vector3 & pos, int in_hLevel )
    {
        std::list<const Pathfinding::Portal*> ret;

        for( std::list<Pathfinding::Portal*>::iterator iter = mListPortal.begin(); iter != mListPortal.end(); iter++ )
        {
            if( (*iter)->getBox().intersects( pos ) )
            {
                ret.push_back(*iter);
            }
        }

        return ret;
    }

    Pathfinding::Sector* IrrPathData::getSectorAtPos( const Pathfinding::Math::Vector3 & pos, int in_hLevel )
    {
        Pathfinding::Sector* ret = 0;

        for( std::list<Pathfinding::Sector*>::iterator iter = mListSector.begin(); iter != mListSector.end() && ret == 0; iter++ )
        {
            if( (*iter)->getBox().intersects( pos ) )
            {
                ret = *iter;
            }
        }

        return ret;
    }

    std::vector<const Pathfinding::Portal*> IrrPathData::getPortalsInZone( const Pathfinding::Math::AxisAlignedBox & in_box, int in_hLevel )
    {
        std::vector<const Pathfinding::Portal*> ret;

        for( std::list<Pathfinding::Portal*>::iterator iter = mListPortal.begin(); iter != mListPortal.end(); iter++ )
        {
            if( (*iter)->getBox().intersects( in_box ) )
            {
                ret.push_back(*iter);
            }
        }

        return ret;
    }
}