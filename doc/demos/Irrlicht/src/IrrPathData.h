/*
-----------------------------------------------------------------------------
This source file is part of Switch Blade

Copyright (c) ADGA - projet Argorha - Argorha.com - 2006-2007

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place - Suite 330, Boston, MA 02111-1307, USA, or go to
http://www.gnu.org/copyleft/lesser.txt.
-----------------------------------------------------------------------------
*/


#ifndef _IRRPATHFINDINGDATA_H
#define _IRRPATHFINDINGDATA_H

#include "PathfindingData.h"
#include <list>

namespace PathIrr {

    /** @class PathfindingData GOOFPathfindingData.h "include/GOOFPathfindingData.h"
    *
    * This class is used to store the data about pathfinding.
    */

    class IrrPathData : public Pathfinding::Data 
    {
    public:

        IrrPathData();

        /**
        * @param io_sector sector to be added
        * @param in_hLevel the hierarchical level into which the sector is going
        */
        virtual void addSector( Pathfinding::Sector * io_sector, int in_hLevel );

        /**
        * @param io_portal portal to be added
        * @param in_hLevel the hierarchical level into which the portal is going
        */
        virtual void addPortal( Pathfinding::Portal * io_portal, int in_hLevel );

        /** 
        * @param io_sector sector to be suppress and deleted
        * @param in_hLevel the hierarchical level into which the sector is going
        * @Return true if the element has been suppressed 
        */
        virtual bool suppressSector( Pathfinding::Sector * io_sector, int in_hLevel );

        /** 
        * @param io_portal portal to be suppress and deleted
        * @param in_hLevel the hierarchical level into which the portal is going
        * @Return true if the element has been suppressed 
        */
        virtual bool suppressPortal( Pathfinding::Portal * io_portal, int in_hLevel );

        /** 
        * @param[in] in_pos position to find the portals
        * @param in_hLevel the hierarchical level into which the portals are
        * @Return list of portals at the in_pos
        */
        virtual std::list<const Pathfinding::Portal*> getPortalsAtPos( const Pathfinding::Math::Vector3 & pos, int in_hLevel );

        /** 
        * @param[in] in_pos position to find the sectors
        * @param in_hLevel the hierarchical level into which the sector is going
        * @Return the sector at that position
        */
        virtual Pathfinding::Sector* getSectorAtPos(const Pathfinding::Math::Vector3 & pos, int in_hLevel);

        /** 
        * @param[in] in_box the box to find the portals
        * @param in_hLevel the hierarchical level into which the portals are
        * @Return list of portals that intersects the box
        */
        virtual std::vector<const Pathfinding::Portal*> getPortalsInZone( const Pathfinding::Math::AxisAlignedBox & in_box, int in_hLevel );

    public:
        /**
        * @typedef a list of the sectors
        */
        std::list<Pathfinding::Sector*> mListSector;

        /**
        * @typedef a list of the portals
        */
        std::list<Pathfinding::Portal*> mListPortal;
    };

} // namespace GOOF
#endif
