/*
-----------------------------------------------------------------------------
This source file is part of Argorha project

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

#ifndef _PATHFINDINGDATA_H
#define _PATHFINDINGDATA_H

#include "PathfindingPrerequisites.h"

namespace Pathfinding {

    /** @class PathfindingData PathfindingData.h "include/PathfindingData.h"
    *
    * This interface is used to store the data about pathfinding.
    * For the hierarchical pathfinding, there is levels.  The lowest level in which the path
    * is perfect is at level 0.
    */
    class _PathfindingExport Data {
    public:

        Data::Data() {}

        /**
        * @param io_sector sector to be added
        * @param in_hLevel the hierarchical level into which the sector is going
        */
        virtual void addSector( Sector * io_sector, int in_hLevel ) = 0;

        /**
        * @param io_portal portal to be added
        * @param in_hLevel the hierarchical level into which the portal is going
        */
        virtual void addPortal( Portal * io_portal, int in_hLevel ) = 0;

        /** 
        * @param io_sector sector to be suppress and deleted
        * @param in_hLevel the hierarchical level into which the sector is going
        * @Return true if the element has been suppressed 
        */
        virtual bool suppressSector( Sector * io_sector, int in_hLevel ) = 0;

        /** 
        * @param io_portal portal to be suppress and deleted
        * @param in_hLevel the hierarchical level into which the portal is going
        * @Return true if the element has been suppressed 
        */
        virtual bool suppressPortal( Portal * io_portal, int in_hLevel ) = 0;

        /** 
        * @param[in] in_pos position to find the portals
        * @param in_hLevel the hierarchical level into which the portals are
        * @Return list of portals at the in_pos
        */
        virtual std::list<const Portal*> getPortalsAtPos( const Pathfinding::Math::Vector3 & pos, int in_hLevel ) = 0;

        /** 
        * @param[in] in_pos position to find the sectors
        * @param in_hLevel the hierarchical level into which the sector is going
        * @Return the sector at that position
        */
        virtual Sector* getSectorAtPos(const Pathfinding::Math::Vector3 & pos, int in_hLevel) = 0;

        /** 
        * @param[in] in_box the box to find the portals
        * @param in_hLevel the hierarchical level into which the portals are
        * @Return list of portals that intersects the box
        */
        virtual std::vector<const Portal*> getPortalsInZone( const Pathfinding::Math::AxisAlignedBox & in_box, int in_hLevel ) = 0;

    };


} // namespace Pathfinding
#endif
