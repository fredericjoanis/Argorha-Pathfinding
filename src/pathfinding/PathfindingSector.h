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


#ifndef _PATHFINDINGSECTOR_H
#define _PATHFINDINGSECTOR_H

#include "PathfindingPrerequisites.h"
#include "Math/PathfindingAxisAlignedBox.h"


namespace Pathfinding {

    /** @class PathfindingSector PathfindingSector.h "include/PathfindingSector.h"
    *
    * This class is used to represent a sector.
    */

    class _PathfindingExport Sector {
    private:
        /// The box defining the sector
        Pathfinding::Math::AxisAlignedBox mBox;

        /// List of portals surrounding this sector
        std::list<Portal*> mPortals;

        /// The average cost for the sector
        Pathfinding::Math::Real mCost;

    public:

        Sector();
        Sector(const Sector &in_sector);
        ~Sector();

        /**
        * Compare if this sector is at a lower position than the input one.  It is used to do a discrimination between portals.
        * This is to find sectors faster.
        * 
        * @param in_sector the sector to compare with
        * @return true if the actual sector is at a lower position than the other.
        */
        bool operator<(const Sector &in_sector) const;

        /**
        * Compare if this sector is at an equal position position than the input one.
        * 
        * @param in_sector the sector to compare with
        * @return true if the actual sector is at same position than the other.
        */
        bool operator==(const Sector &in_sector) const;

        /**
        @return mBox
        */
        inline const Pathfinding::Math::AxisAlignedBox & getBox() const;

        /**
        @param in_value new mBox value
        */
        void setBox(const Pathfinding::Math::AxisAlignedBox & in_value);

        /**
        @return mCost
        */
        inline const Pathfinding::Math::Real getCost() const;

        /**
        @param in_value new mCost value
        */
        void setCost( Pathfinding::Math::Real in_value);


        /**
        @return mPortals
        */
        inline const std::list<Portal*> & getPortals() const;

        /** 
        * Add the portal in the portal list and make sure the portal is unique
        * @ param io_portal portal to be added
        */
        void addPortal( Portal * io_portal );

        /**
        * Suppress a portal from the list
        * @param io_portal portal that needs to be deleted.
        */ 
        void suppressPortal( Portal * io_portal );

    };


    inline const Pathfinding::Math::AxisAlignedBox & Sector::getBox() const {
        return mBox;
    }

    inline const std::list<Portal*> & Sector::getPortals() const {
        return mPortals;
    }

    inline const Pathfinding::Math::Real Sector::getCost() const
    {
        return mCost;
    }

} // namespace Pathfinding
#endif //_PATHFINDINGSECTOR_H
