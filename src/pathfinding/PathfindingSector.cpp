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

#include "PathfindingSector.h"
#include "PathfindingPortal.h"

namespace Pathfinding {

    Sector::Sector()
    {
        this->mCost    = 0;
    }

    Sector::Sector(const Pathfinding::Sector &sector)
    {
        this->mBox     = sector.mBox;
        this->mCost    = sector.mCost;
        this->mPortals = sector.mPortals;
    }

    bool Sector::operator<(const Sector &sector) const
    {
        // This line is just to make a distinction between portals
        // Modification 19 july 2007
        // Line before : return this->mBox.getMinimum().x < portal.mBox.getMinimum().x;
        return this->mBox.getMinimum().x + this->mBox.getMinimum().y < sector.mBox.getMinimum().x + sector.mBox.getMinimum().y;
    }

    bool Sector::operator==(const Sector &sector) const
    {
        return this->mBox.getMinimum() == sector.mBox.getMinimum() && this->mBox.getMaximum() == sector.mBox.getMaximum();
    }

    void Sector::setBox(const Pathfinding::Math::AxisAlignedBox & value) {
        mBox = value;
    }

    void Sector::addPortal( Portal * portal ) 
    {
        this->mPortals.push_front( portal );
        this->mPortals.unique();
    }


    void Sector::setCost( Pathfinding::Math::Real value)
    {
        mCost = value;
    }

    void Sector::suppressPortal( Portal * in_portal )
    {
        mPortals.remove( in_portal );
    }

    Sector::~Sector()
    {
        // When sector is being deleted, set the portals to null.
        for( std::list<Portal*>::iterator iterPortals = mPortals.begin(); iterPortals != mPortals.end(); iterPortals++ )
        {
            if( (*iterPortals)->getDestSector1() == this )
            {
                (*iterPortals)->setDestSector1( NULL );
            }

            if( (*iterPortals)->getDestSector2() == this )
            {
                (*iterPortals)->setDestSector2( NULL );
            }
        }
        mPortals.clear();
    }

} // namespace Pathfinding
