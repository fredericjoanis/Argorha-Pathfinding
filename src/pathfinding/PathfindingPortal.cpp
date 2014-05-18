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

#include "PathfindingPortal.h"
#include "PathfindingAction.h"

namespace Pathfinding {

    Portal::Portal()
    {
        this->mDestSector1 = NULL;
        this->mDestSector2 = NULL;
    }

    Portal::Portal(const Portal &portal)
    {
        this->mActions = portal.mActions;
        this->mBox = portal.mBox;
        this->mDestSector1 = portal.mDestSector1;
        this->mDestSector2 = portal.mDestSector2;
        this->mMechanism = portal.mMechanism;
    }

    bool Portal::operator<(const Portal &portal) const
    {
        // This line is just to make a distinction between portals
        // Modification 19 july 2007
        // Line before : return this->mBox.getMinimum().x < portal.mBox.getMinimum().x;
        return this->mBox.getMinimum().x + this->mBox.getMinimum().y < portal.mBox.getMinimum().x + portal.mBox.getMinimum().y;
    }

    bool Portal::operator==(const Portal &portal) const
    {
        return this->mBox.getMinimum() == portal.mBox.getMinimum() && 
            this->mBox.getMaximum() == portal.mBox.getMaximum() && 
            mDestSector1 == portal.mDestSector1 && 
            mDestSector2 == portal.mDestSector2;
    }


    void Portal::setBox(const Pathfinding::Math::AxisAlignedBox & value) {
        mBox = value;
    }

    void Portal::setDestSector1( Sector * value) {
        mDestSector1 = value;
    }

    void Portal::setDestSector2( Sector * value) {
        mDestSector2 = value;
    }

    void Portal::setMechanism(const Mechanism & value) {
        mMechanism = value;
    }

    //Return true the action has been added
    void Portal::addAction(const Action * action) {
        this->mActions.push_front( action );
        this->mActions.unique();
    }

    Sector* Portal::getDestSector1()
    {
        return mDestSector1;
    }

    Sector* Portal::getDestSector2()
    {
        return mDestSector2;
    }

    Portal::~Portal()
    {
        if( mDestSector1 )
        {
            mDestSector1->suppressPortal( this );
        }
        if( mDestSector2 )
        {
            mDestSector2->suppressPortal( this );
        }
    }

} // namespace Pathfinding
