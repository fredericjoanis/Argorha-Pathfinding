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

#include "PathfindingActor.h"


namespace Pathfinding {

    void Actor::setBox(const Pathfinding::Math::AxisAlignedBox & value) 
    {
        mBox = value;
        calculateSmallVector();
    }

    bool Actor::addAction(const Action & _action)
    {
        return mAction.insert(_action).second;
    }

    void Actor::removeAction(const Action & _action)
    {
        mAction.erase(_action);
    }

    void Actor::setPosition(const Pathfinding::Math::Vector3 & value)
    {
        mPos = value;
    }

    void Actor::setSubdivisionX(const Pathfinding::Math::Real value)
    {
        // Make sure there is no undesired value.
        if( value > 0 )
        {
            mSubdivisionX = value;
            calculateSmallVector();
        }
    }

    void Actor::setSubdivisionZ(const Pathfinding::Math::Real value)
    {
        // Make sure there is no undesired value.
        if( value > 0 )
        {
            mSubdivisionZ = value;
            calculateSmallVector();
        }
    }

    void Actor::setData( Data * io_value )
    {
        mData = io_value;
    }

    void Actor::calculateSmallVector()
    {
        mSmallVector = Pathfinding::Math::Vector3(mBox.getMaximum().x / mSubdivisionX, mBox.getMaximum().y, mBox.getMaximum().z / mSubdivisionZ );
    }

    Actor::~Actor()
    {

    }

} // namespace Pathfinding
