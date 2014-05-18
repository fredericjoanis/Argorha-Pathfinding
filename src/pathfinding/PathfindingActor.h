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

#ifndef _PATHFINDINGACTOR_H
#define _PATHFINDINGACTOR_H

#include "PathfindingPrerequisites.h"
#include "Math/PathfindingAxisAlignedBox.h"
#include "PathfindingAction.h"
#include "PathfindingData.h"


namespace Pathfinding {

    /** @class PathfindingActor PathfindingActor.h "include/PathfindingActor.h"
    *
    * This class is a container of what an actor is in the sense of a pathfinding.  An actor is a container of a certain format
    * that is going to move around the world.  Ideally there would be actors like tinyActor, mediumActor, largeActor, hugeActor.
    * Each actors hold it's own data for the pathfinding.
    */

    class _PathfindingExport Actor {
    protected:

        /**
        The box of the actor.
        */
        Pathfinding::Math::AxisAlignedBox mBox;

        /**
        All the actions that an actor can do.  Not yet implemented
        */
        std::set<Action> mAction;

        /**
        The position of the actor in the world
        */
        Pathfinding::Math::Vector3 mPos;

        /**
        To have a good pathfinding, the actor must be subdivide into smaller pieces.  Divided by 2 is recommended.
        */
        Pathfinding::Math::Real mSubdivisionX; 
        /**
        To have a good pathfinding, the actor must be subdivide into smaller pieces.  Divided by 2 is recommended.
        */
        Pathfinding::Math::Real mSubdivisionZ;

        /**
        The small vector is the vector of the box subdivided by X and Z.  
        This value is an optimization to not have to recalculate it each time.
        */
        Pathfinding::Math::Vector3 mSmallVector;

        /**
        The data of the actor.  Each pathfinding actor must have a different data.
        */
        Data * mData;

    public:

        Actor( Data * in_Data, Pathfinding::Math::AxisAlignedBox in_Box )
            : mSubdivisionX( 2 ), mSubdivisionZ( 2 ), mData( in_Data ), mBox( in_Box ), mPos(), mSmallVector() 
        { 
            // Calculate the small vector because the box is set here
            calculateSmallVector(); 
        }

        ~Actor();

        /**
        @return the subdivided bounding box
        */
        inline const Pathfinding::Math::Vector3 getSmallVector() const;

        /**
        @return the bounding box
        */
        inline const Pathfinding::Math::AxisAlignedBox getBox() const;

        /**
        @param[in] in_value new box value
        */
        void setBox(const Pathfinding::Math::AxisAlignedBox & in_value);

        /**
        @return all the actor possible actions
        */
        inline const std::set<Action> getAction() const;

        /**
        @param in_action the action to be added
        @return the action has been added
        */
        bool addAction(const Action & in_action);

        /**
        @param in_action the action to be removed
        */
        void removeAction(const Action & in_action);

        /**
        @return mPos
        */
        inline const Pathfinding::Math::Vector3 getPosition() const;

        /**
        @param in_value the position to be set
        */
        void setPosition(const Pathfinding::Math::Vector3 & in_value);

        /**
        @return mSubdivisionX
        */
        inline const Pathfinding::Math::Real getSubdivisionX() const;

        /**
        @param in_value new mSubdivisionX value
        */
        void setSubdivisionX(const Pathfinding::Math::Real in_value);

        /**
        @return mSubdivisionZ
        */
        inline const Pathfinding::Math::Real getSubdivisionZ() const;

        /**
        @param in_value new mSubdivisionZ value
        */
        void setSubdivisionZ(const Pathfinding::Math::Real in_value);

        /**
        @return data about this actor
        */
        inline Data* getData() const;

        /**
        @param io_value Set data that goes with the actor.
        */
        void setData( Data * io_value );

    private:
        /**
        Function that calculate the small vector.  It needs to be call whenever a subdivision or the box is changed.
        */
        void calculateSmallVector();

    };

    inline const Pathfinding::Math::Vector3 Actor::getSmallVector() const 
    {
        return mSmallVector;
    }

    inline const Pathfinding::Math::AxisAlignedBox Actor::getBox() const 
    {
        return mBox;
    }

    inline const std::set<Action> Actor::getAction() const 
    {
        return mAction;
    }

    inline const Pathfinding::Math::Vector3 Actor::getPosition() const
    {
        return mPos;
    }

    inline const Pathfinding::Math::Real Actor::getSubdivisionX() const
    {
        return mSubdivisionX;
    }

    inline const Pathfinding::Math::Real Actor::getSubdivisionZ() const
    {
        return mSubdivisionZ;
    }

    inline Data* Actor::getData() const
    {
        return mData;
    }

} // namespace Pathfinding
#endif
