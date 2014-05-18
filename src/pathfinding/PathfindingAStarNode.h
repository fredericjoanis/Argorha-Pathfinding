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

#ifndef _PATHFINDINGASTARNODE_H
#define _PATHFINDINGASTARNODE_H

#include "PathfindingPrerequisites.h"
#include "Math/PathfindingVector3.h"

namespace Pathfinding {
    /** @class PathfindingAStarNode PathfindingAStarNode.h "include/PathfindingAStarNode.h"
    *
    * This class is used to encapsulate all the data relative to a A* node.
    */

    class _PathfindingExport AStarNode {
    private:
        /** Equivalent of the f in algorithms given
        * When the mScore is -1, it means that it's not in the open list
        */
        Pathfinding::Math::Real mScore; 

        /// Equivalent of the h in algorithms given
        Pathfinding::Math::Real mHeuristic; 

        /// Equivalent of the g in algorithms given
        Pathfinding::Math::Real mCost;

        /// The position in the portal.
        Pathfinding::Math::Vector3 mPos; 

        /// The node parent
        const AStarNode* mParent;

        /// The portal in which the node belong
        const Portal *mPortal; 

    public:
        AStarNode( const AStarNode &node );
        AStarNode();

        virtual ~AStarNode();

        /**
        @return mScore
        */
        inline const Pathfinding::Math::Real getScore() const;

        /**
        @param in_value new value for mScore
        */
        void setScore(Pathfinding::Math::Real in_value);

        /**
        @return mHeuristic
        */
        inline const Pathfinding::Math::Real getHeuristic() const;

        /**
        @param in_value new value for mHeuristic
        */
        void setHeuristic(Pathfinding::Math::Real in_value);

        /**
        @return mCost
        */
        inline const Pathfinding::Math::Real getCost() const;

        /**
        @param in_value new value for mCost
        */
        void setCost(Pathfinding::Math::Real in_value);

        /**
        @return mPos
        */
        inline const Pathfinding::Math::Vector3 getPos() const;

        /**
        @param in_value new value for mPos
        */
        void setPos(const Pathfinding::Math::Vector3 & in_value);

        /**
        @return mParent
        */
        inline const AStarNode* getParent() const;

        /**
        @param in_value new value for mParent
        */
        void setParent(const AStarNode* value);

        /**
        @param in_pathNode Compare if this node is lower then input node
        */
        bool operator<(const AStarNode &in_pathNode) const;

        /**
        @param in_pathNode Compare if this node is lower then input node
        */
        bool operator>(const AStarNode &in_pathNode) const;

        /**
        @return mPortal
        */
        inline const Portal* getPortal() const;

        /**
        @param in_value new value for mPortal
        */
        void setPortal(const Portal* in_value);

    };

    inline const Pathfinding::Math::Real AStarNode::getScore() const 
    {
        return mScore;
    }

    inline const Pathfinding::Math::Real AStarNode::getHeuristic() const 
    {
        return mHeuristic;
    }

    inline const Pathfinding::Math::Real AStarNode::getCost() const 
    {
        return mCost;
    }

    inline const Pathfinding::Math::Vector3 AStarNode::getPos() const 
    {
        return mPos;
    }

    inline const Portal* AStarNode::getPortal() const
    {
        return mPortal;
    }

    inline const AStarNode* AStarNode::getParent() const
    {
        return mParent;
    }



} // namespace Pathfinding
#endif
