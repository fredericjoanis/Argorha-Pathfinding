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

#include "PathfindingAStarNode.h"

namespace Pathfinding {

    AStarNode::AStarNode( const AStarNode &node )
    {
        mCost = node.mCost;
        mHeuristic = node.mHeuristic;
        mParent = node.mParent;
        mPortal = node.mPortal;
        mScore = node.mScore;
        mPos = node.mPos;
    }

    AStarNode::AStarNode()
    {
        mParent = NULL;
        mScore = -1;
        mHeuristic = -1;
        mCost = -1;
    }

    AStarNode::~AStarNode()
    {
        mParent = NULL;
        mScore = -1;
        mHeuristic = -1;
        mCost = -1;
    }

    bool AStarNode::operator<(const AStarNode &pathNode) const
    {
        return this->mScore < pathNode.getScore();
    }

    bool AStarNode::operator>(const AStarNode &pathNode) const
    {
        return this->mScore > pathNode.getScore();
    }


    void AStarNode::setScore(Pathfinding::Math::Real value) 
    {
        mScore = value;
    }

    void AStarNode::setHeuristic(Pathfinding::Math::Real value) 
    {
        mHeuristic = value;
    }

    void AStarNode::setCost(Pathfinding::Math::Real value) 
    {
        mCost = value;
    }

    void AStarNode::setPos( const Pathfinding::Math::Vector3 & value ) 
    {
        mPos = value;
    }

    void AStarNode::setPortal(const Portal *value)
    {
        mPortal = value;
    }

    void AStarNode::setParent(const AStarNode* value)
    {
        mParent = value;
    }

} // namespace Pathfinding
