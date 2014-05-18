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

#include "PathfindingAStar.h"
#include "PathfindingAStarNode.h"
#include "PathfindingPortal.h"
#include "PathfindingData.h"
#include "Boost/lambda/lambda.hpp"

namespace Pathfinding {

    AStar::~AStar()
    {
        clearData();
    }

    std::vector<AStarNode> AStar::getPath(const Pathfinding::Math::Vector3 & in_start, const Pathfinding::Math::Vector3 & in_end, Data *in_data, const int in_hLevel ) 
    {
        assert( in_data );
        clearData();

        mData = in_data;
        mStartPos = in_start;
        mEndPos = in_end;
        mLevel = in_hLevel;

        std::vector<AStarNode> path;

        const Sector* secStart = findZone( mStartPos );
        const Sector* secEnd = findZone( mEndPos );

        if( secStart != NULL && secEnd != NULL )
        {
            if( secStart != secEnd )
            {
                createStartEndPortal( secStart, secEnd );
                path = processOpenList();
            }
            if( path.size() > 0 || secStart == secEnd )
            {
                addStartEndPortal( path );
            }
        }

        return path;
    }

    std::vector<AStarNode> AStar::getPath( const Pathfinding::Math::Vector3 & in_start, const Pathfinding::Math::Vector3 & in_end, const Actor* in_actor, Path::Path* in_path, const int in_hLevelStart, const int in_hLevelEnd )
    {
        assert( in_actor != NULL && in_actor->getData() != NULL && in_hLevelStart >= in_hLevelEnd );
        int actualLevel = in_hLevelStart;

        std::vector<AStarNode> waysToCalculate;
        std::vector<Pathfinding::Math::Vector3> posToCalculate;

        posToCalculate.push_back( in_start );
        posToCalculate.push_back( in_end );

        bool bContinue = true;

        while( actualLevel >= in_hLevelEnd && bContinue )
        {
            unsigned int actualPos = 1;

            waysToCalculate.clear();

            // Calculate between each points returned at the higher level
            while( actualPos < posToCalculate.size() && bContinue )
            {
                std::vector<AStarNode> pathNode = getPath( posToCalculate[ actualPos - 1 ], posToCalculate[ actualPos ], in_actor->getData(), actualLevel );
                // Regroup the new path

                std::vector<AStarNode>::iterator lastElement = pathNode.end();
                if(pathNode.size() > 0 )
                {
                    lastElement--;
                    // Insert all the elements but the last one, else the same positions will be there twice.
                    waysToCalculate.insert( waysToCalculate.end(), pathNode.begin(), lastElement );
                    if( actualPos == posToCalculate.size() - 1 )
                    {
                        waysToCalculate.push_back( *lastElement );
                    }
                    actualPos++;
                }
                else
                {
                    // Make sure the path dosen't continue, there's no path.
                    bContinue = false;
                    waysToCalculate.clear();
                }
            }

            if( bContinue )
            {
                // Make sure the path is the best one at that level
                in_path->reCalculatePoints( &waysToCalculate, in_actor );
            }

            if( actualLevel != in_hLevelEnd && bContinue )
            {
                // Put positions for the next level
                posToCalculate.clear();
                for( std::vector<AStarNode>::iterator iterNode = waysToCalculate.begin(); iterNode != waysToCalculate.end(); iterNode++ )
                {
                    posToCalculate.push_back( iterNode->getPos() );
                }
            }

            actualLevel--;
        }

        return waysToCalculate;
    }
    std::vector<AStarNode> AStar::processOpenList()
    {
        std::vector<AStarNode> retPath;
        for( std::list<AStarNode*>::iterator iterNode = mStartNodes.begin(); iterNode != mStartNodes.end(); ++iterNode )
        {
            addInLists( *iterNode );
        }

        bool bEnd = false;
        while( !mOpenList.empty() && !bEnd )
        {
            AStarNode* current = getLowest();
            if( current )
            {
                bEnd = endPortal( current );
                if( !bEnd )
                {
                    std::list<AStarNode*> successors = generateSuccessors( current );
                    processSuccessors( successors, current );
                    //moveFromOpenToClose( current );
                    // Tells the node that it's in the close list.  It has a cost, but no score.
                    current->setScore( -1 );
                }
                // Make sure there is a path
                else if( mEndNodes.size() > 0 )
                {
                    // We need to insert the nodes in the right place, from first to last, but the actual Node is the last one.
                    // So we need to reverse this.
                    AStarNode backTrack = *current;
                    int iNbNodes = 1;
                    while( backTrack.getParent() )
                    {
                        backTrack = *backTrack.getParent();
                        iNbNodes++;
                    }
                    // iNbNodes contains the number of waypoints.
                    retPath.resize( iNbNodes );

                    backTrack = *current;

                    // Go in the tree and get every parents to put it in the list, at the right place
                    while( backTrack.getParent() )
                    {
                        iNbNodes--;
                        retPath[ iNbNodes ] = backTrack;
                        backTrack = *backTrack.getParent();
                    }
                    // Set the first element
                    retPath[ 0 ] = backTrack;
                }
            }
            else
            {
                bEnd = true;
            }
        }
        return retPath;
    }


    void AStar::addStartEndPortal( std::vector<AStarNode> & io_list )
    {
        AStarNode startNode;
        startNode.setPos( mStartPos );
        std::list<const Portal*> portals = mData->getPortalsAtPos( mStartPos, mLevel );

        bool bInsert = true;

        // Make sure there's something in the io_list to make comparison.
        if( io_list.begin() != io_list.end() )
        {
            for( std::list<const Portal*>::iterator iterPortal = portals.begin(); iterPortal != portals.end() && bInsert; iterPortal++ )
            {
                if( *iterPortal == io_list.begin()->getPortal() )
                {
                    bInsert = false;
                }
            }
        }

        if( bInsert )
        {
            if( portals.size() > 0 )
            {
                startNode.setPortal( portals.front() );  // There is a portal at that place.
            }
            else
            {
                startNode.setPortal( NULL ); // That's not a real portal
            }
            io_list.insert( io_list.begin(), startNode );
        }

        portals = mData->getPortalsAtPos( mEndPos, mLevel );

        bInsert = true;

        std::vector<AStarNode>::iterator iterListEnd = io_list.end();

        if( io_list.size() > 0 )
        {
            iterListEnd--;

            for( std::list<const Portal*>::iterator iterPortal = portals.begin(); iterPortal != portals.end() && bInsert; iterPortal++ )
            {
                if( *iterPortal == iterListEnd->getPortal() )
                {
                    bInsert = false;
                }
            }
        }

        if( bInsert )
        {
            AStarNode endNode;
            endNode.setPos( mEndPos );

            if( portals.size() > 0 )
            {
                endNode.setPortal( portals.front() );  // There is a portal at that place.
            }
            else
            {
                endNode.setPortal( NULL ); // That's not a real portal
            }
            io_list.push_back( endNode );
        }

    }

    void AStar::clearData()
    {
        //It's a new path, so we need to reset everything
        mLevel = 0;
        mStartPos = Pathfinding::Math::Vector3::ZERO;
        mEndPos = Pathfinding::Math::Vector3::ZERO;
        mStartNodes.clear();
        mEndNodes.clear();
        mData = NULL;


        for( stdext::hash_map<const Portal*, AStarNode*>::iterator iterHashMap = mVisitedHashMap.begin(); iterHashMap != mVisitedHashMap.end(); iterHashMap++ )
        {
            delete (*iterHashMap).second;
        }

        mVisitedHashMap.clear();

        // These has already been deleted in the mVisitedHashMap.  Just clear the data
        mOpenList.clear();

    }


    void AStar::createStartEndPortal(const Sector* secStart, const Sector* secEnd )
    {
        assert( secStart );
        assert( secEnd );

        for( std::list<Portal*>::const_iterator iter = secStart->getPortals().begin(); iter != secStart->getPortals().end(); iter++ )
        {
            AStarNode* addNode = new AStarNode();
            addNode->setPortal( *iter );
            Pathfinding::Math::Vector3 posInPort = (*iter)->getBox().getCenter();

            addNode->setCost( calculateCost( mStartPos, posInPort ) );
            addNode->setPos( posInPort );
            addNode->setHeuristic( calculateHeuristic( mEndPos, posInPort ) );
            addNode->setScore( addNode->getCost() + addNode->getHeuristic() );

            mStartNodes.push_back( addNode );
        }

        for( std::list<Portal*>::const_iterator iter = secEnd->getPortals().begin(); iter != secEnd->getPortals().end(); iter++ )
        {
            AStarNode* addNode = new AStarNode();
            addNode->setPortal( *iter );
            mEndNodes.push_back( addNode );
            addNode->setScore(0);
        }
    }



    Pathfinding::Math::Real AStar::calculateCost( const Pathfinding::Math::Vector3 &point, const Pathfinding::Math::Vector3 &in_posInPortal )
    {
        return in_posInPortal.distance(point);
    }

    Pathfinding::Math::Real AStar::calculateHeuristic( const Pathfinding::Math::Vector3 &in_endPoint, const Pathfinding::Math::Vector3 &in_posInPortal )
    {
        return in_posInPortal.distance( in_endPoint );
    }

    bool AStar::endPortal( const AStarNode *lower )
    {
        assert( lower );

        bool bEnd = false;
        for( std::list<AStarNode*>::iterator endCell = mEndNodes.begin(); endCell != mEndNodes.end() && !bEnd;++endCell )
        {
            if( lower->getPortal() == (*endCell)->getPortal() )
            {
                bEnd = true;
            }
        }
        return bEnd;
    }

    const Sector* AStar::findZone( const Pathfinding::Math::Vector3 &point )
    {
        // const_cast is bad.  That might be changed.
        return const_cast<Pathfinding::Data*>(mData)->getSectorAtPos( point, mLevel );
    }


    std::list<AStarNode*> AStar::generateSuccessors( const AStarNode *current )
    {
        assert( current );
        assert( current->getPortal() );

        std::list<AStarNode*> successors;

        std::list<Portal*> destPortals1 = current->getPortal()->getDestSector1()->getPortals();

        generateSuccessorsList( successors, destPortals1, current );

        if( current->getPortal()->getDestSector2() )
        {
            std::list<Portal*> destPortals2 = current->getPortal()->getDestSector2()->getPortals();

            generateSuccessorsList( successors, destPortals2, current );
        }

        return successors;
    }

    void AStar::generateSuccessorsList( std::list<AStarNode*>& out_nodes, std::list<Portal*> in_portals, const AStarNode* in_current )
    {
        for( std::list<Portal*>::const_iterator iterPortal = in_portals.begin(); iterPortal != in_portals.end(); ++iterPortal ) 
        {
            // Conditions to know if there must be a successors or not
            if( ( in_current->getPortal() != (*iterPortal) ) && (in_current->getParent() == NULL || in_current->getParent()->getPortal() != (*iterPortal)) )
            {
                // Find if the successors already exist
                stdext::hash_map<const Portal*, AStarNode*>::iterator iterNode = mVisitedHashMap.find( (*iterPortal) );

                AStarNode* nodeSucc = NULL;
                if( iterNode != mVisitedHashMap.end() )
                {
                    nodeSucc = (*iterNode).second;
                }
                else
                {
                    // The successors isn't in closed or open list.  Create a new one.
                    nodeSucc = new AStarNode();
                    nodeSucc->setPortal( *iterPortal );
                }
                out_nodes.push_back( nodeSucc );
            }
        }
    }


    AStarNode* AStar::getLowest()
    {
        AStarNode *lowest = NULL;

        /** From Amit A* */
        // Get the lowest element and remove it from the heap
        lowest = mOpenList.front();

        pop_heap( mOpenList.begin(), mOpenList.end(), *(boost::lambda::_1) > *(boost::lambda::_2) );
        mOpenList.pop_back();

        return lowest;
    }


    void AStar::processSuccessors( std::list<AStarNode*> & successors, AStarNode* current )
    {
        for( std::list<AStarNode*>::iterator iterNodeSucc = successors.begin(); iterNodeSucc != successors.end(); ++iterNodeSucc )
        {
            //Set the cost for the successors. It's cost is the cost for getting from the last one to this one + the previous node cost
            Pathfinding::Math::Vector3 posInPortal = (*iterNodeSucc)->getPortal()->getBox().getCenter();
            Pathfinding::Math::Real costRecalc = calculateCost( current->getPos(), posInPortal ) + current->getCost();

            // Check if the possible new value has a lower cost than this one
            if( (*iterNodeSucc)->getCost() == -1 || costRecalc < (*iterNodeSucc)->getCost() )
            {
                (*iterNodeSucc)->setPos( posInPortal );

                // Check if the node is in the open list
                bool isOpen = (*iterNodeSucc)->getScore() != -1;

                (*iterNodeSucc)->setCost(costRecalc);

                // Set the node and add it to open list
                (*iterNodeSucc)->setParent( current );
                (*iterNodeSucc)->setHeuristic( calculateHeuristic( mEndPos, (*iterNodeSucc)->getPos() ) );
                (*iterNodeSucc)->setScore( (*iterNodeSucc)->getCost() + (*iterNodeSucc)->getHeuristic() );

                if( isOpen )
                {
                    // Replace the heap with the new value.  Using a binary heap in here.  
                    // The *(boost::lambda::_1) > *(boost::lambda::_2) permits
                    // to compare the data of the pointers and not the adress of the pointers
                    push_heap( mOpenList.begin(), findNode( *iterNodeSucc ) + 1, *(boost::lambda::_1) > *(boost::lambda::_2) );
                }
                else
                {
                    addInLists( (*iterNodeSucc) );
                }
            }
        }
    }

    std::vector<AStarNode*>::iterator AStar::findNode( const AStarNode* in_node )
    {
        return std::find (mOpenList.begin (), mOpenList.end (), in_node);
    }

    void AStar::addInLists( AStarNode* io_node )
    {
        // There should always be a portal
        assert( io_node->getPortal() );

        mVisitedHashMap.insert( tdVistedDef::value_type( io_node->getPortal(), io_node ) );

        // Put the data in the list
        mOpenList.push_back( io_node );

        // Replace the heap.  Using a binary heap in here.  The *(boost::lambda::_1) < *(boost::lambda::_2) permits
        // to compare the data of the pointers and not the adress of the pointers
        push_heap( mOpenList.begin(), mOpenList.end(), *(boost::lambda::_1) > *(boost::lambda::_2) );
    }

} // namespace Pathfinding
