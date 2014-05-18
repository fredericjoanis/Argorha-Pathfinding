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

#ifndef _PATHFINDINGASTAR_H
#define _PATHFINDINGASTAR_H


#include "PathfindingPrerequisites.h"
#include "Math/PathfindingVector3.h"
#include "PathfindingAStarNode.h"
#include "PathfindingPortal.h"
#include "Path/PathfindingPath.h"
#include "PathfindingActor.h"

namespace Pathfinding {


    /**
    * @class PathfindingAStar PathfindingAStar.h "include/PathfindingAStar.h"
    * @brief Class used to calculate a path between 2 points with sectors and portals system.
    * 
    * The algorithm used is this one : \n
    * 1 Create a node containing the goal state node_goal // All the portals surrounding the m_EndPos \n
    * 2 Create a node containing the start state node_start // All the portals surrounding the m_StartPos \n
    * 3 Put node_start on the open list  \n
    * 4 while the OPEN list is not empty \n
    * 5 { \n
    * 6     Get the node off the open list with the lowest f and call it node_current \n
    * 7     if node_current is the same state as node_goal we have found the solution; break from the while loop \n
    * 8     Generate each state node_successor that can come after node_current \n
    * 9     for each node_successor of node_current \n
    * 10    { \n
    * 11        Set the mCost of node_successor to be the mCost of node_current plus the mCost to get to node_successor from node_current \n
    * 12        find node_successor on the OPEN list \n
    * 13        if node_successor is on the OPEN list but the existing one is as good or better then discard this successor and continue \n
    * 14        if node_successor is on the CLOSED list but the existing one is as good or better then discard this successor and continue \n
    * 15        Remove occurences of node_successor from OPEN and CLOSED \n
    * 16        Set the parent of node_successor to node_current \n
    * 17        Set h to be the estimated distance to node_goal (Using the mHeuristric function) \n
    * 18        Add node_successor to the OPEN list \n
    * 19    } \n
    * 20    Add node_current to the CLOSED list \n
    * 21} \n
    */

    class _PathfindingExport AStar {
    private:

        typedef stdext::hash_map<const Portal*, AStarNode*> tdVistedDef; /// Type definition for the visited map

        tdVistedDef mVisitedHashMap;                    /// The close and open list of the nodes.
        std::vector<AStarNode*> mOpenList;   /// The open list of the nodes

        Pathfinding::Math::Vector3 mStartPos;         /// The start position. This is a member variable because a lot of functions use it.
        Pathfinding::Math::Vector3 mEndPos;           /// The end position. This is a member variable because a lot of functions use it.
        Data * mData;         /// The data to go trough. This is a member variable because a lot of functions use it.


        int mLevel;                      /// The level that is actually processed. Not yet implemented

        std::list<AStarNode*> mStartNodes; /// All nodes (portals) surrounding the start position
        std::list<AStarNode*> mEndNodes;   /// All nodes (portals) surrounding the end position

    public:
        AStar() {}
        virtual ~AStar();

        /**
        * Calculate the optimal path between the start and endPoint.
        * @param in_start The place where the pathfinding start
        * @param in_end The place where the pathfinding end
        * @param in_data The data where the portals and sectors are
        * @param in_hLevel the hierarchical level into which the pathfinding is going to be made
        * @return the array of PathfindingAStarNode with the Best path possible.
        */
        std::vector<AStarNode>      getPath(const Pathfinding::Math::Vector3 & in_start, const Pathfinding::Math::Vector3 & in_end, Data *in_data, const int in_hLevel );

        /**
        * Calculate the optimal path between the start and endPoint, hierarchiacaly.
        *
        * @param in_start The place where the pathfinding start
        * @param in_end The place where the pathfinding end
        * @param in_actor The actor that is going to get the path
        * @param in_path An instance of a path to recalculate the levels between the points
        * @param in_hLevelStart the first hierarchical level into which the pathfinding is going to be made
        * @param in_hLevelEnd the last hierarchical level into which the pathfinding is going to be made.  The return is made at that level
        * @return the array of PathfindingAStarNode with the Best path possible at the hierarchically level of in_hLevelEnd
        */
        std::vector<AStarNode>      getPath(const Pathfinding::Math::Vector3 & in_start, const Pathfinding::Math::Vector3 & in_end, const Actor *in_actor, Path::Path* in_path, const int in_hLevelStart, const int in_hLevelEnd );

    private:

        /**
        * This function adds a node in the open list and the visited list
        */
        inline void addInLists( AStarNode* io_node );

        /** 
        * Add the Start and the End Position to a list of PathfindingAStarNode and return it.
        * @param[out] out_list the list to add the surrounding nodes
        */
        void                                addStartEndPortal( std::vector<AStarNode> & out_list );

        /**
        This function clear the data and put everything back for a new path.
        */
        void                                clearData();

        /**
        * Put the AStarNode for the mStartNodes list and the mEndNodes list
        *
        * @param[in] in_secStart The Sector where the start pos is
        * @param[in] in_secEnd The Sector where the start pos is
        */
        void                                createStartEndPortal(const Sector* in_secStart, const Sector* in_secEnd );

        /**
        * This function calculate the cost that it is to go from the start to the node where it's being checked
        * 
        * @param[in] in_point The point where the path is
        * @param[in] in_posInPortal The position into the portal where it's going
        * 
        * @return The cost of going from in_point to in_posInPortal.
        */
        Pathfinding::Math::Real                             calculateCost( const Pathfinding::Math::Vector3 &in_point, const Pathfinding::Math::Vector3 &in_posInPortal );

        /** 
        * This method calculates the heuristic of the node.  The heuristic is an approximation of the remaining distance
        * until the path is finished.
        *
        * @param[in] in_end The point where the trajectory will end
        * @param[in] in_posInPortal The position into the portal where the path is
        * 
        * @return The cost of going from in_end to in_posInPortal.
        */
        Pathfinding::Math::Real                             calculateHeuristic( const Pathfinding::Math::Vector3 &in_end, const Pathfinding::Math::Vector3 &in_posInPortal );

        /**
        * Find a node and return it's iterator.  For better performance, 
        * a check to see if the node is in the open list would be a good idea.
        * 
        * @param[in] in_node The node to search in the open list.
        * @return the iterator where the node is
        */
        std::vector<AStarNode*>::iterator  findNode( const AStarNode* in_node );

        /**
        * Verify if the actual node is a surrounding node of the endPos
        *
        * @param[in] in_lower = The lowest node, check if this is the end.
        * 
        * @return false if it's not the last portal. Return true if there is no solutions or the end has been found.
        */
        bool                                    endPortal( const AStarNode *in_lower );

        /**
        * @param[in] in_point The point where the sector needs to be find.
        * 
        * @return the sector in which the point is.
        */
        const Sector*             findZone( const Pathfinding::Math::Vector3 &in_point );

        /**
        * @param[in] in_current The node to generate the successors from
        * 
        * @return a list of all the successors of the current node
        */
        std::list<AStarNode*>    generateSuccessors( const AStarNode *in_current );

        /**
        * @param[out] out_nodes The nodes to generate the successors from
        * @param[in] portals to be added in the nodes of successor list
        * @param[in] in_current The node to generate the successors from
        */
        void    generateSuccessorsList( std::list<AStarNode*>& out_nodes, std::list<Portal*> in_portals, const AStarNode *in_current );

        /**
        * @return the lowest mScore value in the open list.
        * This function delete the lowest from the open list and put it in the close list
        */
        AStarNode*                 getLowest();

        /**
        * The function start by adding the start Nodes in the OpenList
        * It loop until a path is found or all nodes have been verified.
        * It take the lowest mScore value, verify this is not the end
        * and generate his successors. 
        * 
        * @return The best path.  empty if no path is found.
        */
        std::vector<AStarNode>    processOpenList();

        /**
        * The function verify if this successor is the lowest mCost
        * possible in the open and close list, 
        * if so it calculate the new mHeuristric and mScore.
        *
        * @param io_successors The list of successors
        * @param io_current The current node of which the successors are coming from
        *
        */
        void                                    processSuccessors( std::list<AStarNode*> & io_successors, AStarNode* io_current );

        /**
        * Adds a node to the open list
        * 
        * @param io_node the node to be added
        */
        void                         addNodeToOpen( AStarNode* io_node );
    };




} // namespace Pathfinding
#endif
