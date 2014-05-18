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

#ifndef _PATHFINDINGPATH_H
#define _PATHFINDINGPATH_H

#include "..\PathfindingPrerequisites.h"

namespace Pathfinding 
{
    namespace Path
    {
        /// Structure to put any elements necessary to calculate a path inherited from this class by the interpolateFromWaypoints method.
        struct InterpolatePathData
        {
            std::vector<AStarNode> * waypoints;  /// Nodes where the path pass. These are the control points.
            int nbPoints;                        /// the number of points that need to be returned between each portals.
            Pathfinding::Data * data;            /// the data of the pathfinding
            int hLevel;                          /// The hierarchical level to interpolate the waypoints
        };

        /** @class PathfindingPath PathfindingPath.h "include/PathfindingPath.h"
        *
        * This is an abstract class for every type of path.  This class give the actor given a list of AStarNode.
        */
        class _PathfindingExport Path
        {
        public:
            Path(void);
            virtual ~Path(void);

            /** Interpolate the path between the nodes
            @param in_interpolate the data necessary to interpolate
            @return all the points on the path.  The returned vector size should be the nbPoints * wayPoints.size()
            */
            virtual std::vector<Pathfinding::Math::Vector3> interpolateFromWaypoints( InterpolatePathData * in_interpolate ) = 0;

            /** Recalculate the points to give a better path.
            @param io_lWaypoints the points that needs to be recalculated.
            @param[in] in_actor the actor that goes into the waypoints.
            */
            virtual void reCalculatePoints( std::vector<Pathfinding::AStarNode> * io_lWaypoints, const Actor * in_actor );

        private:
            /**
            * Calculates if the potential pos would make a straight line with the input points.  If so, the points in the straight line are
            * adjusted to make the straight line.
            *
            * @param io_straightLine the list that is already straight
            * @param[in] in_potentialPos the point to be add in the io_straightLine
            * @param[in] in_actor the actor that goes into the waypoints.
            */
            void calculateStraightLine( std::vector<Pathfinding::AStarNode*> & io_straightLine, const Pathfinding::Math::Vector3 & in_potentialPos, const Actor * in_actor );
        };
    }
}

#endif
