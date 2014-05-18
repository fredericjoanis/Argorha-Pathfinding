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

#ifndef _PATHFINDINGPATHBEZIER_H
#define _PATHFINDINGPATHBEZIER_H

#include "..\PathfindingPrerequisites.h"
#include "PathfindingPath.h"


namespace Pathfinding
{
    namespace Path
    {

        /** @class PathfindingPathBezier PathfindingPathBezier.h "include/PathfindingPathBezier.h"
        *
        * This class calculates a bezier curve.
        */
        class _PathfindingExport Bezier : public Path
        {
        public:
            Bezier(void);
            virtual ~Bezier(void);

            /** Interpolate the path between the nodes
            @param in_interpolate the data necessary to interpolate
            @return all the points on the path.  The returned vector size should be the nbPoints * wayPoints.size()
            */
            virtual std::vector<Pathfinding::Math::Vector3> interpolateFromWaypoints( InterpolatePathData * in_interpolate );

            /**
            Calculates a quadratic bezier curve
            @param in_lPoints the points with which you do the new curve
            @param in_numberPoint the number of points to be calculated on the curve
            @return a vector with points on the curve
            */
            std::vector<Pathfinding::Math::Vector3> pointOnQuadraticBezier( const std::vector<Pathfinding::Math::Vector3> & in_lPoints, int in_numberPoint );

            /**
            Calculates a cubic bezier curve
            @param in_lPoints the points with which you do the new curve
            @param in_numberPoint the number of points to be calculated on the curve
            @return a vector with points on the curve
            */
            std::vector<Pathfinding::Math::Vector3> pointOnCubicBezier( const std::vector<Pathfinding::Math::Vector3> & in_lPoints, int in_numberPoint );
        };
    }
}

#endif
