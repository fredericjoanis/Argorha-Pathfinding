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

#ifndef _PATHFINDINGPATHLCARDINALSPLINE_H
#define _PATHFINDINGPATHLCARDINALSPLINE_H

#include "..\PathfindingPrerequisites.h"
#include "PathfindingPath.h"

namespace Pathfinding
{
    namespace Path
    {

        /** @class PathfindingPathCardinalSpline PathfindingPathCardinalSpline.h "include/PathfindingPathCardinalSpline.h"
        *
        * This class calculates a cardinal spline.
        */
        class _PathfindingExport CardinalSpline : public Path
        {
        public:
            CardinalSpline(void);
            virtual ~CardinalSpline(void);

            /** Interpolate the path between the nodes
            @param in_interpolate the data necessary to interpolate
            @return all the points on the path.  The returned vector size should be the nbPoints * wayPoints.size()
            */
            virtual std::vector<Pathfinding::Math::Vector3> interpolateFromWaypoints( InterpolatePathData * in_interpolate );
        };
    }
}

#endif
