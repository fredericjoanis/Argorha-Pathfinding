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

#include "Path\PathfindingPathLinear.h"
#include "PathfindingAStarNode.h"
#include "PathfindingPortal.h"
#include "PathfindingActor.h"

namespace Pathfinding
{
    namespace Path
    {

        Linear::Linear()
        {
        }

        Linear::~Linear()
        {
        }

        std::vector<Pathfinding::Math::Vector3> Linear::interpolateFromWaypoints( InterpolatePathData * in_interpolate )
        {
            assert( in_interpolate->nbPoints - 1 ); // Division by 0

            std::vector<Pathfinding::Math::Vector3> vRet;

            if( in_interpolate->waypoints->size() > 1 )
            {
                float fOneOnNbPoints = 1.f / ( in_interpolate->nbPoints - 1);

                vRet.push_back( in_interpolate->waypoints->begin()->getPos() );

                Pathfinding::Math::Vector3 pointStart = in_interpolate->waypoints->begin()->getPos();
                Pathfinding::Math::Vector3 pointEnd;

                for( std::vector<AStarNode>::const_iterator iter = in_interpolate->waypoints->begin(); iter != in_interpolate->waypoints->end(); )
                {
                    iter++;
                    if( iter != in_interpolate->waypoints->end() )
                    {
                        pointEnd = iter->getPos();

                        for( int i = 1; i < in_interpolate->nbPoints; i++ )
                        {
                            float perc = i * fOneOnNbPoints;
                            vRet.push_back( perc * pointEnd + (1.f - perc) * pointStart );
                        }
                        pointStart = pointEnd;
                    }
                }
            }

            return vRet;
        }
    }
}
