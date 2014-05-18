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

#include "Path\PathfindingPathBezier.h"
#include "PathfindingAStarNode.h"
#include "PathfindingPortal.h"
#include "PathfindingData.h"

namespace Pathfinding
{
    namespace Path
    {

        Bezier::Bezier()
        {
        }

        Bezier::~Bezier()
        {
        }

        std::vector<Pathfinding::Math::Vector3> Bezier::interpolateFromWaypoints( InterpolatePathData * in_interpolate )
        {
            bool bRight = false;

            std::vector<Pathfinding::Math::Vector3> retPoints;


            for( unsigned int i = 0; i < in_interpolate->waypoints->size() - 1; i++ )
            {
                // Find the sector in which the points will be.
                Pathfinding::Math::AxisAlignedBox zoneBox;  // The box of the sector

                //Exception case for the first portal which is NULL.  
                if( (*in_interpolate->waypoints)[i].getPortal() == NULL )
                {
                    zoneBox = in_interpolate->data->getSectorAtPos( (*in_interpolate->waypoints)[i].getPos(), in_interpolate->hLevel )->getBox();
                }
                else if( (*in_interpolate->waypoints)[i + 1].getPortal() == NULL )  // This should be the end portal
                {
                    zoneBox = in_interpolate->data->getSectorAtPos( (*in_interpolate->waypoints)[i + 1].getPos(), in_interpolate->hLevel )->getBox();
                }
                // We get the sector by comparing the destinations sectors of the 2 portals.  It is faster then searching for sectors each time.
                else if( (*in_interpolate->waypoints)[i].getPortal()->getDestSector1() == (*in_interpolate->waypoints)[i + 1].getPortal()->getDestSector1() || (*in_interpolate->waypoints)[i].getPortal()->getDestSector1() == (*in_interpolate->waypoints)[i + 1].getPortal()->getDestSector2() )
                {
                    zoneBox = (*in_interpolate->waypoints)[i].getPortal()->getDestSector1()->getBox();
                }
                else if( (*in_interpolate->waypoints)[i].getPortal()->getDestSector2() == (*in_interpolate->waypoints)[i + 1].getPortal()->getDestSector1() || (*in_interpolate->waypoints)[i].getPortal()->getDestSector2() == (*in_interpolate->waypoints)[i + 1].getPortal()->getDestSector2() )
                {
                    zoneBox = (*in_interpolate->waypoints)[i].getPortal()->getDestSector2()->getBox();
                }

                std::vector<Pathfinding::Math::Vector3> lPoints;
                Pathfinding::Math::Vector3 pointStart = (*in_interpolate->waypoints)[i].getPos();
                Pathfinding::Math::Vector3 pointEnd = (*in_interpolate->waypoints)[i + 1].getPos();
                Pathfinding::Math::Vector3 vDist = pointEnd - pointStart;  // Compute the distance between points

                lPoints.push_back( pointStart );

                // To have a more natural curve, we need to inverse the control points. By doing this,
                // it will switch between concave and convex curve.
                if( ( vDist.x >= 0 && vDist.z >= 0 ) || ( vDist.x < 0 && vDist.z < 0 ) )
                {
                    if( bRight )
                    {
                        vDist.x = -vDist.x;
                    }
                    else
                    {
                        vDist.z = -vDist.z;
                    }
                }
                else
                {
                    if( bRight )
                    {
                        vDist.z = -vDist.z;
                    }
                    else
                    {
                        vDist.x = -vDist.x;
                    }
                }

                // Switch between control points going to left or right of the curve
                bRight = !bRight;


                Pathfinding::Math::Real rLength = vDist.normalise();

                float perc = 0.2f; // might be a parameter?
                Pathfinding::Math::Vector3 vControlPoint1 = perc * pointEnd + (1.f - perc) * pointStart;
                Pathfinding::Math::Vector3 vControlPoint2 = perc * pointStart + (1.f - perc) * pointEnd;

                vDist *= rLength * perc;

                vControlPoint1.x += vDist.x;
                vControlPoint1.y += vDist.y;
                vControlPoint1.z += vDist.z;

                vControlPoint2.x += vDist.x;
                vControlPoint2.y += vDist.y;
                vControlPoint2.z += vDist.z;

                if( !zoneBox.intersects( vControlPoint1 ) )
                {
                    if( vControlPoint1.x > zoneBox.getMaximum().x )
                    {
                        vControlPoint1.x = zoneBox.getMaximum().x;
                        // Mathematical function to clip the value properly.  I didn't get the good one.
                        //vControlPoint1 = vControlPoint1 - (pointEnd - pointStart) * (vControlPoint1.x - zoneBox.getMaximum().x) / vDist.x;
                    }
                    else if( vControlPoint1.x < zoneBox.getMinimum().x )
                    {
                        vControlPoint1.x = zoneBox.getMinimum().x;
                    }

                    if( vControlPoint1.y > zoneBox.getMaximum().y )
                    {
                        vControlPoint1.y = zoneBox.getMaximum().y;
                    }
                    else if( vControlPoint1.y < zoneBox.getMinimum().y )
                    {
                        vControlPoint1.y = zoneBox.getMinimum().y;
                    }

                    if( vControlPoint1.z > zoneBox.getMaximum().z )
                    {
                        vControlPoint1.z = zoneBox.getMaximum().z;
                    }
                    else if( vControlPoint1.z < zoneBox.getMinimum().z )
                    {
                        vControlPoint1.z = zoneBox.getMinimum().z;
                    }
                }

                if( !zoneBox.intersects( vControlPoint2 ) )
                {
                    if( vControlPoint2.x > zoneBox.getMaximum().x )
                    {
                        vControlPoint2.x = zoneBox.getMaximum().x;
                        // Mathematical function to clip the value properly.  I didn't get the good one.
                        //vControlPoint2 = vControlPoint2 - (pointEnd - pointStart) * (vControlPoint2.x - zoneBox.getMaximum().x) / vDist.x;
                    }
                    else if( vControlPoint2.x < zoneBox.getMinimum().x )
                    {
                        vControlPoint2.x = zoneBox.getMinimum().x;
                    }

                    if( vControlPoint2.y > zoneBox.getMaximum().y )
                    {
                        vControlPoint2.y = zoneBox.getMaximum().y;
                    }
                    else if( vControlPoint2.y < zoneBox.getMinimum().y )
                    {
                        vControlPoint2.y = zoneBox.getMinimum().y;
                    }

                    if( vControlPoint2.z > zoneBox.getMaximum().z )
                    {
                        vControlPoint2.z = zoneBox.getMaximum().z;
                    }
                    else if( vControlPoint2.z < zoneBox.getMinimum().z )
                    {
                        vControlPoint2.z = zoneBox.getMinimum().z;
                    }
                }


                lPoints.push_back( vControlPoint1 );
                lPoints.push_back( vControlPoint2 );

                lPoints.push_back( pointEnd );


                std::vector<Pathfinding::Math::Vector3> pointToAdd = pointOnCubicBezier( lPoints, in_interpolate->nbPoints );
                retPoints.insert( retPoints.end(), pointToAdd.begin(), pointToAdd.end() );

            }

            return retPoints;
        }

        std::vector<Pathfinding::Math::Vector3> Bezier::pointOnCubicBezier( const std::vector<Pathfinding::Math::Vector3> & lPoints, int numberPoint )
        {
            float   tSquared, tCubed;
            std::vector<Pathfinding::Math::Vector3> ret;

            /* calculate the polynomial coefficients */

            float cx = 3.f * (lPoints[1].x - lPoints[0].x);
            float bx = 3.f * (lPoints[2].x - lPoints[1].x) - cx;
            float ax = lPoints[3].x - lPoints[0].x - cx - bx;

            float cy = 3.f * (lPoints[1].y - lPoints[0].y);
            float by = 3.f * (lPoints[2].y - lPoints[1].y) - cy;
            float ay = lPoints[3].y - lPoints[0].y - cy - by;

            float cz = 3.f * (lPoints[1].z - lPoints[0].z);
            float bz = 3.f * (lPoints[2].z - lPoints[1].z) - cz;
            float az = lPoints[3].z - lPoints[0].z - cz - bz;

            /* calculate the curve point at parameter value t */
            for( int i = 0; i < numberPoint; i++ )
            {
                float t = static_cast<float>(i) / ( numberPoint - 1 );
                tSquared = t * t;
                tCubed = tSquared * t;

                Pathfinding::Math::Vector3 result;
                result.x = (ax * tCubed) + (bx * tSquared) + (cx * t) + lPoints[0].x;
                //result.y = 0.1f;
                result.y = (ay * tCubed) + (by * tSquared) + (cy * t) + lPoints[0].y;
                result.z = (az * tCubed) + (bz * tSquared) + (cz * t) + lPoints[0].z;

                ret.push_back( result );
            }

            return ret;
        }

        std::vector<Pathfinding::Math::Vector3> Bezier::pointOnQuadraticBezier( const std::vector<Pathfinding::Math::Vector3> & lPoints, int numberPoint )
        {
            float   tSquared;
            std::vector<Pathfinding::Math::Vector3> ret;

            /* calculate the polynomial coefficients */
            float b2x = 2.f * lPoints[1].x;
            float b2y = 2.f * lPoints[1].y;
            float b2z = 2.f * lPoints[1].z;

            /* calculate the curve point at parameter value t */

            for( int i = 0; i <= numberPoint; i++ )
            {
                float t = static_cast<float>(i) / numberPoint;
                tSquared = t * t;
                float oneMinusT = 1 - t;
                float oneMinusTSquared = oneMinusT * oneMinusT;

                Pathfinding::Math::Vector3 result;
                result.x = oneMinusTSquared * lPoints[0].x + oneMinusT * t * b2x + tSquared * lPoints[2].x;

                //result.y = 0.1f;
                result.y = oneMinusTSquared * lPoints[0].y + oneMinusT * t * b2y + tSquared * lPoints[2].y;
                result.z = oneMinusTSquared * lPoints[0].z + oneMinusT * t * b2z + tSquared * lPoints[2].z;

                ret.push_back( result );
            }

            return ret;
        }
    } // namespace Path
} // namespace Pathfinding
