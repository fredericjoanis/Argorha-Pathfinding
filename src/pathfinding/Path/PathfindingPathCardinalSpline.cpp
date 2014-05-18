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

#include "Path\PathfindingPathCardinalSpline.h"
#include "PathfindingAStarNode.h"
#include "PathfindingPortal.h"
#include "PathfindingData.h"


namespace Pathfinding
{
    namespace Path
    {
        CardinalSpline::CardinalSpline(void)
        {
        }

        CardinalSpline::~CardinalSpline(void)
        {
        }

        std::vector<Pathfinding::Math::Vector3> CardinalSpline::interpolateFromWaypoints( InterpolatePathData * in_interpolate )
        {

            std::vector<Pathfinding::Math::Vector3> aPointsRet;
            std::vector<Pathfinding::Math::Vector3> aPointsNormalized;

            if( in_interpolate->waypoints->size() <= 1 )
            {
                return aPointsRet;
            }

            // First and last element must be doubled for the Cardinal spline to work.

            aPointsNormalized.push_back( (*in_interpolate->waypoints)[0].getPos() );
            for( unsigned int i = 0; i < in_interpolate->waypoints->size(); i++ )
            {
                aPointsNormalized.push_back( (*in_interpolate->waypoints)[i].getPos() );
            }
            aPointsNormalized.push_back( (*in_interpolate->waypoints)[ in_interpolate->waypoints->size() - 1 ].getPos() );

            // Cardinal spline formula 
            // mi = 1 / 2 * ( 1 - C ) * ( pi+1 - pi-1 ) - Sources Wikipedia http://en.wikipedia.org/wiki/Cardinal_spline
            // C = Tension
            // starting point pi and an ending point pi+1 with starting tangent mi and ending tangent mi+1 with the tangents defined by
            float fOneMinusTension = 0.35f;  // c = 0.3f | ( 1 / 2 ) * ( 1 - c );
            std::vector<Pathfinding::Math::Vector3> lMi;         // list of all the point tangents

            // The first tangent is a tricky one, just calculate it as is
            lMi.push_back( Pathfinding::Math::Vector3( 0.f, 0.f, 0.f  ));   // Delta Y is 0 for the flat line
            for( unsigned int i = 1; i < aPointsNormalized.size() - 1; i++ )
            {
                lMi.push_back( Pathfinding::Math::Vector3( fOneMinusTension * ( aPointsNormalized[i + 1].x - aPointsNormalized[i - 1].x ), fOneMinusTension * ( aPointsNormalized[i + 1].y - aPointsNormalized[i - 1].y ), fOneMinusTension * ( aPointsNormalized[i + 1].z - aPointsNormalized[i - 1].z ) ));
            }
            lMi.push_back( Pathfinding::Math::Vector3( 0.f, 0.f, 0.f ));


            float fOneOnNbElement      = 1.f / static_cast<float>(in_interpolate->nbPoints - 1);

            /* Loop to generate values for the curve
            The curve is a Cubic Hermite spline.  http://en.wikipedia.org/wiki/Cubic_Hermite_spline 
            */
            // Loop between each point.  The first and last one are arbitrary values, don't need to interpolate between them.
            for( unsigned int pos = 1; pos < aPointsNormalized.size() - 2; pos++ )
            {
                // Find the sector in which the points will be.
                Pathfinding::Math::AxisAlignedBox zoneBox;  // The box of the sector

                //Exception case for the first portal which is NULL.  
                if( (*in_interpolate->waypoints)[pos - 1].getPortal() == NULL )
                {
                    zoneBox = in_interpolate->data->getSectorAtPos( (*in_interpolate->waypoints)[pos - 1].getPos(), in_interpolate->hLevel )->getBox();
                }
                else if( (*in_interpolate->waypoints)[pos].getPortal() == NULL )  // This should be the end portal
                {
                    zoneBox = in_interpolate->data->getSectorAtPos( (*in_interpolate->waypoints)[pos].getPos(), in_interpolate->hLevel )->getBox();
                }
                // We get the sector by comparing the destinations sectors of the 2 portals.  It is faster then searching for sectors each time.
                else if( (*in_interpolate->waypoints)[pos - 1].getPortal()->getDestSector1() == (*in_interpolate->waypoints)[pos].getPortal()->getDestSector1() || (*in_interpolate->waypoints)[pos - 1].getPortal()->getDestSector1() == (*in_interpolate->waypoints)[pos].getPortal()->getDestSector2() )
                {
                    zoneBox = (*in_interpolate->waypoints)[pos - 1].getPortal()->getDestSector1()->getBox();
                }
                else if( (*in_interpolate->waypoints)[pos - 1].getPortal()->getDestSector2() == (*in_interpolate->waypoints)[pos].getPortal()->getDestSector1() || (*in_interpolate->waypoints)[pos - 1].getPortal()->getDestSector2() == (*in_interpolate->waypoints)[pos].getPortal()->getDestSector2() )
                {
                    zoneBox = (*in_interpolate->waypoints)[pos - 1].getPortal()->getDestSector2()->getBox();
                }

                for( int i = 0; i < in_interpolate->nbPoints; i++ )
                {
                    float t   = i * fOneOnNbElement;
                    float h1  = 2 * t * t * t - 3 * t * t + 1;                // h1   = 2t^3-3t^2+1
                    float h2  = t * t * t - 2 * t * t + t;                    // h2   = t^3-2t^2+t
                    float h3  = -2 * t * t * t + 3 * t * t;                   // h3   = -2t^3+3t^2
                    float h4  = t * t * t - t * t;                            // h4   = t^3-t^2

                    Pathfinding::Math::Vector3 pt;

                    pt.x = h1 * aPointsNormalized[ pos ].x + h2 * lMi[ pos ].x + h3 * aPointsNormalized[ pos + 1 ].x + h4 * lMi[ pos + 1 ].x;// p(t)     = h00t * p0 + h10t * m0 + h01t * p1 + h11t * m1
                    if( pt.x > zoneBox.getMaximum().x )
                    {
                        pt.x = zoneBox.getMaximum().x;
                    }
                    else if(  pt.x < zoneBox.getMinimum().x )
                    {
                        pt.x = zoneBox.getMinimum().x;
                    }

                    pt.y = h1 * aPointsNormalized[ pos ].y + h2 * lMi[ pos ].y + h3 * aPointsNormalized[ pos + 1 ].y + h4 * lMi[ pos + 1 ].y;// p(t)     = h00t * p0 + h10t * m0 + h01t * p1 + h11t * m1
                    if( pt.y > zoneBox.getMaximum().y )
                    {
                        pt.y = zoneBox.getMaximum().y;
                    }
                    else if(  pt.y < zoneBox.getMinimum().y )
                    {
                        pt.y = zoneBox.getMinimum().y;
                    }

                    pt.z = h1 * aPointsNormalized[ pos ].z + h2 * lMi[ pos ].z + h3 * aPointsNormalized[ pos + 1 ].z + h4 * lMi[ pos + 1 ].z;// p(t)     = h00t * p0 + h10t * m0 + h01t * p1 + h11t * m1
                    if( pt.z > zoneBox.getMaximum().z )
                    {
                        pt.z = zoneBox.getMaximum().z;
                    }
                    else if(  pt.z < zoneBox.getMinimum().z )
                    {
                        pt.z = zoneBox.getMinimum().z;
                    }

                    // Value clipping should go here

                    aPointsRet.push_back( pt );
                }
            }
            return aPointsRet;
        }
    }
}