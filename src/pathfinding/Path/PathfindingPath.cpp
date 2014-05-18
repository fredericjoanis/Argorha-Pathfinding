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

#include "Path\PathfindingPath.h"
#include "PathfindingAStarNode.h"
#include "PathfindingPortal.h"
#include "PathfindingActor.h"

namespace Pathfinding
{
    namespace Path
    {

        Path::Path()
        {
        }

        Path::~Path()
        {
        }

        void Path::reCalculatePoints( std::vector<Pathfinding::AStarNode> * io_lWaypoints, const Actor * in_actor )
        {
            if( io_lWaypoints->size() > 2 ) // Make sure we can interpolate points
            {
                std::vector<Pathfinding::AStarNode*> straightLine;
                straightLine.push_back( &((*io_lWaypoints)[0]) );

                for( unsigned int i = 1; i < io_lWaypoints->size() - 1; i++ )
                {
                    Pathfinding::Math::AxisAlignedBox portalBox   = (*io_lWaypoints)[i].getPortal()->getBox();
                    Pathfinding::Math::Vector3 vecBefore    = (*io_lWaypoints)[i - 1].getPos();
                    Pathfinding::Math::Vector3 vecAfter     = (*io_lWaypoints)[i + 1].getPos();
                    Pathfinding::Math::Vector3 potentialPos = ( vecBefore + vecAfter ) * 0.5f;

                    // Find on which way is positioned the portalBox to put the point at the intersection of the sectors
                    if( portalBox.getMaximum().x - portalBox.getMinimum().x == 2 * in_actor->getSmallVector().x )
                    {
                        potentialPos.x = portalBox.getCenter().x;
                    }
                    else if( portalBox.getMaximum().y - portalBox.getMinimum().y == 2 * in_actor->getSmallVector().y )
                    {
                        potentialPos.y = portalBox.getCenter().y;
                    } 
                    else if( portalBox.getMaximum().z - portalBox.getMinimum().z == 2 * in_actor->getSmallVector().z )
                    {
                        potentialPos.z = portalBox.getCenter().z;
                    }

                    // If the potentialPos intersect the portalBox, no need to continue calculating
                    if( !portalBox.intersects( potentialPos ) )
                    {
                        // Find a better position
                        if( potentialPos.x > portalBox.getMaximum().x )
                        {
                            potentialPos.x = portalBox.getMaximum().x - in_actor->getBox().getMaximum().x;
                        }
                        else if( potentialPos.x < portalBox.getMinimum().x )
                        {
                            potentialPos.x = portalBox.getMinimum().x + in_actor->getBox().getMaximum().x;
                        }

                        if( potentialPos.y > portalBox.getMaximum().y )
                        {
                            potentialPos.y = portalBox.getMaximum().y - in_actor->getBox().getMaximum().y;
                        }
                        else if( potentialPos.y < portalBox.getMinimum().y )
                        {
                            potentialPos.y = portalBox.getMinimum().y - in_actor->getBox().getMaximum().y;
                        }

                        if( potentialPos.z > portalBox.getMaximum().z )
                        {
                            potentialPos.z = portalBox.getMaximum().z - in_actor->getBox().getMaximum().z;
                        }
                        else if( potentialPos.z < portalBox.getMinimum().z )
                        {
                            potentialPos.z = portalBox.getMinimum().z + in_actor->getBox().getMaximum().z;
                        }
                    }

                    (*io_lWaypoints)[i].setPos( potentialPos );

                    calculateStraightLine( straightLine, potentialPos, in_actor );

                    straightLine.push_back( &((*io_lWaypoints)[i]) );
                }

                // Calculate with the last node
                calculateStraightLine( straightLine, ((*io_lWaypoints)[io_lWaypoints->size() - 1].getPos() ), in_actor );

                // Make sure there is not 2 points at the same place.
                std::vector<Pathfinding::AStarNode>::iterator iterPrev = io_lWaypoints->begin();
                std::vector<Pathfinding::AStarNode>::iterator iterActu = io_lWaypoints->begin();
                iterActu++;

                while( iterActu != io_lWaypoints->end() )
                {
                    if( iterActu->getPos() == iterPrev->getPos() )
                    {
                        // 2 points are equals, erase the previous one.
                        io_lWaypoints->erase( iterPrev );
                        // Replace the iterPrev at one under iterActu
                        iterPrev = iterActu;
                        iterPrev--;
                    }
                    iterActu++;
                    iterPrev++;
                }
            }
        }

        void Path::calculateStraightLine( std::vector<Pathfinding::AStarNode*> & io_straightLine, const Pathfinding::Math::Vector3 & in_potentialPos, const Actor * in_actor )
        {
            if( io_straightLine.size() > 1 ) // Make sure we can interpolate points
            {
                float perc = 0.f; // The percentage of where the point is compare to the straight line.
                bool bIsStraight = true;
                std::vector<Pathfinding::AStarNode*>::iterator iterStraight = io_straightLine.begin();
                std::vector<Pathfinding::Math::Vector3> listPotentialPos;

                Pathfinding::Math::Vector3 diff = io_straightLine[0]->getPos() - in_potentialPos; // The first element minus the last element

                Pathfinding::Math::Vector3 oneOnDiff; // inverse of diff

                oneOnDiff.x = diff.x == 0 ? 0.f : 1.f / diff.x;  // If the diff is 0, just put 0 back, else divide.
                oneOnDiff.y = diff.y == 0 ? 0.f : 1.f / diff.y;  // If the diff is 0, just put 0 back, else divide.
                oneOnDiff.z = diff.z == 0 ? 0.f : 1.f / diff.z;  // If the diff is 0, just put 0 back, else divide.

                //Start at the first element
                for( unsigned int iStraight = 1; iStraight < io_straightLine.size() && bIsStraight; iStraight++ )
                {
                    Pathfinding::Math::Vector3 potentialPosStraight;
                    Pathfinding::Math::AxisAlignedBox portalBox = io_straightLine[iStraight]->getPortal()->getBox();

                    if( portalBox.getMaximum().x - portalBox.getMinimum().x == 2 * in_actor->getSmallVector().x )
                    {
                        perc = ( io_straightLine[iStraight]->getPos().x - in_potentialPos.x ) * oneOnDiff.x;
                        potentialPosStraight.x = io_straightLine[iStraight]->getPos().x;
                        potentialPosStraight.y = diff.y * perc + in_potentialPos.y;
                        potentialPosStraight.z = diff.z * perc + in_potentialPos.z;
                    }
                    else if( portalBox.getMaximum().y - portalBox.getMinimum().y == 2 * in_actor->getSmallVector().y )
                    {
                        perc = ( io_straightLine[iStraight]->getPos().y - in_potentialPos.y ) * oneOnDiff.y;
                        potentialPosStraight.x = diff.x * perc + in_potentialPos.x;
                        potentialPosStraight.y = io_straightLine[iStraight]->getPos().y;
                        potentialPosStraight.z = diff.z * perc + in_potentialPos.z;
                    } 
                    else if( portalBox.getMaximum().z - portalBox.getMinimum().z == 2 * in_actor->getSmallVector().z )
                    {
                        perc = ( io_straightLine[iStraight]->getPos().z - in_potentialPos.z ) * oneOnDiff.z;
                        potentialPosStraight.x = diff.x * perc + in_potentialPos.x;
                        potentialPosStraight.y = diff.y * perc + in_potentialPos.y;
                        potentialPosStraight.z = io_straightLine[iStraight]->getPos().z;
                    }
                    else
                    {
                        // Should not happen; the portal must have a direction
                        assert( false );
                    }

                    if( portalBox.intersects( potentialPosStraight ) )
                    {
                        listPotentialPos.push_back( potentialPosStraight );
                    }
                    else
                    {
                        bIsStraight = false;
                    }
                }

                // The line is no more straight, start a new line
                if( !bIsStraight )
                {
                    io_straightLine.clear();
                }
                else
                {
                    for( unsigned int i = 1; i < io_straightLine.size(); i++ )
                    {
                        io_straightLine[i]->setPos( listPotentialPos[i-1] );
                    }
                }
            }
        }
    } // namespace Path
} // namespace Pathfinding
