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

#ifndef _ABSTRACTPREPATHFINDINGSOURCE_H
#define _ABSTRACTPREPATHFINDINGSOURCE_H


#include "../PathfindingPrerequisites.h"
#include "../PathfindingActor.h"

namespace Pathfinding 
{
    /** @class Physic PathfindingPrePhysic.h "include/PathfindingPrePhysic.h"
    *
    * This abstract class is used to make the physic of the actor. Move and find places where the actor can stand on.
    */
    namespace Pre
    {
        class _PathfindingExport Physic 
        {
        public:

            /** 
            * The move function will take as much steps as it is needed to do to the displacement.  I.E the actor want
            * to move 10 meters to the rigth, it's gonna take say 12 steps.
            * @param[in] in_displacement the displacement to be made by the actor, there should be no y values.
            * @param[in] io_actor the actor that's gonna be used to make displacement.
            * 
            * @return the number of step it taken to go make the displacement to displace the actor from
            * the actual actor place to the other.  This function must displace the actor.
            * Return -1 if it's not possible to displace the actor from that value.  This value will be used to calculate the average cost of the sector.
            */
            virtual Pathfinding::Math::Real move( const Pathfinding::Math::Vector3& displacement, const Actor* in_actor, Pathfinding::Math::Real& out_finalHeight ) = 0;

            /**
            @param[in] in_actor the actor to check if he can stand on place at his current position
            @return the actor can stand on the place where he's actually.
            */ 
            virtual bool canStandOn( const Actor* in_actor ) = 0;

            /** 
            @param[in] in_actor the actor that need to find where he can stand on heights at he's actual X and Z.
            @return The actor can stand on all the returned heights of where he is actually in X,Z.
            For the returned X's and Z's, put the same values as the actor in the returned vectors.
            */
            virtual std::list<Pathfinding::Math::Real> getStandOnHeights( const Actor* in_actor ) = 0;

            /**
            * This function is to prepare any data needed for the pathfinding that is starting to be compute
            */
            virtual void prepare( const Pathfinding::Actor* ) {}
        };
    } //namespace Pre
} // namespace Pathfinding
#endif
