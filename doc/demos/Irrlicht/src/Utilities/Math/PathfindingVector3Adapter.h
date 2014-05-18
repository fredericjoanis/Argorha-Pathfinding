/*
-----------------------------------------------------------------------------
This source file is part of Switch Blade

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


#ifndef _OGREVECTOR3ADAPTER_H
#define _OGREVECTOR3ADAPTER_H

#include "Math/PathfindingVector3.h"

namespace PathIrr
{

/** @class PathfindingVector3Adapter PathfindingVector3Adapter.h "include/GOOFPathfinding/Utilities/Math/PathfindingVector3Adapter.h"
*
* This class is an adapter to interchange from Ogre::Vector3 to Pathfinding::Math::Vector3
*/

class PathfindingVector3Adapter : public Pathfinding::Math::Vector3
{

public:
    PathfindingVector3Adapter( irr::core::vector3df &in_adaptee ) : Pathfinding::Math::Vector3( in_adaptee.X, in_adaptee.Y, in_adaptee.Z )
   {
   }

   PathfindingVector3Adapter( const irr::core::vector3df &in_adaptee ) : Pathfinding::Math::Vector3( in_adaptee.X, in_adaptee.Y, in_adaptee.Z )
   {
   }
};

}


#endif
