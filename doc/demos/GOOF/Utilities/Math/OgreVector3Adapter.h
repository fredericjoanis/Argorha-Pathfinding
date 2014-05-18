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

#include "OgreVector3.h"

namespace GOOF
{

   /** @class OgreVector3Adapter OgreVector3Adapter.h "include/GOOFPathfinding/Utilities/Math/OgreVector3Adapter.h"
   *
   * This class is an adapter to interchange from PathfindingMath::Vector3 to Ogre::Vector3
   */

class OgreVector3Adapter : public Ogre::Vector3
{

public:
   OgreVector3Adapter( PathfindingMath::Vector3 &in_adaptee ) : Ogre::Vector3( in_adaptee.x, in_adaptee.y, in_adaptee.z )
   {
   }

   OgreVector3Adapter( const PathfindingMath::Vector3 &in_adaptee ) : Ogre::Vector3( in_adaptee.x, in_adaptee.y, in_adaptee.z )
   {
   }
};

}


#endif
