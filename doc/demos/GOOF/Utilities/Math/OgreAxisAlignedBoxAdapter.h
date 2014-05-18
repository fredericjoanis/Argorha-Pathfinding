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


#ifndef _OGREAXISALIGNEDBOXADAPTER_H
#define _OGREAXISALIGNEDBOXADAPTER_H

#include "OgreAxisAlignedBox.h"
#include "OgreVector3Adapter.h"

namespace GOOF
{
   /** @class OgreAxisAlignedBoxAdapter OgreAxisAlignedBoxAdapter.h "include/GOOFPathfinding/Utilities/Math/OgreAxisAlignedBoxAdapter.h"
   *
   * This class is an adapter to interchange from PathfindingMath::AxisAlignedBox to Ogre::AxisAlignedBox
   */

class OgreAxisAlignedBoxAdapter : public Ogre::AxisAlignedBox
{

public:
   OgreAxisAlignedBoxAdapter( PathfindingMath::AxisAlignedBox &in_adaptee ) : Ogre::AxisAlignedBox( OgreVector3Adapter( in_adaptee.getMinimum() ), OgreVector3Adapter( in_adaptee.getMaximum() ) )
   {
   }

   OgreAxisAlignedBoxAdapter( const PathfindingMath::AxisAlignedBox &in_adaptee ) : Ogre::AxisAlignedBox( OgreVector3Adapter( in_adaptee.getMinimum() ), OgreVector3Adapter( in_adaptee.getMaximum() ) )
   {
   }

};

}


#endif
