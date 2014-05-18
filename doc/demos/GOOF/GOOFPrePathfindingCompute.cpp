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

#include "GOOFPrePathfindingCompute.h"
#include "PathfindingActor.h"
#include "GOOFPathfindingData.h"
#include "PrePathfindingCommon.h"

namespace GOOF {

	bool PrePathfindingCompute::verifySectorContinuity( int dir, const Pathfinding::PrePathfindingSector * pSector, const std::list<Pathfinding::PrePathfindingCell*> listCell )
	{
		return true;
	}

   bool PrePathfindingCompute::verifyFloodFillContinuity()
   {
      return mbContinue;
   }

   PrePathfindingCompute::~PrePathfindingCompute()
   {

   }

   PrePathfindingCompute::PrePathfindingCompute()
   {
      mbContinue = true;
   }

   void PrePathfindingCompute::setContinue( bool in_Continue )
   {
      mbContinue = in_Continue;
   }

   bool PrePathfindingCompute::getContinue()
   {
      return mbContinue;
   }


   void PrePathfindingCompute::fillFinish()
   {
      if( mbContinue )
      {
         static_cast<GOOF::PathfindingData*>(mActor->getData())->clearPartition( mPartID, Pathfinding::PrePathfindingCommon::BASE_LEVEL );
      }
   }

   void PrePathfindingCompute::setPartitionID( const Ogre::String& in_partID )
   {
      mPartID = in_partID;
   }

} // namespace GOOF
