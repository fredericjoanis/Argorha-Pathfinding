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

#ifndef _GOOFPREPATHFINDINGCOMPUTE_H
#define _GOOFPREPATHFINDINGCOMPUTE_H

#pragma once

#include "GOOFPathfindingGameSystemPrerequisites.h"
#include "..\Pathfinding\PrePathfindingCompute.h"
#include "..\Pathfinding\PrePathfindingCell.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include "GOOFCoreGameObjectManager.h"

namespace GOOF {

/** @class PrePathfindingCompute GOOFPrePathfindingCompute.h "include/GOOFPrePathfindingCompute.h"
*
* This class is used to implement functions that PrePathfindingCompute cannot know because it's related to GOOF and arbitrary decisions.
*/
class _GOOFPathfindingGameSystemExport PrePathfindingCompute : public Pathfinding::PrePathfindingCompute 
{
public:
   PrePathfindingCompute();
   virtual ~PrePathfindingCompute();

   /**
    @param[in] in_continue sets mContinue
   */
   void setContinue( bool in_continue );

   /**
    @return mContinue
   */
   bool getContinue();

   /**
    sets the partitions that is being calculated.
   */
   void setPartitionID( const Ogre::String& in_partID );

protected:
   /**
      This function checks whether the sector can continue in this direction.  This function is in this inherited class because the decision
      must be taken by the designer to know when a sector should stop.  I.E A new surface at that place, too much slope, etc.
      @param[in] in_dir the direction in which the sector is being created
      @param[in] in_sector the actual sector, before adding the line
      @param[in] in_listCell the list of cells that is going to be added to the sector
   */
   virtual bool verifySectorContinuity( int in_dir, const Pathfinding::PrePathfindingSector * in_sector, const std::list<Pathfinding::PrePathfindingCell*> in_listCell );
   
   /**
    @return true if the flood fill has not been interrupted by something. I.E The partition has been dirtified.
   */
   virtual bool verifyFloodFillContinuity();

   /** 
     This is call once the floodFill or fillAll is completed.
   */
   virtual void fillFinish();



private:
   /// The variable to know if the flood fill can continue
   bool mbContinue;

   /// The partition that is being calculated
   Ogre::String mPartID;

   /// Private mutex, not allowed to lock from outside
	OGRE_AUTO_MUTEX
};

} // namespace GOOF
#endif
