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

#include "GOOFPathfindingData.h"
#include "GOOFCorePartition.h"
#include "GOOFGridPartitionManager.h"
#include "Utilities/Math/OgreAxisAlignedBoxAdapter.h"


namespace GOOF {


	PathfindingData::PathfindingData() : Pathfinding::PathfindingData()
	{
	}

   void PathfindingData::addSector( Pathfinding::PathfindingSector * io_sector, int in_hLevel )
	{
      CorePartition* partition = mGameObjectMgr->getPartitionManager()->getPartition( OgreVector3Adapter( io_sector->getBox().getCenter() ) );
		mSectorMap[in_hLevel][partition->getID()].push_back( io_sector );
	}

   void PathfindingData::addPortal( Pathfinding::PathfindingPortal * io_portal, int in_hLevel )
   {
      GOOF::GridPartitionManager* gridPart = dynamic_cast<GridPartitionManager*>( mGameObjectMgr->getPartitionManager() );
      GOOF::CorePartition *partition = mGameObjectMgr->getPartitionManager()->getPartition( OgreVector3Adapter( io_portal->getBox().getCenter() ) );

      if( partition->getWorldPartitionBox().contains( OgreAxisAlignedBoxAdapter( io_portal->getBox() ) ) )
      {
         mPortalMap[in_hLevel][partition->getID()].push_back( io_portal );
      }
      else
      {
         std::vector<CorePartition*> partitions;
         partition->enumerateConnectedPartitions( partitions );
         
         for( std::vector<CorePartition*>::iterator iterPart = partitions.begin(); iterPart != partitions.end(); iterPart++ )
         {
            if( (*iterPart)->getWorldPartitionBox().intersects( OgreAxisAlignedBoxAdapter( io_portal->getBox() ) ) )
            {
               mPortalMap[in_hLevel][(*iterPart)->getID()].push_back( io_portal );
            }
         }
      }
   }

    bool PathfindingData::suppressSector( Pathfinding::PathfindingSector * io_sector, int in_hLevel )
	{
      CorePartition* partition = mGameObjectMgr->getPartitionManager()->getPartition( OgreVector3Adapter( io_sector->getBox().getCenter() ) );
      mSectorMap[in_hLevel][partition->getID()].remove( io_sector );

      delete io_sector;

		return true;
	}

   bool PathfindingData::suppressPortal( Pathfinding::PathfindingPortal * io_portal, int in_hLevel )
	{
      GOOF::GridPartitionManager* gridPart = dynamic_cast<GridPartitionManager*>( mGameObjectMgr->getPartitionManager() );
      GOOF::CorePartition *partition = gridPart->getPartition( OgreVector3Adapter( io_portal->getBox().getCenter() ) );

      if( partition->getWorldPartitionBox().contains( OgreAxisAlignedBoxAdapter( io_portal->getBox() ) ) )
      {
         mPortalMap[in_hLevel][partition->getID()].remove( io_portal );
      }
      else
      {
         std::vector<CorePartition*> partitions;
         partition->enumerateConnectedPartitions( partitions );
         
         for( std::vector<CorePartition*>::iterator iterPart = partitions.begin(); iterPart != partitions.end(); iterPart++ )
         {
            if( (*iterPart)->getWorldPartitionBox().intersects( OgreAxisAlignedBoxAdapter( io_portal->getBox() ) ) )
            {
               mPortalMap[in_hLevel][(*iterPart)->getID()].remove( io_portal );
            }
         }
      }

      delete io_portal;

      return true;
	}


	std::list<const Pathfinding::PathfindingPortal*> PathfindingData::getPortalsAtPos(const PathfindingMath::Vector3 & pos, int in_hLevel )
	{
		std::list<const Pathfinding::PathfindingPortal*> ret;
      CorePartition* partition = mGameObjectMgr->getPartitionManager()->getPartition( OgreVector3Adapter( pos ) );
      
      for( std::list<Pathfinding::PathfindingPortal*>::const_iterator iter = mPortalMap[in_hLevel][partition->getID()].begin(); iter != mPortalMap[in_hLevel][partition->getID()].end(); ++iter )
		{
		   if( (*iter)->getBox().intersects( pos ) )
		   {
			   ret.push_back( (*iter) );
		   }
		}
		return ret;
	}

   Pathfinding::PathfindingSector* PathfindingData::getSectorAtPos( const PathfindingMath::Vector3 & pos, int in_hLevel )
	{
		Pathfinding::PathfindingSector* ret = NULL;
      CorePartition* partition = mGameObjectMgr->getPartitionManager()->getPartition( OgreVector3Adapter( pos ) );

      for( std::list<Pathfinding::PathfindingSector*>::iterator iter = mSectorMap[in_hLevel][partition->getID()].begin(); iter != mSectorMap[in_hLevel][partition->getID()].end() && ret == NULL; ++iter )
      {
         if( (*iter)->getBox().intersects( pos ) )
         {
	         ret = (*iter);
         }
      }
      
		return ret;
	}

   void PathfindingData::setGameObjectMgr( CoreGameObjectManager* gameObjectMgr )
   {
      mGameObjectMgr = gameObjectMgr;
   }

   void PathfindingData::clearPartition( const Ogre::String in_ID, int in_hLevel )
   {
      for( std::list<Pathfinding::PathfindingSector*>::iterator iter = mSectorMap[in_hLevel][in_ID].begin(); iter != mSectorMap[in_hLevel][in_ID].end(); )
		{
         Pathfinding::PathfindingSector* sector = *iter;
         // Move node before deleting
         ++iter;
         delete sector;
		}
      mSectorMap[in_hLevel][in_ID].clear();

      GOOF::GridPartitionManager* gridPart = dynamic_cast<GridPartitionManager*>( mGameObjectMgr->getPartitionManager() );
      GOOF::CorePartition *partition = gridPart->getPartition( in_ID );

      for( std::list<Pathfinding::PathfindingPortal*>::iterator iter = mPortalMap[in_hLevel][in_ID].begin(); iter != mPortalMap[in_hLevel][in_ID].end(); )
		{
         Pathfinding::PathfindingPortal* portal = *iter;
         // move iterator before deleting.
         ++iter;
         suppressPortal( portal, in_hLevel );

         //if( partition->getWorldPartitionBox().contains( portal->getBox() ) )
         //{
         //   mPortalMap[in_hLevel][in_ID].remove( portal );
         //}
         //else
         //{
         //   std::vector<CorePartition*> partitions;
         //   partition->enumerateConnectedPartitions( partitions );
         //   
         //   for( std::vector<CorePartition*>::iterator iterPart = partitions.begin(); iterPart != partitions.end(); iterPart++ )
         //   {
         //      if( (*iterPart)->getWorldPartitionBox().intersects( portal->getBox() )
         //      {
         //         mPortalMap[in_hLevel][(*iterPart)->getID()].remove( portal );
         //      }
         //   }
         //}
         //delete portal;
		}

   }

   const std::list<Pathfinding::PathfindingSector*>* PathfindingData::getSectorsFromPartition( const Ogre::String in_ID, int in_hLevel )
   {
      return &mSectorMap[in_hLevel][in_ID];
   }

   const std::list<Pathfinding::PathfindingPortal*>* PathfindingData::getPortalsFromPartition( const Ogre::String in_ID, int in_hLevel )
   {
      return &mPortalMap[in_hLevel][in_ID];
   }

   std::vector<const Pathfinding::PathfindingPortal*> PathfindingData::getPortalsInZone( const PathfindingMath::AxisAlignedBox & in_box, int in_hLevel )
   {
      std::vector<const Pathfinding::PathfindingPortal*> retPortals;

      std::vector<CorePartition*> partitions;

      CorePartition* middlePart = mGameObjectMgr->getPartitionManager()->getPartition( OgreVector3Adapter( in_box.getCenter() ) );

      if( !middlePart->getWorldPartitionBox().contains( OgreAxisAlignedBoxAdapter( in_box ) ) )
      {
         middlePart->enumerateConnectedPartitions( partitions );
      }

      partitions.push_back( middlePart );

      for( std::vector<CorePartition*>::iterator iterPartitions = partitions.begin(); iterPartitions != partitions.end(); iterPartitions++ )
      {
         for( std::list<Pathfinding::PathfindingPortal*>::iterator iterPort = mPortalMap[in_hLevel][ (*iterPartitions)->getID() ].begin(); iterPort != mPortalMap[in_hLevel][ (*iterPartitions)->getID() ].end(); iterPort++ )
         {
            if( in_box.intersects( (*iterPort)->getBox() ) )
            {
               retPortals.push_back( *iterPort );
            }
         }
      }

      return retPortals;
   }

} // namespace GOOF
