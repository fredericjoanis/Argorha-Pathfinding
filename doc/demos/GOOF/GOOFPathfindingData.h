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


#ifndef _GOOFPATHFINDINGDATA_H
#define _GOOFPATHFINDINGDATA_H

#include "GOOFPathfindingGameSystemPrerequisites.h"
#include "..\Pathfinding\PathfindingPrerequisites.h"
#include "..\Pathfinding\PathfindingData.h"
#include "..\Pathfinding\PathfindingSector.h"
#include "..\Pathfinding\PathfindingPortal.h"
#include "GOOFCoreGameObjectManager.h"
#include <list>

namespace GOOF {

   /** @class PathfindingData GOOFPathfindingData.h "include/GOOFPathfindingData.h"
    *
    * This class is used to store the data about pathfinding.
    */

   class _GOOFPathfindingGameSystemExport PathfindingData : public Pathfinding::PathfindingData 
   {
   public:

   PathfindingData();

   /**
     * @param io_sector sector to be added
     * @param in_hLevel the hierarchical level into which the sector is going
   */
    virtual void addSector( Pathfinding::PathfindingSector * io_sector, int in_hLevel );

   /**
      * @param io_portal portal to be added
      * @param in_hLevel the hierarchical level into which the portal is going
   */
    virtual void addPortal( Pathfinding::PathfindingPortal * io_portal, int in_hLevel );

   /** 
     * @param io_sector sector to be suppress and deleted
     * @param in_hLevel the hierarchical level into which the sector is going
     * @Return true if the element has been suppressed 
   */
    virtual bool suppressSector( Pathfinding::PathfindingSector * io_sector, int in_hLevel );

   /** 
     * @param io_portal portal to be suppress and deleted
     * @param in_hLevel the hierarchical level into which the portal is going
     * @Return true if the element has been suppressed 
   */
    virtual bool suppressPortal( Pathfinding::PathfindingPortal * io_portal, int in_hLevel );

   /** 
     * @param[in] in_pos position to find the portals
     * @param in_hLevel the hierarchical level into which the portals are
     * @Return list of portals at the in_pos
   */
   virtual std::list<const Pathfinding::PathfindingPortal*> getPortalsAtPos( const PathfindingMath::Vector3 & pos, int in_hLevel );

   /** 
     * @param[in] in_pos position to find the sectors
     * @param in_hLevel the hierarchical level into which the sector is going
     * @Return the sector at that position
   */
   virtual Pathfinding::PathfindingSector* getSectorAtPos(const PathfindingMath::Vector3 & pos, int in_hLevel);

   /** 
      * @param[in] in_box the box to find the portals
      * @param in_hLevel the hierarchical level into which the portals are
      * @Return list of portals that intersects the box
   */
    virtual std::vector<const Pathfinding::PathfindingPortal*> getPortalsInZone( const PathfindingMath::AxisAlignedBox & in_box, int in_hLevel );


   /** 
     * @param io_gameObjectMgr gameObjectManager to be set
   */
   void setGameObjectMgr( CoreGameObjectManager* io_gameObjectMgr );

   /** 
     * This function suppress and delete all the sectors and portals of the partition.
     * @param[in] in_ID id of the partition to be cleared
   */
   void clearPartition( const Ogre::String in_ID, int in_hLevel );

   /** 
     * @param[in] in_ID id of the partition
     * @return list of the pathfindingSector at the partition
   */
   const std::list<Pathfinding::PathfindingSector*>* getSectorsFromPartition( const Ogre::String in_ID, int in_hLevel );

   /** 
     * @param[in] in_ID id of the partition
     * @return list of the pathfindingSector at the partition
   */
   const std::list<Pathfinding::PathfindingPortal*>* getPortalsFromPartition( const Ogre::String in_ID, int in_hLevel );


   private:
   /**
    * @typedef a map of list of the sectors
   */
   typedef std::map< Ogre::String, std::list<Pathfinding::PathfindingSector*>> tdSectorMap;

   /**
    * @typedef a map of list of the portals
   */
   typedef std::map< Ogre::String, std::list<Pathfinding::PathfindingPortal*>> tdPortalMap;

   /**
    * All the data about the sectors are stored into mSectorMap
   */
   std::map<int, tdSectorMap> mSectorMap;

   /**
    * All the data about the portals are stored into mSectorMap
   */
   std::map<int, tdPortalMap> mPortalMap;

   /**
    * The game object manager to access partitions.
   */
   CoreGameObjectManager* mGameObjectMgr;

   };

} // namespace GOOF
#endif
