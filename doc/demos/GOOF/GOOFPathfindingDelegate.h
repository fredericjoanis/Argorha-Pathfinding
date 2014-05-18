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

#ifndef _GOOFPATHFINDINGDELEGATE_H
#define _GOOFPATHFINDINGDELEGATE_H

#include "GOOFPathfindingGameSystemPrerequisites.h"
#include "GOOFCoreGameObjectManager.h"
#include "GOOFGameObjectManagerGameSystem.h"
#include "GOOFCorePartition.h"
#include "GOOFPathfindingThread.h"

namespace GOOF 
{

/** @class PathfindingDelegate GOOFPathfindingDelegate.h "include/GOOFPathfindingDelegate.h"
*
* This class is used to intercept the actions made by the user in the Switch Blade editor.
*/
class _GOOFPathfindingGameSystemExport PathfindingDelegate
{
public:

   /**
      @param io_gameObjectMgr gameObjectManager to be set
      @param io_listActor list of actors to be set
   */
   PathfindingDelegate(CoreGameObjectManager* io_gameObjectMgr, std::vector<Pathfinding::PathfindingActor*>* io_listActor );
   virtual ~PathfindingDelegate();

   /**
      Listener for when objects start to move.
      @param[in] in_objects the object that start moving
   */
   void startMovingObjectsListener(vector<CoreGameObject*>& in_objects);

   /**
      Listener for when objects finish moving.
      @param[in] in_objects the object that finish moving
   */
   void finishMovingObjectsListener(vector<CoreGameObject*>& in_objects);

   /**
      Listener when objects are rotated
      @param[in] in_objects the object that are being rotated
   */
   void rotateObjectsListener(vector<CoreGameObject*>& in_objects);

   
   /**
      Listener when an object is rotated
      @param[in] in_object the object that is being rotated
   */
   void rotateObjectListener(CoreGameObject* in_object);

   /**
      Listener when objects are translated
      @param[in] in_objects the objects that is are translated
   */
   void translateObjectsListener(vector<CoreGameObject*>& in_objects);

  /**
      Listener when an object is translated
      @param[in] in_object the object that is being translate
   */
   void translateObjectListener(CoreGameObject* in_object);

  /**
      Listener when an object is finish to be created
      @param[in] in_object the object that has been created
   */
   void initFinishObjectListener(CoreGameObject* in_object);


   /**
      This function will be called when an object is added in the editor
      @param[in] in_object the object that is being add
   */
   void addObjectListener(CoreGameObject* in_object);

   /**
      This function will be called when an object is removed in the editor
      @param[in] in_object the object that is being add
   */
   void removeObjectListener(CoreGameObject* in_object);

  /**
      Listener when an object is destroyed
      @param[in] in_object the object that is being destroyed
   */
   void destroyObjectListener(CoreGameObject* in_object);

   /**
      This function will be called when an partition is added in the editor
      @param[in] partition the partition that is being created
   */
   void addPartitionListener(CorePartition* in_partition);

   /**
      This function will be called when an partition is removed in the editor
      @param[in] partition the partition that is being removed
   */
   void removePartitionListener(CorePartition* in_partition);

   /** 
      @param io_gameObjectMgr gameObjectManager to be set
   */
   void setGameObjectMgr( CoreGameObjectManager* io_gameObjectMgr );

   /** 
      @param io_listActor list of actors to be set
   */
   void setActorArray( std::vector<Pathfinding::PathfindingActor*>* io_listActor );

   /**
    * Take an object and add it to the partition that has been dirty. 
   */
   void addPartitionsForObject( CoreGameObject* in_object );


private:
   
   CoreGameObjectManager::tdDelegateObjects*    mStartMoveObjectsDelegate;    /// Delegate for the callback
   CoreGameObjectManager::tdDelegateObjects*    mFinishMoveObjectsDelegate;   /// Delegate for the callback
   CoreGameObjectManager::tdDelegateObjects*    mRotateObjectsDelegate;       /// Delegate for the callback
   CoreGameObjectManager::tdDelegateObjects*    mTranslateObjectsDelegate;    /// Delegate for the callback

   CorePartition::tdDelegateObject*             mAddObjectDelegate;           /// Delegate for the callback
   CorePartition::tdDelegateObject*             mRemoveObjectDelegate;        /// Delegate for the callback

   CoreGameObject::tdDelegateObject*            mRotateObjectDelegate;        /// Delegate for the callback
   CoreGameObject::tdDelegateObject*            mTranslateObjectDelegate;     /// Delegate for the callback
   CoreGameObject::tdDelegateObject*            mInitFinishObjectDelegate;    /// Delegate for the callback
   CorePartition::tdDelegateObject*             mDestroyObjectDelegate;       /// Delegate for the callback
   

   CorePartitionManager::tdDelegatePartition*   mAddPartitionDelegate;        /// Delegate for the callback
   CorePartitionManager::tdDelegatePartition*   mRemovePartitionDelegate;     /// Delegate for the callback
      
   /**
      The game object manager to access partitions.
   */
   CoreGameObjectManager*                       mGameObjectMgr;

   /**
    The list of actors
   */
   std::vector<Pathfinding::PathfindingActor*>* mpListActors;

   /**
    All the initial data that needs to be set at start
   */
   void init();
};


} // namespace GOOF



#endif
