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

#include "GOOFPathfindingDelegate.h"
#include "GOOFCorePartitionManager.h"
#include "GOOFGridPartitionManager.h"
#include "PathfindingSector.h"
#include "PrePathfindingCommon.h"
#include "Utilities/Math/PathfindingAxisAlignedBoxAdapter.h"

namespace GOOF 
{

   PathfindingDelegate::PathfindingDelegate( CoreGameObjectManager* io_gameObjectMgr, std::vector<Pathfinding::PathfindingActor*>* io_listActor )
   {
      mGameObjectMgr = io_gameObjectMgr;
      mpListActors = io_listActor;
      init();
   }

   PathfindingDelegate::~PathfindingDelegate()
   {
      // Delete the delegates
      // CoreGameObjectManager
      if( mStartMoveObjectsDelegate )
      {
         mGameObjectMgr->removeListenerStartMovingObjects( mStartMoveObjectsDelegate );
         delete mStartMoveObjectsDelegate;
      }

      if( mFinishMoveObjectsDelegate )
      {
         mGameObjectMgr->removeListenerFinishMovingObjects( mFinishMoveObjectsDelegate );
         delete mFinishMoveObjectsDelegate;
      }
      
      if( mRotateObjectsDelegate )
      {
         mGameObjectMgr->removeListenerRotateObjects( mRotateObjectsDelegate );
         delete mRotateObjectsDelegate;
      }

      if( mTranslateObjectsDelegate )
      {
         mGameObjectMgr->removeListenerTranslateObjects( mTranslateObjectsDelegate );
         delete mTranslateObjectsDelegate;
      }


/*
      // CorePartition
      if( mAddObjectDelegate )
      {
         CorePartitionManager::PartitionMap partMap = mGameObjectMgr->getPartitionManager()->getPartitionMap();
         for( CorePartitionManager::PartitionMap::iterator iter = partMap.begin(); iter != partMap.end(); iter++ )
         {
            iter->second->removeListenerAddObject( mAddObjectDelegate );
         }
         delete mAddObjectDelegate;
      }

      if( mRemoveObjectDelegate )
      {
         CorePartitionManager::PartitionMap partMap = mGameObjectMgr->getPartitionManager()->getPartitionMap();
         for( CorePartitionManager::PartitionMap::iterator iter = partMap.begin(); iter != partMap.end(); iter++ )
         {
            iter->second->removeListenerRemoveObject( mRemoveObjectDelegate );
         }
         delete mRemoveObjectDelegate;
      }

      // CorePartitionManager
      if( mAddPartitionDelegate )
      {
         mGameObjectMgr->getPartitionManager()->removeListenerAddPartition( mAddPartitionDelegate );
         delete mAddPartitionDelegate;
      }

      if( mRemovePartitionDelegate )
      {
         mGameObjectMgr->getPartitionManager()->removeListenerRemovePartition( mRemovePartitionDelegate );
         delete mRemovePartitionDelegate;
      }

      // CoreGameObject
      if( mRotateObjectDelegate )
      {
         CorePartitionManager::PartitionMap partMap = mGameObjectMgr->getPartitionManager()->getPartitionMap();
         for( CorePartitionManager::PartitionMap::iterator iter = partMap.begin(); iter != partMap.end(); iter++ )
         {
            vector<CoreGameObject*> vecObjects;
            iter->second->enumerateObjects( vecObjects );

            for( vector<CoreGameObject*>::iterator iterObj = vecObjects.begin(); iterObj != vecObjects.end(); iterObj++ )
            {
               (*iterObj)->removeListenerRotate( mRotateObjectDelegate );
            }
         }
         delete mRotateObjectDelegate;
      }

      if( mTranslateObjectDelegate )
      {
         CorePartitionManager::PartitionMap partMap = mGameObjectMgr->getPartitionManager()->getPartitionMap();
         for( CorePartitionManager::PartitionMap::iterator iter = partMap.begin(); iter != partMap.end(); iter++ )
         {
            vector<CoreGameObject*> vecObjects;
            iter->second->enumerateObjects( vecObjects );

            for( vector<CoreGameObject*>::iterator iterObj = vecObjects.begin(); iterObj != vecObjects.end(); iterObj++ )
            {
               (*iterObj)->removeListenerTranslate( mTranslateObjectDelegate );
            }
         }
         delete mTranslateObjectDelegate;
      }

      if( mInitFinishObjectDelegate )
      {
         CorePartitionManager::PartitionMap partMap = mGameObjectMgr->getPartitionManager()->getPartitionMap();
         for( CorePartitionManager::PartitionMap::iterator iter = partMap.begin(); iter != partMap.end(); iter++ )
         {
            vector<CoreGameObject*> vecObjects;
            iter->second->enumerateObjects( vecObjects );

            for( vector<CoreGameObject*>::iterator iterObj = vecObjects.begin(); iterObj != vecObjects.end(); iterObj++ )
            {
               (*iterObj)->removeListenerFinish( mInitFinishObjectDelegate );
            }
         }
         delete mInitFinishObjectDelegate;
      }

      if( mDestroyObjectDelegate )
      {
         CorePartitionManager::PartitionMap partMap = mGameObjectMgr->getPartitionManager()->getPartitionMap();
         for( CorePartitionManager::PartitionMap::iterator iter = partMap.begin(); iter != partMap.end(); iter++ )
         {
            vector<CoreGameObject*> vecObjects;
            iter->second->enumerateObjects( vecObjects );

            for( vector<CoreGameObject*>::iterator iterObj = vecObjects.begin(); iterObj != vecObjects.end(); iterObj++ )
            {
               (*iterObj)->removeListenerDestroy( mDestroyObjectDelegate );
            }
         }
         delete mDestroyObjectDelegate;
      }
      */ 
   }



   /**
      Listener for when objects start to move.
   */
   void PathfindingDelegate::startMovingObjectsListener(vector<CoreGameObject*>& in_objects)
   {
      std::set<Ogre::String> setPartition;

      for( vector<CoreGameObject*>::iterator iter = in_objects.begin(); iter != in_objects.end(); iter++ )
      {
         addPartitionsForObject( *iter );
         //setPartition.insert( mGameObjectMgr->getPartitionManager()->getPartition( (*iter)->getWorldPosition() )->getID() );
      }

      //for( std::set<Ogre::String>::iterator iter = setPartition.begin(); iter != setPartition.end(); iter++ )
      //{
      //   addPartitionsForObject( *iter );
      //}
   }

   /**
      Listener for when objects finish moving.
   */
   void PathfindingDelegate::finishMovingObjectsListener( vector<CoreGameObject*>& in_objects )
   {
      std::set<Ogre::String> setPartition;

      for( vector<CoreGameObject*>::iterator iter = in_objects.begin(); iter != in_objects.end(); iter++ )
      {
         addPartitionsForObject( *iter );
         //setPartition.insert( mGameObjectMgr->getPartitionManager()->getPartition( (*iter)->getWorldPosition() )->getID() );
      }

      //for( std::set<Ogre::String>::iterator iter = setPartition.begin(); iter != setPartition.end(); iter++ )
      //{
      //   addPartitionsForObject( *iter );
      //}
   }

   /**
      Listener when objects are rotated
   */
   void PathfindingDelegate::rotateObjectsListener( vector<CoreGameObject*>& in_objects )
   {
      /*
      std::set<Ogre::String> setPartition;

      for( vector<CoreGameObject*>::iterator iter = in_objects.begin(); iter != in_objects.end(); iter++ )
      {
         setPartition.insert( mGameObjectMgr->getPartitionManager()->getPartition( (*iter)->getWorldPosition() )->getID() );
      }

      for( std::set<Ogre::String>::iterator iter = setPartition.begin(); iter != setPartition.end(); iter++ )
      {
         addPartitionsForObject( *iter );
      }
      */
   }

   /**
      Listener when objects are translated
   */
   void PathfindingDelegate::translateObjectsListener( vector<CoreGameObject*>& in_objects )
   {
      /*
      std::set<Ogre::String> setPartition;

      for( vector<CoreGameObject*>::iterator iter = in_objects.begin(); iter != in_objects.end(); iter++ )
      {
         setPartition.insert( mGameObjectMgr->getPartitionManager()->getPartition( (*iter)->getWorldPosition() )->getID() );
      }

      for( std::set<Ogre::String>::iterator iter = setPartition.begin(); iter != setPartition.end(); iter++ )
      {
         addPartitionsForObject( *iter );
      }
      */
   }

   /**
      This function will be called when an object is added in the editor
   */
   void PathfindingDelegate::addObjectListener( CoreGameObject* in_object )
   {
      //addPartitionsForObject( in_object );

      in_object->addListenerFinish( mInitFinishObjectDelegate );
      in_object->addListenerRotate( mRotateObjectDelegate );
      in_object->addListenerDestroy( mDestroyObjectDelegate );
      in_object->addListenerTranslate( mTranslateObjectDelegate );
   }

   /**
      This function will be called when an object is removed in the editor
   */
   void PathfindingDelegate::removeObjectListener( CoreGameObject* in_object )
   {
      //addPartitionsForObject( mGameObjectMgr->getPartitionManager()->getPartition( in_object->getWorldPosition() )->getID() );
   }

   /**
      This function will be called when an partition is added in the editor
   */
   void PathfindingDelegate::addPartitionListener( CorePartition* in_partition )
   {
      // We need to create this 
      in_partition->addListenerAddObject(mAddObjectDelegate);
      in_partition->addListenerRemoveObject(mRemoveObjectDelegate);

      GOOF::GridPartitionManager *part = dynamic_cast<GridPartitionManager*>( mGameObjectMgr->getPartitionManager() );

      PathfindingMath::Vector3 v3PartitionPos = PathfindingVector3Adapter( in_partition->getWorldPosition() );

      float fGridCellSize = part->getGridCellSize() * 0.5f;
      PathfindingMath::AxisAlignedBox zoneBox( PathfindingMath::Vector3( v3PartitionPos.x - fGridCellSize, v3PartitionPos.y - 10.f, v3PartitionPos.z - fGridCellSize ) , PathfindingMath::Vector3(v3PartitionPos.x + fGridCellSize, v3PartitionPos.y + fGridCellSize, v3PartitionPos.z + fGridCellSize ));

      // Create a sector with the size of the partition
      Pathfinding::PathfindingSector* sec = new Pathfinding::PathfindingSector();
      sec->setBox( zoneBox );
      sec->setCost( 1 );

      for( std::vector<Pathfinding::PathfindingActor*>::iterator iter = mpListActors->begin(); iter != mpListActors->end(); iter++ )
      {
         // Add the sector in the list of each actors
         static_cast<PathfindingData*>( (*iter)->getData())->addSector( sec, Pathfinding::PrePathfindingCommon::BASE_LEVEL );
         // Kind of a hack.
         //PathfindingThread::getSingleton().getCompute().generatePortalsAroundZone( PathfindingThread::getSingleton().getPhysic(), (*iter), &sec->getBox() );
      }
   }

   /**
      This function will be called when an partition is removed in the editor
   */
   void PathfindingDelegate::removePartitionListener( CorePartition* in_partition )
   {
      in_partition->removeListenerAddObject(mAddObjectDelegate);
      in_partition->removeListenerRemoveObject(mRemoveObjectDelegate);
   }

   void PathfindingDelegate::destroyObjectListener( CoreGameObject* in_object )
   {
      addPartitionsForObject( in_object );
   }

   void PathfindingDelegate::translateObjectListener( CoreGameObject* in_object )
   {
      addPartitionsForObject( in_object );
   }

   void PathfindingDelegate::initFinishObjectListener( CoreGameObject* in_object )
   {
      addPartitionsForObject( in_object );
   }

   void PathfindingDelegate::rotateObjectListener( CoreGameObject* in_object )
   {
      addPartitionsForObject( in_object );
   }


   void PathfindingDelegate::setGameObjectMgr( CoreGameObjectManager* gameObjectMgr )
   {
      mGameObjectMgr = gameObjectMgr;
   }

   void PathfindingDelegate::setActorArray( std::vector<Pathfinding::PathfindingActor*>* io_listActor )
   {
      mpListActors = io_listActor;
   }



   void PathfindingDelegate::init()
   {
      // Create the delegates
      // CoreGameObjectManager
      mStartMoveObjectsDelegate  = new CoreGameObjectManager::tdDelegateObjects(&(*this), &PathfindingDelegate::startMovingObjectsListener);
      mFinishMoveObjectsDelegate = new CoreGameObjectManager::tdDelegateObjects(&(*this), &PathfindingDelegate::finishMovingObjectsListener);
      mRotateObjectsDelegate     = new CoreGameObjectManager::tdDelegateObjects(&(*this), &PathfindingDelegate::rotateObjectsListener);
      mTranslateObjectsDelegate  = new CoreGameObjectManager::tdDelegateObjects(&(*this), &PathfindingDelegate::translateObjectsListener);

      // CorePartition
      mAddObjectDelegate         = new CorePartition::tdDelegateObject(&(*this), &PathfindingDelegate::addObjectListener);
      mRemoveObjectDelegate      = new CorePartition::tdDelegateObject(&(*this), &PathfindingDelegate::removeObjectListener);

      // CorePartitionManager
      mAddPartitionDelegate      = new CorePartitionManager::tdDelegatePartition(&(*this), &PathfindingDelegate::addPartitionListener);
      mRemovePartitionDelegate   = new CorePartitionManager::tdDelegatePartition(&(*this), &PathfindingDelegate::removePartitionListener);

      // CoreGameObject
      mRotateObjectDelegate      = new CoreGameObject::tdDelegateObject( &(*this), &PathfindingDelegate::rotateObjectListener );
      mTranslateObjectDelegate   = new CoreGameObject::tdDelegateObject( &(*this), &PathfindingDelegate::translateObjectListener );
      mInitFinishObjectDelegate  = new CoreGameObject::tdDelegateObject( &(*this), &PathfindingDelegate::initFinishObjectListener );
      mDestroyObjectDelegate     = new CoreGameObject::tdDelegateObject( &(*this), &PathfindingDelegate::destroyObjectListener );


      // Register the delegates
      // CoreGameObjectManager
      mGameObjectMgr->addListenerStartMovingObjects( mStartMoveObjectsDelegate );
      mGameObjectMgr->addListenerFinishMovingObjects( mFinishMoveObjectsDelegate );
      mGameObjectMgr->addListenerRotateObjects( mRotateObjectsDelegate );
      mGameObjectMgr->addListenerTranslateObjects( mTranslateObjectsDelegate );

      // CorePartition
      CorePartitionManager::PartitionMap partMap = mGameObjectMgr->getPartitionManager()->getPartitionMap();
      for( CorePartitionManager::PartitionMap::iterator iter = partMap.begin(); iter != partMap.end(); iter++ )
      {
         iter->second->addListenerAddObject( mAddObjectDelegate );
         iter->second->addListenerRemoveObject( mRemoveObjectDelegate );

         vector<CoreGameObject*> vecObjects;
         iter->second->enumerateObjects( vecObjects );
         for( vector<CoreGameObject*>::iterator iterObj = vecObjects.begin(); iterObj != vecObjects.end(); iterObj++ )
         {
            (*iterObj)->addListenerRotate( mRotateObjectDelegate );
            (*iterObj)->addListenerDestroy( mDestroyObjectDelegate );
            (*iterObj)->addListenerTranslate( mTranslateObjectDelegate );
         }
      }

      // CorePartitionManager
      mGameObjectMgr->getPartitionManager()->addListenerAddPartition( mAddPartitionDelegate );
      mGameObjectMgr->getPartitionManager()->addListenerRemovePartition( mRemovePartitionDelegate );
   }

   void PathfindingDelegate::addPartitionsForObject( CoreGameObject* in_object )
   {
      std::set<Ogre::String> setPartition;

      setPartition.insert( in_object->getPartitionID() );

      Ogre::AxisAlignedBox objectBox( -in_object->getWorldAABB().getCenter() + in_object->getWorldAABB().getMinimum() + in_object->getWorldPosition(), -in_object->getWorldAABB().getCenter() + in_object->getWorldAABB().getMaximum() + in_object->getWorldPosition() );

      if( !in_object->getPartition()->getWorldPartitionBox().contains( objectBox ) )
      {
         std::vector<CorePartition*> surroundingPartitions;
         in_object->getPartition()->enumerateConnectedPartitions( surroundingPartitions );

         for( std::vector<CorePartition*>::iterator iterPart = surroundingPartitions.begin(); iterPart != surroundingPartitions.end(); iterPart++ )
         {
            if( objectBox.intersects( (*iterPart)->getWorldPartitionBox() ) )
            {
                setPartition.insert( (*iterPart)->getID() );
            }
         }
      }

      for( std::set<Ogre::String>::iterator iter = setPartition.begin(); iter != setPartition.end(); iter++ )
      {
         PathfindingThread::getSingleton().addPartitionToQueue( *iter );
      }

   }



};