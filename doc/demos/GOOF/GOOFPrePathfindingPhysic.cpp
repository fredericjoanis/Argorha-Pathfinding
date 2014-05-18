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

#include "GOOFPrePathfindingPhysic.h"
#include "GOOFCoreGameObject.h"
#include "GOOFCorePartition.h"
#include "GOOFCorePartitionManager.h"
#include "GOOFPartitionManager.h"
#include "GOOFCollisionGameSystem.h"
#include "Utilities/Math/OgreVector3Adapter.h"
#include "Utilities/Math/OgreAxisAlignedBoxAdapter.h"

namespace GOOF 
{

   PrePathfindingPhysic::~PrePathfindingPhysic()
   {
      mGameObjectMgr->getSceneManager()->destroyQuery(mRaySceneQuery);
   }

   PrePathfindingPhysic::PrePathfindingPhysic() : Pathfinding::AbstractPrePathfindingPhysic()
   {
      mRaySceneQuery = NULL;
   }

   PrePathfindingPhysic::PrePathfindingPhysic( CoreGameObjectManager* gameObjectMgr ) : Pathfinding::AbstractPrePathfindingPhysic()
   {
      this->mGameObjectMgr = gameObjectMgr;
      mRaySceneQuery = mGameObjectMgr->getSceneManager()->createRayQuery(Ogre::Ray(Vector3::ZERO, Vector3::NEGATIVE_UNIT_Y));
      mRaySceneQuery->setQueryMask(Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);
      mRaySceneQuery->setWorldFragmentType(Ogre::SceneQuery::WFT_SINGLE_INTERSECTION);
   }

   /** Return the number of step it taken to go make the displacement to displace the actor from
   the actual actor place to the other.  This function must displace the actor.
   Return -1 if it's not possible to displace the actor from that value. */
   PathfindingMath::Real PrePathfindingPhysic::move( const PathfindingMath::Vector3 & displacement, Pathfinding::PathfindingActor* io_actor )
   {
      OGRE_LOCK_AUTO_MUTEX
      PathfindingMath::Real ret = 1;
      
      PathfindingMath::Vector3 v3MaxOn2 = io_actor->getSmallVector() * 0.5f;
      PathfindingMath::AxisAlignedBox box( io_actor->getPosition() - v3MaxOn2, io_actor->getPosition() + v3MaxOn2 );

      PathfindingMath::Vector3 vec;

      if( displacement.x > 0 ) 
      {
         vec = box.getMaximum();
         vec.x += displacement.x;
         box.setMaximum( vec );
      }
      else
      {
         vec = box.getMinimum();
         vec.x += displacement.x;
         box.setMinimum( vec );
      }

      if( displacement.z > 0 )
      {
         vec = box.getMaximum();
         vec.z += displacement.z;
         box.setMaximum( vec );
      }
      else
      {
         vec = box.getMinimum();
         vec.z += displacement.z;
         box.setMinimum( vec );
      }

      for( vector<CoreGameObject*>::iterator itr = mObjects.begin(); itr != mObjects.end() && ret == 1; itr++ )
      {
         Ogre::AxisAlignedBox worldBox;
         worldBox.setMinimum( (*itr)->getWorldAABB().getMinimum() + (*itr)->getWorldPosition() );
         worldBox.setMaximum( (*itr)->getWorldAABB().getMaximum() + (*itr)->getWorldPosition() );

         if( worldBox.intersects( OgreAxisAlignedBoxAdapter( box ) ) ) 
         {
            ret = -1;
         }
      }

      if( ret != -1 )
      {
         vec = io_actor->getPosition();
         vec.x += displacement.x;
         vec.z += displacement.z;

         // Taken from the Ogre terrain demo
         static Ray updateRay;

         if( mRaySceneQuery == NULL )
         {
            mRaySceneQuery = mGameObjectMgr->getSceneManager()->createRayQuery( Ogre::Ray( OgreVector3Adapter( io_actor->getPosition() ), Vector3::NEGATIVE_UNIT_Y));
            mRaySceneQuery->setQueryMask(Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);
            mRaySceneQuery->setWorldFragmentType(Ogre::SceneQuery::WFT_SINGLE_INTERSECTION);
         }

         mRaySceneQuery->setRay(updateRay);
         RaySceneQueryResult& qryResult = mRaySceneQuery->execute();
         RaySceneQueryResult::iterator i = qryResult.begin();
         if (i != qryResult.end() && i->worldFragment)
         {
            SceneQuery::WorldFragment* wf = i->worldFragment;
            io_actor->setPosition(PathfindingMath::Vector3(vec.x, i->worldFragment->singleIntersection.y, vec.z));
         }
         //End the part taken from the tutorial
      }

      return ret;
   }

   bool PrePathfindingPhysic::canStandOn( const Pathfinding::PathfindingActor* in_actor )
   {
      OGRE_LOCK_AUTO_MUTEX
      bool bRet = true;
      vector<CoreGameObject*> objects;


      for( vector<CoreGameObject*>::iterator itr = mObjects.begin(); itr != mObjects.end() && bRet; itr++ )
      {
         Ogre::AxisAlignedBox worldBox;
         worldBox.setMinimum( (*itr)->getWorldAABB().getMinimum() + (*itr)->getWorldPosition() );
         worldBox.setMaximum( (*itr)->getWorldAABB().getMaximum() + (*itr)->getWorldPosition() );

         if( worldBox.intersects( OgreVector3Adapter( in_actor->getPosition() ) ) )
         {
            bRet = false;
         }
      }
      return bRet;
   }

   void PrePathfindingPhysic::setGameObjectMgr( CoreGameObjectManager* gameObjectMgr )
   {
      mGameObjectMgr = gameObjectMgr;
   }

   std::list<PathfindingMath::Vector3> PrePathfindingPhysic::getStandOnHeights( const Pathfinding::PathfindingActor* in_actor )
   {
      OGRE_LOCK_AUTO_MUTEX
      std::list<PathfindingMath::Vector3> ret;

      static Ray updateRay;

      if( mRaySceneQuery == NULL )
      {
         mRaySceneQuery = mGameObjectMgr->getSceneManager()->createRayQuery( Ogre::Ray(Ogre::Vector3( OgreVector3Adapter( in_actor->getPosition() ).x, 5000.f, OgreVector3Adapter( in_actor->getPosition() ).z ), Vector3::NEGATIVE_UNIT_Y));
         mRaySceneQuery->setQueryMask(Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);
         mRaySceneQuery->setWorldFragmentType(Ogre::SceneQuery::WFT_SINGLE_INTERSECTION);
      }

      mRaySceneQuery->setRay(updateRay);

      RaySceneQueryResult& qryResult = mRaySceneQuery->execute();

      std::list<Ogre::AxisAlignedBox> objectsBox;
      SceneQuery::WorldFragment* wf = NULL;
      
      // Get all the objects and scene
      for ( RaySceneQueryResult::iterator iter = qryResult.begin(); iter != qryResult.end(); iter++ )
      {
         if( iter->worldFragment )
         {
            wf = iter->worldFragment;
         }
         else if( iter->movable )
         {
            MovableObject* movable = iter->movable;
            objectsBox.push_back( movable->getWorldBoundingBox() );
         }
      }

      PathfindingMath::Vector3 v3MaxOn2 = in_actor->getSmallVector() * 0.5f;

      if( wf )
      {
         PathfindingMath::AxisAlignedBox terrainBox( PathfindingMath::Vector3( in_actor->getPosition().x - v3MaxOn2.x, wf->singleIntersection.y, in_actor->getPosition().z - v3MaxOn2.z ), PathfindingMath::Vector3( in_actor->getPosition().x + v3MaxOn2.x, wf->singleIntersection.y + in_actor->getSmallVector().y, in_actor->getPosition().z + v3MaxOn2.z ) );

         bool bIntersect = false;
         for( vector<CoreGameObject*>::iterator itr = mObjects.begin(); itr != mObjects.end() && !bIntersect; itr++ )
         {
            Ogre::AxisAlignedBox worldBox( Ogre::Vector3( (*itr)->getWorldAABB().getMinimum() + (*itr)->getWorldPosition()), Ogre::Vector3((*itr)->getWorldAABB().getMaximum() + (*itr)->getWorldPosition())) ;

            if( worldBox.intersects( OgreAxisAlignedBoxAdapter( terrainBox ) ) ) 
            {
               bIntersect = true;
            }
         }
         
         if( !bIntersect )
         {
            ret.push_back( terrainBox.getCenter() );
         }
      }

      //// Go trough each object and check if we  can stand on it without intersecting an other object
      //for( std::vector<CoreGameObject*>::iterator itr = objects.begin(); itr != objects.end(); itr++ )
      //{
      //   bIntersect = false;
      //   PathfindingMath::AxisAlignedBox worldBox( PathfindingMath::Vector3( (*itr)->getWorldAABB().getMinimum() + (*itr)->getWorldPosition() ), PathfindingMath::Vector3((*itr)->getWorldAABB().getMaximum() + (*itr)->getWorldPosition())) ;
      //   //worldBox.setMinimum( (*iter)->getWorldAABB().getMinimum() + (*iter)->getWorldPosition() - (*iter)->getWorldAABB().getCenter() );
      //   //worldBox.setMaximum( (*iter)->getWorldAABB().getMaximum() + (*iter)->getWorldPosition() - (*iter)->getWorldAABB().getCenter() );

      //   std::vector<CoreGameObject*>::iterator iterAfter = itr;
      //   iterAfter++; // Start at the next element
      //   while( iterAfter != objects.end() && !bIntersect )
      //   { 
      //      PathfindingMath::AxisAlignedBox worldBoxAfter( PathfindingMath::Vector3( (*iterAfter)->getWorldAABB().getMinimum() + (*iterAfter)->getWorldPosition()), PathfindingMath::Vector3((*iterAfter)->getWorldAABB().getMaximum() + (*iterAfter)->getWorldPosition())) ;
      //      //worldBox.setMinimum( (*iterAfter)->getWorldAABB().getMinimum() + (*iterAfter)->getWorldPosition() - (*iterAfter)->getWorldAABB().getCenter() );
      //      //worldBox.setMaximum( (*iterAfter)->getWorldAABB().getMaximum() + (*iterAfter)->getWorldPosition() - (*iterAfter)->getWorldAABB().getCenter() );
      //      if( worldBox.intersects( worldBoxAfter ) )
      //      {
      //         bIntersect = true;
      //      }
      //      iterAfter++;
      //   }
      //   //if( !bIntersect )
      //   //{
      //   //   ret.push_back( worldBox.getMinimum() );
      //   //}
      //}

      return ret;
   }

   void PrePathfindingPhysic::prepare( const Pathfinding::PathfindingActor* in_actor )
   {
      OGRE_LOCK_AUTO_MUTEX
      mObjects.clear();
      mGameObjectMgr->getPartitionManager()->getPartition( OgreVector3Adapter( in_actor->getPosition() ) )->enumerateObjectsOver( mObjects );
   }



} // namespace GOOF
