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

#include "GOOFPathfindingDisplay.h"
#include "PathfindingAStar.h"
#include "PathfindingPathLinear.h"
#include "PathfindingPathCardinalSpline.h"
#include "PathfindingPathBezier.h"
#include "PathfindingActor.h"
#include "GOOFCorePartitionManager.h"
#include "GOOFCorePartition.h"
#include "Utilities/Math/OgreAxisAlignedBoxAdapter.h"

template<> GOOF::PathfindingDisplay* Ogre::Singleton<GOOF::PathfindingDisplay>::ms_Singleton = 0;
namespace GOOF 
{

PathfindingDisplay::PathfindingDisplay()
{
   OGRE_LOCK_AUTO_MUTEX
   /**
    Sectors
   */
   mMatSector = MaterialManager::getSingleton().create("MatPathSector","debugger");

   mMatSector->setReceiveShadows(false); 
   mMatSector->getTechnique(0)->setLightingEnabled(true); 
   // Sectors are in red
   mMatSector->getTechnique(0)->getPass(0)->setDiffuse(1,0,0,0); 
   mMatSector->getTechnique(0)->getPass(0)->setAmbient(1,0,0); 
   mMatSector->getTechnique(0)->getPass(0)->setSelfIllumination(1,0,0); 

   /**
    Portals
   */
   mMatPortal = MaterialManager::getSingleton().create("MatPathPortal","debugger"); 
   mMatPortal->setReceiveShadows(false); 
   mMatPortal->getTechnique(0)->setLightingEnabled(true); 
   // Portals are in blue
   mMatPortal->getTechnique(0)->getPass(0)->setDiffuse(0,0,1,0); 
   mMatPortal->getTechnique(0)->getPass(0)->setAmbient(0,0,1); 
   mMatPortal->getTechnique(0)->getPass(0)->setSelfIllumination(0,0,1);

   /**
    Linear Path
   */
   mMatLinear = MaterialManager::getSingleton().create("MatLinear","debugger"); 
   mMatLinear->setReceiveShadows(false); 
   mMatLinear->getTechnique(0)->setLightingEnabled(true); 
   // Path is yellow
   mMatLinear->getTechnique(0)->getPass(0)->setDiffuse(1,1,0,0); 
   mMatLinear->getTechnique(0)->getPass(0)->setAmbient(1,1,0); 
   mMatLinear->getTechnique(0)->getPass(0)->setSelfIllumination(1,1,0); 

   /**
    Bezier Path
   */
   mMatBezier = MaterialManager::getSingleton().create("MatBezier","debugger"); 
   mMatBezier->setReceiveShadows(false); 
   mMatBezier->getTechnique(0)->setLightingEnabled(true); 
   // Path is green
   mMatBezier->getTechnique(0)->getPass(0)->setDiffuse(0,1,0,0); 
   mMatBezier->getTechnique(0)->getPass(0)->setAmbient(0,1,0); 
   mMatBezier->getTechnique(0)->getPass(0)->setSelfIllumination(0,1,0); 

   /**
    Cardinal Spline Path
   */
   mMatCardinalSpline = MaterialManager::getSingleton().create("MatCardinalSpline","debugger"); 
   mMatCardinalSpline->setReceiveShadows(false); 
   mMatCardinalSpline->getTechnique(0)->setLightingEnabled(true); 
   // Path is pink
   mMatCardinalSpline->getTechnique(0)->getPass(0)->setDiffuse(0,1,1,0); 
   mMatCardinalSpline->getTechnique(0)->getPass(0)->setAmbient(0,1,1);
   mMatCardinalSpline->getTechnique(0)->getPass(0)->setSelfIllumination(0,1,1); 

   mLineNumber = 0;

   mNodePortal = NULL;
   mNodeSector = NULL;
}

PathfindingDisplay::~PathfindingDisplay()
{
   if( ms_Singleton )
   {
      delete ms_Singleton;
   }
}


void PathfindingDisplay::showAllSectorsAndPortals( PathfindingData* in_data, int in_hLevel )
{
   CorePartitionManager::PartitionMap partMap = mGameObjectMgr->getPartitionManager()->getPartitionMap();
   for( CorePartitionManager::PartitionMap::iterator iter = partMap.begin(); iter != partMap.end(); iter++ )
   {
      const std::list<Pathfinding::PathfindingSector*>* listSectors = in_data->getSectorsFromPartition( iter->second->getID(), in_hLevel );
      showBox( iter->second->getID(), "Sector", listSectors, mNodeSector, "MatPathSector" );
      const std::list<Pathfinding::PathfindingPortal*>* listPortals = in_data->getPortalsFromPartition( iter->second->getID(), in_hLevel );
      showBox( iter->second->getID(), "Portal", listPortals, mNodePortal, "MatPathPortal" );
   }
}

void PathfindingDisplay::showSectors( const Ogre::String in_ID, PathfindingData* in_data, int in_hLevel )
{
   const std::list<Pathfinding::PathfindingSector*>* listSectors = in_data->getSectorsFromPartition( in_ID, in_hLevel );
   showBox( in_ID, "Sector", listSectors, mNodeSector, "MatPathSector" );
}

void PathfindingDisplay::showPortals( const Ogre::String in_ID, PathfindingData* in_data, int in_hLevel )
{
   const std::list<Pathfinding::PathfindingPortal*>* listPortals = in_data->getPortalsFromPartition( in_ID, in_hLevel );
   showBox( in_ID, "Portal", listPortals, mNodePortal, "MatPathPortal" );
}

void PathfindingDisplay::showBezier( std::vector<Pathfinding::PathfindingAStarNode> * lWaypoints, const Pathfinding::PathfindingActor* in_actor, int in_nbPoints, Ogre::SceneNode* io_node, int in_hLevel )
{
   Pathfinding::PathfindingPathBezier pathBezier;

   showLine( lWaypoints, in_actor, in_nbPoints, &pathBezier, "MatBezier", io_node, in_hLevel );
}

void PathfindingDisplay::showStraightLine( std::vector<Pathfinding::PathfindingAStarNode> * lWaypoints, const Pathfinding::PathfindingActor* in_actor, int in_nbPoints, Ogre::SceneNode* io_node, int in_hLevel )
{
   Pathfinding::PathfindingPathLinear pathLinear;

   //pathLinear.reCalculatePoints( lWaypoints, in_actor );

   showLine( lWaypoints, in_actor, in_nbPoints, &pathLinear, "MatLinear", io_node, in_hLevel );
}


void PathfindingDisplay::showCardinalSpline( std::vector<Pathfinding::PathfindingAStarNode> * lWaypoints, const Pathfinding::PathfindingActor* in_actor, int in_nbPoints, Ogre::SceneNode* io_node, int in_hLevel )
{
   Pathfinding::PathfindingPathCardinalSpline pathCardinal;

   showLine( lWaypoints, in_actor, in_nbPoints, &pathCardinal, "MatCardinalSpline", io_node, in_hLevel );
}

PathfindingDisplay& PathfindingDisplay::getSingleton(void)
{  
   if( !ms_Singleton )
   {
      // The singleton has not been created, create it
      new PathfindingDisplay();
   }
   assert( ms_Singleton );  return ( *ms_Singleton );  
}



/**
The function showBox is used to displayed a list of boxes in a sector.
   @param
       in_ID The ID of the zone in which boxes need to be displayed
   @param
      in_type The name of the type of boxes that need to be shown i.e. ( "Sector" or "Portal" )
   @param
      in_list The list containing boxes to be shown.  The parameter T is PathfindingPortal or PathfindingSector
   @param
      io_node The node in which the boxes belong
   @param
      in_material The name of the material used by the boxes to be displayed
*/
template<class T>
void PathfindingDisplay::showBox( const Ogre::String in_ID, const Ogre::String in_type, const std::list<T*> * in_list, SceneNode* io_node, const Ogre::String in_material )
{
   OGRE_LOCK_AUTO_MUTEX
   int number = 0;
   char cNumber[256];
   Ogre::String ostrNumber;

   // Check if this sector already exist
   Ogre::SceneNode::ChildNodeIterator childNodeIter = io_node->getChildIterator();
   Ogre::SceneNode* pChildNode = NULL;

   // Find if the sector is already displayed
   while( childNodeIter.hasMoreElements() && !pChildNode )
   {
      Ogre::Node* node = childNodeIter.getNext();
      if( node->getName() == in_type + in_ID )
      {
         pChildNode = static_cast<Ogre::SceneNode*>( node );
      }
   }

   if( pChildNode )
   {
      // The sector already exist, destroy all manual objects so only new portals or sectors are shown.
      destroyNode( pChildNode );
   }
   else
   {
      // It dosen't exist, so create a new node
      pChildNode = io_node->createChildSceneNode( in_type + in_ID );
   }

   for( std::list<T*>::const_iterator iter = in_list->begin(); iter != in_list->end(); iter++ )
   {
      sprintf(cNumber,"%d",number);
      ostrNumber = cNumber;

      Ogre::ManualObject* myManualObject =  mGameObjectMgr->getSceneManager()->createManualObject("manual" + in_type + in_ID + ostrNumber); 

      myManualObject->begin(in_material, Ogre::RenderOperation::OT_LINE_STRIP); 
      myManualObject->position((*iter)->getBox().getMinimum().x, (*iter)->getBox().getMaximum().y + 1.f, (*iter)->getBox().getMinimum().z ); 
      myManualObject->position((*iter)->getBox().getMinimum().x, (*iter)->getBox().getMaximum().y + 1.f, (*iter)->getBox().getMaximum().z ); 
      myManualObject->position((*iter)->getBox().getMaximum().x, (*iter)->getBox().getMaximum().y + 1.f, (*iter)->getBox().getMaximum().z ); 
      myManualObject->position((*iter)->getBox().getMaximum().x, (*iter)->getBox().getMaximum().y + 1.f, (*iter)->getBox().getMinimum().z );
      myManualObject->position((*iter)->getBox().getMinimum().x, (*iter)->getBox().getMaximum().y + 1.f, (*iter)->getBox().getMinimum().z );

      myManualObject->end(); 

      myManualObject->setBoundingBox( OgreAxisAlignedBoxAdapter( (*iter)->getBox() ) );
      pChildNode->attachObject(myManualObject);

      number++;
   }
}



void PathfindingDisplay::setGameObjectMgr( CoreGameObjectManager* gameObjectMgr )
{
   OGRE_LOCK_AUTO_MUTEX
   mGameObjectMgr = gameObjectMgr;
   if( !mNodeSector && !mNodePortal )
   {
      mNodeSector = mGameObjectMgr->getSceneManager()->getRootSceneNode()->createChildSceneNode( "SectorPathfinding" );
      mNodePortal = mGameObjectMgr->getSceneManager()->getRootSceneNode()->createChildSceneNode( "PortalPathfinding" );
   }
   
}



void PathfindingDisplay::showLine( std::vector<Pathfinding::PathfindingAStarNode> * lWaypoints, const Pathfinding::PathfindingActor* in_actor, int in_nbPoints, Pathfinding::PathfindingPath* in_path, const Ogre::String in_material, Ogre::SceneNode* in_node, int in_hLevel )
{
   OGRE_LOCK_AUTO_MUTEX
   if( lWaypoints->size() > 1 )
   {
      char cNumber[256];
      Ogre::String ostrNumber;

      // Path
      sprintf(cNumber,"%d",mLineNumber);
      ostrNumber = cNumber;

      Pathfinding::InterpolatePathData dataPath;
      dataPath.waypoints = lWaypoints;
      dataPath.nbPoints  = in_nbPoints;
      dataPath.data      = in_actor->getData();
      dataPath.hLevel    = in_hLevel;

      std::vector<PathfindingMath::Vector3> lPoints = in_path->interpolateFromWaypoints( &dataPath );

      if( !in_node )
      {
         in_node = mGameObjectMgr->getSceneManager()->getRootSceneNode()->createChildSceneNode("manual_node" + ostrNumber); 
      }

      Ogre::ManualObject* myManualObject =  mGameObjectMgr->getSceneManager()->createManualObject("manual" + ostrNumber); 

      myManualObject->begin(in_material, Ogre::RenderOperation::OT_LINE_STRIP); 

      PathfindingMath::AxisAlignedBox box;

      std::vector<PathfindingMath::Vector3>::iterator iter = lPoints.begin();

      Ogre::Vector3 vecStart = OgreVector3Adapter( *iter );

      float fRand = Ogre::Math::RangeRandom( 1.f, 5.f );
      for( ;iter != lPoints.end(); iter++ )
      {
         myManualObject->position( Ogre::Vector3( iter->x, iter->y + fRand, iter->z ) );
      }

      iter--;
      Ogre::Vector3 vecEnd = OgreVector3Adapter( *iter );

      myManualObject->end();

      Ogre::Vector3 vecMin = vecStart;
      vecMin.makeFloor( vecEnd );
      Ogre::Vector3 vecMax = vecStart;
      vecMax.makeCeil( vecEnd );

      myManualObject->setBoundingBox( Ogre::AxisAlignedBox( vecMin, vecMax ) );

      in_node->attachObject(myManualObject);

      mLineNumber++;
   }

}

/**
 Destroy the node and the manual objects contained into
*/
void PathfindingDisplay::destroyNode( Ogre::SceneNode* in_node )
{
   OGRE_LOCK_AUTO_MUTEX
   if( in_node )
   {
      Ogre::SceneNode::ObjectIterator objIter = in_node->getAttachedObjectIterator();
      std::list< Ogre::MovableObject* > listToDelete;
      while( objIter.hasMoreElements() )
      {
         listToDelete.push_back( objIter.getNext() );
      }
      in_node->detachAllObjects();
      for( std::list< Ogre::MovableObject* >::iterator iter = listToDelete.begin(); iter != listToDelete.end(); iter++ )
      {
         mGameObjectMgr->getSceneManager()->destroyManualObject( static_cast<Ogre::ManualObject*>( *iter ) );
      }
   }
}

void PathfindingDisplay::addPortalsToShow( const Ogre::String & in_ID )
{
   mIDPortalsToProcess.push_back( in_ID );
}

void PathfindingDisplay::addSectorsToShow( const Ogre::String & in_ID )
{
   mIDSectorsToProcess.push_back( in_ID );
}

void PathfindingDisplay::processToShow( PathfindingData* in_data, int in_hLevel )
{
   for( std::list<Ogre::String>::iterator iterString = mIDSectorsToProcess.begin(); iterString != mIDSectorsToProcess.end(); iterString++ )
   {
      const std::list<Pathfinding::PathfindingSector*>* listSectors = in_data->getSectorsFromPartition( *iterString, in_hLevel );
      showBox( *iterString, "Sector", listSectors, mNodeSector, "MatPathSector" );
   }
   mIDSectorsToProcess.clear();

   for( std::list<Ogre::String>::iterator iterString = mIDPortalsToProcess.begin(); iterString != mIDPortalsToProcess.end(); iterString++ )
   {
      const std::list<Pathfinding::PathfindingPortal*>* listPortals = in_data->getPortalsFromPartition( *iterString, in_hLevel );
      showBox( *iterString, "Portal", listPortals, mNodePortal, "MatPathPortal" );
   }
   mIDPortalsToProcess.clear();

}

} // namespace GOOF