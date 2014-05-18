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

#include "GOOFPathfindingGameSystem.h"
#include "PathfindingAStar.h"
#include "GOOFGridPartitionManager.h"
#include "GOOFCorePartition.h"
#include "PathfindingPathBezier.h"
#include "PathfindingPathLinear.h"
#include "PathfindingPathCardinalSpline.h"
#include "OgreString.h"
#include "PrePathfindingCommon.h"
#include "Utilities/Math/PathfindingVector3Adapter.h"

namespace GOOF {




//Return true if the data has been generated.
bool PathfindingGameSystem::generatePrePathfinding(const PathfindingMath::AxisAlignedBox & sector) 
{
	return true;
}


/** 
*/
PathfindingGameSystem::PathfindingGameSystem() : GameObjectManagerGameSystem()
{
   mNode = NULL;

	this->mXDivideFactor = 2;  // Default dividing factor
	this->mZDivideFactor = 2;  // Default dividing factor

   // Set the actor, bad programming!! Should be an game object that is passed here ... Testing purpose only
   PathfindingMath::Vector3 minVec( 0,0,0 );
   PathfindingMath::Vector3 maxVec( 5.f,2.f,5.f );

   PathfindingMath::AxisAlignedBox box( minVec, maxVec );

   // The data needs to be created
   Pathfinding::PathfindingData * data = new GOOF::PathfindingData();
   Pathfinding::PathfindingActor * actor = new Pathfinding::PathfindingActor( data, box );

   mActors.push_back( actor );
   mDefaultActor = actor;

   mStartPos = Ogre::Vector3::ZERO;
   mEndPos = Ogre::Vector3::ZERO;

   mbActivate = true;

   mDelegate = NULL;

   mLvlStart = 1;
   mLvlEnd = 0; 
   mSPLevel = 0;
}


/** 
*/
PathfindingGameSystem::~PathfindingGameSystem()
{
   for( std::vector<Pathfinding::PathfindingActor*>::iterator iter = mActors.begin(); iter != mActors.end(); iter++  )
   {
      delete (*iter);
   }
   if( mDelegate )
   {
      delete mDelegate;
   }
}

/** frame started event, pass it down to the Pathfinding manager
*/
void PathfindingGameSystem::update(float deltaTime)
{
   PathfindingDisplay::getSingleton().processToShow( static_cast<GOOF::PathfindingData*>( mDefaultActor->getData() ), mSPLevel );
}

//-------------------------------------------------------------------
void PathfindingGameSystem::init(GameObjectManager* gameObjectMgr)
{
	mGameObjectMgr = static_cast<CoreGameObjectManager*>(gameObjectMgr);

   for( std::vector<Pathfinding::PathfindingActor*>::iterator iter = mActors.begin(); iter != mActors.end(); iter++ )
   {
      static_cast<PathfindingData*>( (*iter)->getData() )->setGameObjectMgr( mGameObjectMgr );
   }

   PathfindingThread::getSingleton().setGameObjectMgr( mGameObjectMgr );
   PathfindingThread::getSingleton().setActorArray( &mActors );

   mDelegate = new GOOF::PathfindingDelegate(mGameObjectMgr, &mActors);
}

/** Saves values to the element provided
*/
bool PathfindingGameSystem::save(DataElementPtr element)
{
	// TODO: Make code
   // Need to wait the SwitchBlade to make the save in the new format.
	return true;
}

/** Loads values from the element provided
*/
bool PathfindingGameSystem::load(DataElementPtr element)
{
	// TODO: Make code
   // Need to wait the SwitchBlade to make the save in the new format.
	return true;
}


//-----------------------------------------------------------------------
void PathfindingGameSystem::enumerateProperties(vector<Property>& properties)
{
	Property prop;

	prop.initFloat("PathfindingGameSystem_DivideXFactor", "X Divide factor", mXDivideFactor, 1, 1000, 0.1);
	properties.push_back(prop);

   prop.initFloat("PathfindingGameSystem_DivideZFactor", "Z Divide factor", mZDivideFactor, 1, 1000, 0.1);
	properties.push_back(prop);

   prop.initVector("PathfindingGameSystem_Start", "Start position", mStartPos );
   properties.push_back(prop);

   prop.initVector("PathfindingGameSystem_End", "End position", mEndPos );
   properties.push_back(prop);

   prop.initInt("PathfindingGameSystem_ShowSectorsAndPortalsLevel", "Show S&P Level", mSPLevel );
   properties.push_back(prop);

   prop.initInt("PathfindingGameSystem_ShowPathfindingLevelStart", "Pathfinding Level Start", mLvlStart );
   properties.push_back(prop);

   prop.initInt("PathfindingGameSystem_ShowPathfindingLevelEnd", "Pathfinding Level End", mLvlEnd );
   properties.push_back(prop);

   prop.initBool("PathfindingGameSystem_Activated", "Pathfinding Activated", mbActivate );
   properties.push_back(prop);

}

//-----------------------------------------------------------------------
void PathfindingGameSystem::setProperty(const String& id, const PropertyData& data)
{
	if(id == "PathfindingGameSystem_DivideXFactor")
   {
		setDivideXFactor(data.floatVal);
   }
	else if(id == "PathfindingGameSystem_DivideZFactor")
   {
		setDivideZFactor(data.floatVal);
   }
	else if(id == "PathfindingGameSystem_Start")
   {
      mStartPos = data.vectorVal;
      calculatePathfinding();
   }	
   else if(id == "PathfindingGameSystem_End")
   {
      mEndPos = data.vectorVal;
      calculatePathfinding();
   }
   else if(id == "PathfindingGameSystem_ShowSectorsAndPortalsLevel")
   {
      if( data.intVal != mSPLevel )
      {
         mSPLevel = data.intVal;
         PathfindingDisplay::getSingleton().showAllSectorsAndPortals( static_cast<GOOF::PathfindingData*>(mDefaultActor->getData()), mSPLevel );
      }
   }
   else if(id == "PathfindingGameSystem_ShowPathfindingLevelStart")
   {
      if( data.intVal >= mLvlEnd )
      {
         mLvlStart = data.intVal;
         calculatePathfinding();
      }
   }
   else if(id == "PathfindingGameSystem_ShowPathfindingLevelEnd")
   {
      if( data.intVal <= mLvlStart )
      {
         mLvlEnd = data.intVal;
         calculatePathfinding();
      }
   }
   else if(id == "PathfindingGameSystem_Activated")
   {
      mbActivate = data.boolVal;
      // Set the activation in the threading part.
      PathfindingThread::getSingleton().setActivated( mbActivate );
   }
}

void PathfindingGameSystem::setDivideXFactor( float value )
{
	if( value > 0 )
	{
      for( std::vector<Pathfinding::PathfindingActor*>::iterator iter = mActors.begin(); iter != mActors.end(); iter++ )
      {
         (*iter)->setSubdivisionX( (PathfindingMath::Real)value );
      }
	}
}

void PathfindingGameSystem::setDivideZFactor( float value )
{
	if( value > 0 )
	{
      for( std::vector<Pathfinding::PathfindingActor*>::iterator iter = mActors.begin(); iter != mActors.end(); iter++ )
      {
		   (*iter)->setSubdivisionZ( (PathfindingMath::Real)value );
      }
	}
}


void PathfindingGameSystem::calculatePathfinding()
{
   if( mNode )
   {
      PathfindingDisplay::getSingleton().destroyNode( mNode );
      mNode->removeAndDestroyAllChildren();
   }
   else
   {
      mNode = mGameObjectMgr->getSceneManager()->getRootSceneNode()->createChildSceneNode("manual_nodePath"); 
   }

   Pathfinding::PathfindingAStar aStar;
   Pathfinding::PathfindingPathLinear pathLinear;
   std::vector<Pathfinding::PathfindingAStarNode> lWaypoints = aStar.getPath( PathfindingVector3Adapter( mStartPos ), PathfindingVector3Adapter( mEndPos ), mDefaultActor, &pathLinear, mLvlStart, mLvlEnd );

   if( mLvlEnd >= 1 )
   {
      PathfindingDisplay::getSingleton().showStraightLine( &lWaypoints, mDefaultActor, 2, mNode, mLvlEnd );
   }
   else
   {
      PathfindingDisplay::getSingleton().showCardinalSpline( &lWaypoints, mDefaultActor, 15, mNode, mLvlEnd );
   }

   //PathfindingDisplay::getSingleton().showBezier( &lWaypoints, mDefaultActor, 15, mNode, Pathfinding::PrePathfindingCommon::BASE_LEVEL );

}






} // namespace GOOF
