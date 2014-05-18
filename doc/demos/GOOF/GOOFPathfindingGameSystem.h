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

#ifndef _GOOFPATHFINDING_H
#define _GOOFPATHFINDING_H

#include "GOOFCommonPrerequisites.h"
#include "GOOFCoreGameObjectManager.h"
#include "GOOFGameObjectManagerGameSystem.h"
#include "GOOFPathfindingGameSystemPrerequisites.h"
#include "GOOFPrePathfindingCompute.h"
#include "GOOFPrePathfindingPhysic.h"
#include "GOOFPathfindingData.h"
#include "GOOFPathfindingThread.h"
#include "GOOFPathfindingDelegate.h"
#include "GOOFPathfindingDisplay.h"

#include <Ogre.h>

namespace GOOF {
   /** @class PathfindingGameSystem GOOFPathfindingGameSystem.h "include/GOOFPathfindingGameSystem.h"
    *
    * This class is the main class to control the pathfinding into GOOF.
    */

	class _GOOFPathfindingGameSystemExport PathfindingGameSystem : public GameObjectManagerGameSystem {
  public:

     /**
       Not yet implemented
       @param[in] sector the zone to be precalculated
       @Return true if the data has been generated.
     */
    bool generatePrePathfinding(const PathfindingMath::AxisAlignedBox & in_sector);
	
	PathfindingGameSystem();

	/** 
      Destructor
	*/
	virtual ~PathfindingGameSystem();

	/** 
     * frame started event, pass it down to the Pathfinding manager
     * @param in_deltaTime time allowed to execute.
	*/
	virtual void update(float in_deltaTime);

   /** 
     *  Initialization of the game system.  Called by the framework.
     *  @param io_gameObjectMgr gameObjectManager to be set
   */
	virtual void init(GameObjectManager* in_gameObjectMgr);

	/**
     *  Not yet implemented
     *  Saves values to the element provided
     *  @param in_element to be saved
	*/
	virtual bool save(DataElementPtr in_element);

	/** 
     *  Not yet implemented
     *  Loads values from the element provided
     *  @param in_element to be loaded
	*/
	virtual bool load(DataElementPtr in_element);

	/** 
     *  Enumerate a list of properties which can be set via setProperty
     *  @param[out] out_properties properties will be put into that parameter
	*/
	virtual void enumerateProperties(vector<Property>& out_properties);


	/** 
     *  Assigns a value to a property.  This function is called when the user changed a property.
     *  @param[in] in_id name of the property
     *  @param[in] in_data Value of the property
	*/
	virtual void setProperty(const String& in_id, const PropertyData& in_data);

   /**
    * @param[in] in_value value to make X Factor.  value cannot be 0.
   */
	void setDivideXFactor( float in_value );
   
   /**
    * @param[in] in_value value to make Z Factor.  value cannot be 0.
   */
   void setDivideZFactor( float in_value );

   /**
    * This function will calculate the pathfinding defined by mStartPos and mEndPos
   */
	void calculatePathfinding();

	protected:
		CoreGameObjectManager*                       mGameObjectMgr;   ///The object manager

		std::vector<Pathfinding::PathfindingActor*>  mActors;          /// List of all type of actors in Switch Blade
		Pathfinding::PathfindingActor*               mDefaultActor;    /// Default actor is the one that will be calculated first
      PathfindingDelegate*                         mDelegate;        /// The class that handles all the delegates from Switch Blade

		PathfindingMath::Real                        mXDivideFactor;   /// The actor small vector X division
		PathfindingMath::Real                        mZDivideFactor;   /// The actor small vector Z division
		unsigned int                                 mMakeAction;      /// The number of the action that has been called
      Ogre::Vector3                                mStartPos;        /// The start position of the pathfinding
      Ogre::Vector3                                mEndPos;          /// The end position of the pathfinding
      unsigned int                                 mSPLevel;         /// The level of the sectors and portals shown
      int                                          mLvlStart;        /// The start level of the pathfinding
      int                                          mLvlEnd;          /// The end level of the pathfinding
      bool                                         mbActivate;       /// The pathfinding is activated
      Ogre::SceneNode*                             mNode;            /// Node to show lines.

};

} // namespace GOOF
#endif
