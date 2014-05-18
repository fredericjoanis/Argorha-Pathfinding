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

#ifndef _GOOFPATHFINDINGTHREAD_H
#define _GOOFPATHFINDINGTHREAD_H

#pragma once

#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include "OgreString.h"
#include "GOOFPrePathfindingCompute.h"
#include "GOOFCoreGameObjectManager.h"
#include "GOOFPrePathfindingPhysic.h"
#include "GOOFCorePrerequisites.h"
#include "OgrePrerequisites.h"
#include "GOOFPathfindingGameSystemPrerequisites.h"
#include "GOOFPathfindingDisplay.h"

namespace GOOF {

   typedef unsigned long BackgroundProcessTicket;


 /** @class PathfindingThread GOOFPathfindingThread.h "include/GOOFPathfindingThread.h"
 *
 * This class is handle the request to generate pathfinding in partitions.
 * The threading code is heavily coming from the background resource loading system from Ogre.
 */
class _GOOFPathfindingGameSystemExport PathfindingThread : public Ogre::Singleton<PathfindingThread>
{

private:
   PathfindingThread(void);
   virtual ~PathfindingThread(void);

public:
   /**
      Function to return the singleton of the class
   */
   static PathfindingThread&                    getSingleton(void);

   /**
      Adds a partition to be processed in the queue
      @param[in] in_partID the partition that needs to be recalculated
   */
   void                                         addPartitionToQueue( const Ogre::String & in_partID );

   /** 
      @param io_gameObjectMgr gameObjectManager to be set
   */
   void                                         setGameObjectMgr( CoreGameObjectManager* gameObjectMgr );

   /** 
      @param io_listActor list of actors to be set
   */
   void                                         setActorArray( std::vector<Pathfinding::PathfindingActor*>* io_listActor );

   /**
    @param in_bActivated new value for mActivated
   */
   void                                         setActivated( bool in_bActivated );

   /**
    @return mActivated
   */
   bool                                         getActivated();

   /**
    @return the mGameObjectMgr
   */
   CoreGameObjectManager*                       getGameObjectMgr();

   /**
    @return mCompute
   */
   GOOF::PrePathfindingCompute&                 getCompute();

   /**
    @return mpListActors
   */
   std::vector<Pathfinding::PathfindingActor*>* getActors();

   /**
    @return mPhysic
   */
   Pathfinding::AbstractPrePathfindingPhysic*   getPhysic();

   /**
    @return mbIsSleeping
   */
   bool                                         getIsSleeping();

   /**
    @param[in] in_bISleeping sets mbIsSleeping
   */
   void                                         setIsSleeping( bool in_bIsSleeping );

protected:

   GOOF::PrePathfindingCompute                  mCompute;         /// The class that compute the prepathfinding data
   CoreGameObjectManager*                       mGameObjectMgr;   /// The game object manager
   std::vector<Pathfinding::PathfindingActor*>* mpListActors;     /// The list of actors
   GOOF::PrePathfindingPhysic                   mPhysic;          /// The physic to calculate
   bool                                         mbIsSleeping;     /// Variable to know if the thread is sleeping or waiting to be notify.
   bool                                         mbActivated;      /// Tells if the pathfinding can continue to call flood fills.

	/** Enumerates the type of requests */
	enum RequestType
	{
      RT_INITIALISE,
		RT_GENERATE_PARTITION,
		RT_SHUTDOWN
	};
	/** Encapsulates a queued request for the background queue */
	struct Request
	{
		BackgroundProcessTicket ticketID;
		RequestType type;
      Ogre::String sPartitionID;
	};

	typedef std::list<Request> RequestQueue;
	typedef std::map<BackgroundProcessTicket, Request*> RequestTicketMap;
	
	/// Queue of requests, used to store and order requests
	RequestQueue mRequestQueue;
	
	/// Request lookup by ticket
	RequestTicketMap mRequestTicketMap;

	/// Next ticket ID
	unsigned long mNextTicketID;

	/// The single background thread which will process loading requests
	boost::thread* mThread;
	/// Synchronizer token to wait / notify on queue
	boost::condition mCondition;
	/// Thread function
	static void threadFunc(void);
	/// Internal method for adding a request; also assigns a ticketID
	BackgroundProcessTicket addRequest(Request& req);
   /// Initialisation of a new thread
   void _initThread();

	/// Init notification mutex (must lock before waiting on initCondition)
   OGRE_MUTEX(initMutex)
	/// Synchroniser token to wait / notify on thread init (public incase external thread)
	OGRE_THREAD_SYNCHRONISER(initSync);

	/// public mutex, allowed to lock from outside
	OGRE_AUTO_MUTEX


};

}
#endif
