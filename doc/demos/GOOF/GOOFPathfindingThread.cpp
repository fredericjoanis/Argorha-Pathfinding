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

#include "GOOFPathfindingThread.h"
#include "GOOFGridPartitionManager.h"
#include "GOOFCorePartition.h"
#include <boost/bind.hpp>
#include "OgreSingleton.h"
#include "boost/thread/xtime.hpp"
#include "OgrePrerequisites.h"
#include "PrePathfindingCommon.h"
#include "Utilities/Math/PathfindingAxisAlignedBoxAdapter.h"

template<> GOOF::PathfindingThread* Ogre::Singleton<GOOF::PathfindingThread>::ms_Singleton = 0;
using namespace Ogre;

namespace GOOF 
{
   PathfindingThread::PathfindingThread()
   {
      {
         OGRE_LOCK_MUTEX_NAMED(initMutex, initLock)
         Ogre::Root::getSingleton().getRenderSystem()->preExtraThreadsStarted();
         mThread        = new boost::thread(boost::function0<void>(&PathfindingThread::threadFunc));
         
         OGRE_THREAD_WAIT(initSync, initLock)
         
         Ogre::Root::getSingleton().getRenderSystem()->postExtraThreadsStarted();
      }

      mNextTicketID  = 0;
      mbIsSleeping   = false;
      mbActivated    = true;
   }

   PathfindingThread::~PathfindingThread()
   {
      if (mThread)
		{
			delete mThread;
			mThread = 0;
			mRequestQueue.clear();
			mRequestTicketMap.clear();
		}
      if( ms_Singleton)
      {
         delete ms_Singleton;
      }
   }


   PathfindingThread& PathfindingThread::getSingleton(void)
   {  
      if( !ms_Singleton )
      {
         // The singleton has not been created, create it
         new PathfindingThread();
      }
      assert( ms_Singleton );  return ( *ms_Singleton );  
      //return Singleton<PathfindingThread>::getSingleton();
   }

   void PathfindingThread::addPartitionToQueue( const Ogre::String & in_partID )
   {
      if( mbActivated )
      {
         Request req;
		   req.type = RT_GENERATE_PARTITION;
		   req.sPartitionID = in_partID;

         addRequest( req );
      }
   }

   BackgroundProcessTicket PathfindingThread::addRequest(Request& req)
	{
		// Lock
		OGRE_LOCK_AUTO_MUTEX

      bool bUnique = true;
      bool bFirstElement = false;
      RequestQueue::iterator iter = mRequestQueue.begin();
      
      req.ticketID = ++mNextTicketID;

      //It's the first one, we need to handle this case differently because it's being calculated and we want to kill the thread.
      if( iter != mRequestQueue.end() && iter->sPartitionID == req.sPartitionID )
      {
         mCompute.setContinue( false );
         bUnique = false;
         bFirstElement = true;
      }

      while( bUnique && iter != mRequestQueue.end() )
      {
         if( iter->sPartitionID == req.sPartitionID )
         {
            mRequestQueue.erase( iter );
            bUnique = false;
         }
         else
         {
            iter++;
         }
      }

      if( !bFirstElement )
      {
         mRequestQueue.push_back(req);

		   Request* requestInList = &(mRequestQueue.back());
		   mRequestTicketMap[req.ticketID] = requestInList;

		   // Notify to wake up loading thread
         if( !mbIsSleeping )
         {
            if( !mThread )
            {
               mThread = new boost::thread(boost::function0<void>(&PathfindingThread::threadFunc));
            }
		      mCondition.notify_one();
         }
      }
 
		return req.ticketID;
	}


   void PathfindingThread::setGameObjectMgr( CoreGameObjectManager* gameObjectMgr )
   {
      mGameObjectMgr = gameObjectMgr;
      mPhysic.setGameObjectMgr( gameObjectMgr );
      PathfindingDisplay::getSingleton().setGameObjectMgr( mGameObjectMgr );
   }

   CoreGameObjectManager* PathfindingThread::getGameObjectMgr()
   {
      OGRE_LOCK_AUTO_MUTEX
      return mGameObjectMgr;
   }

   PrePathfindingCompute& PathfindingThread::getCompute()
   {
      OGRE_LOCK_AUTO_MUTEX
      return mCompute;
   }

   std::vector<Pathfinding::PathfindingActor*>* PathfindingThread::getActors()
   {
      OGRE_LOCK_AUTO_MUTEX
      return mpListActors;
   }

   Pathfinding::AbstractPrePathfindingPhysic* PathfindingThread::getPhysic()
   {
      return static_cast<Pathfinding::AbstractPrePathfindingPhysic*>(&mPhysic);
   }

   void PathfindingThread::setActorArray( std::vector<Pathfinding::PathfindingActor*>* io_listActor )
   {
      mpListActors = io_listActor;
   }

   void PathfindingThread::_initThread()
   {
      Root::getSingleton().getRenderSystem()->registerThread();
		{
			// notify waiting thread(s)
			OGRE_LOCK_MUTEX(initMutex)
			OGRE_THREAD_NOTIFY_ALL(initSync)
		}
   }

   void PathfindingThread::threadFunc( void )
   {
      // Background thread implementation 
		// Static (since no params allowed), so get instance
		PathfindingThread& queueInstance = PathfindingThread::getSingleton();

      queueInstance._initThread();
		bool shuttingDown = false;
		// Spin forever until we're told to shut down
		while (!shuttingDown)
		{
			Request* req;
			// Manual scope block just to define scope of lock
			{
				// Lock; note that 'mCondition.wait()' will free the lock
				boost::recursive_mutex::scoped_lock queueLock(
                    queueInstance.OGRE_AUTO_MUTEX_NAME);
				if (queueInstance.mRequestQueue.empty())
				{
					// frees lock and suspends the thread
					queueInstance.mCondition.wait(queueLock);
				}
				// When we get back here, it's because we've been notified 
				// and thus the thread as been woken up. Lock has also been
				// re-acquired.

				// Process one request
				req = &(queueInstance.mRequestQueue.front());
			} // release lock so queuing can be done while we process one request
			// use of std::list means that references guaranteed to remain valid

         bool bEraseTicket = true; // If the data has been all processed, delete the ticket, else start again.
			ResourceManager* rm = 0;
			switch (req->type)
			{
		      case RT_INITIALISE:
			      break;
		      case RT_GENERATE_PARTITION:
               {
                  GOOF::GridPartitionManager *part = dynamic_cast<GridPartitionManager*>( PathfindingThread::getSingleton().getGameObjectMgr()->getPartitionManager() );

                  PathfindingMath::AxisAlignedBox zoneBox = PathfindingAxisAlignedBoxAdapter( part->getPartition( req->sPartitionID )->getWorldPartitionBox() );

                  // Process them all.  Might have an algorithm to process the default one first, and when list is empty
                  // Process others size
                  for( std::vector<Pathfinding::PathfindingActor*>::iterator iter = queueInstance.getActors()->begin(); iter != queueInstance.getActors()->end(); iter++ )
                  {
                     if( PathfindingThread::getSingleton().getCompute().getContinue() ) 
                     {
                        // Needed to clear the data at the right time
                        PathfindingThread::getSingleton().getCompute().setPartitionID( req->sPartitionID );
                        // Cast to GOOF::PathfindingData to clear the partition that has been recalculated
                        PathfindingThread::getSingleton().getCompute().generateSectorsAndPortals( PathfindingThread::getSingleton().getPhysic(), (*iter), &zoneBox, Pathfinding::PrePathfindingCommon::BASE_LEVEL );
                        PathfindingThread::getSingleton().getCompute().generatePortalsAroundZone( PathfindingThread::getSingleton().getPhysic(), (*iter), &zoneBox, Pathfinding::PrePathfindingCommon::BASE_LEVEL );

                        static_cast<GOOF::PathfindingData*>((*iter)->getData())->clearPartition( req->sPartitionID, 1 );
                        PathfindingThread::getSingleton().getCompute().generateSectorsAndPortals( PathfindingThread::getSingleton().getPhysic(), (*iter), &zoneBox, 1 );

                        PathfindingDisplay::getSingleton().addPortalsToShow( req->sPartitionID );
                        PathfindingDisplay::getSingleton().addSectorsToShow( req->sPartitionID );

                        //PathfindingDisplay::getSingleton().showSectors( req->sPartitionID, static_cast<GOOF::PathfindingData*>((*iter)->getData()), queueInstance.getSPLevel() );
                        //PathfindingDisplay::getSingleton().showPortals( req->sPartitionID, static_cast<GOOF::PathfindingData*>((*iter)->getData()), queueInstance.getSPLevel() );
                     }
                     else
                     {
                        // Tells the other function that it dosen't want to be waked up
                        PathfindingThread::getSingleton().setIsSleeping( true );
                        boost::xtime tTime;
                        xtime_get(&tTime, boost::TIME_UTC);
                        tTime.sec += 10;
                        // Stop the thread.
                        boost::recursive_mutex::scoped_lock tLock( queueInstance.OGRE_AUTO_MUTEX_NAME );
                        queueInstance.mCondition.timed_wait(tLock, tTime);
                        PathfindingThread::getSingleton().setIsSleeping( false );
                        bEraseTicket = false;
                        // Start back the calculation for this actor again.
                     }
                  }
                  PathfindingThread::getSingleton().getCompute().setContinue( true );
               }
			      break;
		      case RT_SHUTDOWN:
			      // If there is something computing, make sure it stop.
               PathfindingThread::getSingleton().getCompute().setContinue(false);
               // That's all folks
			      shuttingDown = true;
               Root::getSingleton().getRenderSystem()->unregisterThread();
			      break;
			};


         {
            // re-lock to consume completed request
            boost::recursive_mutex::scoped_lock queueLock(queueInstance.OGRE_AUTO_MUTEX_NAME);
            // The algorithm has been fully calculated
            if( bEraseTicket )
			   {
				   // Consume the ticket
				   queueInstance.mRequestTicketMap.erase(req->ticketID);
			   }
            else // The algorithm has been interrupted, push the first element to the last element
            {
               queueInstance.mRequestQueue.push_back( queueInstance.mRequestQueue.front() );
            }
            queueInstance.mRequestQueue.pop_front();
         }
      }
   }

   bool PathfindingThread::getIsSleeping()
   {
      return mbIsSleeping;
   }

   void PathfindingThread::setIsSleeping( bool in_bIsSleeping )
   {
      mbIsSleeping = in_bIsSleeping;
   }

   void PathfindingThread::setActivated( bool in_bActivated )
   {
      mbActivated = in_bActivated;
   }

   bool PathfindingThread::getActivated()
   {
      return mbActivated;
   }
}
