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

#ifndef __GOOFPathfindingGameSystemPrerequisites_H__
#define __GOOFPathfindingGameSystemPrerequisites_H__

#include "OgrePrerequisites.h"

namespace GOOF 
{
/*   
   #define OGRE_LOCK_AUTO_MUTEX mutex
   // Taken from OgrePrerequiste.  Make it to GOOF, so make sure it dosen't need any change to Ogre.
   #define OGRE_AUTO_MUTEX mutable boost::recursive_mutex OGRE_LOCK_AUTO_MUTEX;
   #define OGRE_LOCK_AUTO_MUTEX boost::recursive_mutex::scoped_lock goofAutoMutexLock(OGRE_LOCK_AUTO_MUTEX);
   #define OGRE_MUTEX(name) mutable boost::recursive_mutex name;
   #define OGRE_LOCK_MUTEX(name) boost::recursive_mutex::scoped_lock goofnameLock(name);
   // like OGRE_AUTO_MUTEX but mutex held by pointer
   #define OGRE_AUTO_SHARED_MUTEX mutable boost::recursive_mutex *OGRE_LOCK_AUTO_MUTEX;
   #define OGRE_LOCK_AUTO_SHARED_MUTEX assert(OGRE_LOCK_AUTO_MUTEX); boost::recursive_mutex::scoped_lock goofAutoMutexLock(*OGRE_LOCK_AUTO_MUTEX);
   #define OGRE_NEW_AUTO_SHARED_MUTEX assert(!OGRE_LOCK_AUTO_MUTEX); OGRE_LOCK_AUTO_MUTEX = new boost::recursive_mutex();
   #define OGRE_DELETE_AUTO_SHARED_MUTEX assert(OGRE_LOCK_AUTO_MUTEX); delete OGRE_LOCK_AUTO_MUTEX;
   #define OGRE_COPY_AUTO_SHARED_MUTEX(from) assert(!OGRE_LOCK_AUTO_MUTEX); OGRE_LOCK_AUTO_MUTEX = from;
   #define OGRE_SET_AUTO_SHARED_MUTEX_NULL OGRE_LOCK_AUTO_MUTEX = 0;
   #define OGRE_MUTEX_CONDITIONAL(mutex) if (mutex)
*/

	// Forward declaration
	class PathfindingGameSystem;
	class PrePathfindingPhysic;
	class PrePathfindingCompute;
	class PathfindingData;
   class PathfindingDisplay;
   class PathfindingThread;
   class PathfindingDelegate;

}

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#   ifdef GOOFPATHFINDINGGAMESYSTEM_EXPORTS
#       define _GOOFPathfindingGameSystemExport __declspec(dllexport) 
#   else 
#       define _GOOFPathfindingGameSystemExport __declspec(dllimport) 
#   endif 
#else 
#   define _GOOFPathfindingGameSystemExport 
#endif 

#endif //__GOOFPathfindingGameSystemPrerequisites_H__
