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

#include "GOOFPathfindingGameSystemFactory.h"
#include "GOOFPathfindingGameSystem.h"

namespace GOOF
{

	//-----------------------------------------------------------------------
   bool PathfindingGameSystemFactory::supportsGameSystemType(const Ogre::String& gameSystemType) const
	{
      //Ogre::LogManager::getSingleton().logMessage( "PATHFINDING Test:" + gameSystemType );

      bool ret = false;
		if(gameSystemType == "PathfindingGameSystem")
      {
        // Ogre::LogManager::getSingleton().logMessage( "PATHFINDING true" );
			ret = true;
      }
      
      return ret;
	}

	//-----------------------------------------------------------------------
	GameSystem* PathfindingGameSystemFactory::createGameSystem(const Ogre::String& gameSystemType) const
	{
      GameSystem* gameSystem = NULL;

		if(gameSystemType == "PathfindingGameSystem")
		{	
			gameSystem = new PathfindingGameSystem();
		}
		
      return gameSystem;
	}
}
