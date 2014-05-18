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

#ifndef __GOOFPathfindingGameSystemFactory_H__
#define __GOOFPathfindingGameSystemFactory_H__

#include "GOOFPathfindingGameSystemPrerequisites.h"
#include "GOOFGameSystemFactory.h"
#include <Ogre.h>

using namespace Ogre;
using namespace std;

namespace GOOF
{
    /** @class PathfindingGameSystemFactory GOOFPathfindingGameSystemFactory.h "include/GOOFPathfindingGameSystemFactory.h"
    *
    * This class is used to create GameSystems
    */

	class _GOOFPathfindingGameSystemExport PathfindingGameSystemFactory : public GameSystemFactory
	{
	public:

		/** 
		*/
		PathfindingGameSystemFactory() {}

		/** 
		*/
		virtual ~PathfindingGameSystemFactory() {}

		/** 
         @param[in] in_gameSystemType the name of the game system that is being searched
         @return if the parameter is the same name as the pathfinding game system.
		*/
		virtual bool supportsGameSystemType(const Ogre::String& in_gameSystemType) const;

		/** Create a game system of the type provided
         @param[in] in_gameSystemType the name of the game system to be created
         @return null if the game system name is not the good one. Else it returns the created GameSystem
		*/
		virtual GameSystem* createGameSystem(const Ogre::String& in_gameSystemType) const;
		
	};
}

#endif //__GOOFPathfindingGameSystemFactory_H__
