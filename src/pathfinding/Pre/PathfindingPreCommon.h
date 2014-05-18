/*
-----------------------------------------------------------------------------
This source file is part of Argorha project

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

#ifndef PREPATHFINDINGCOMMON_H
#define PREPATHFINDINGCOMMON_H

namespace Pathfinding
{
    namespace Pre
    {
        /** @class PrePathfindingCommon PathfindingPreCommon.h "include/PathfindingPreCommon.h"
        *
        * This class is a class with data common to every other classes
        */
        class _PathfindingExport Common
        {
        public :
            //Important : Directions must be continuous
            static const int Common::NORTH = 0;
            static const int Common::SOUTH = 1;
            static const int Common::EAST = 2;
            static const int Common::WEST = 3;

            //Give variables to be able to go trough all directions
            static const int Common::FIRST_DIRECTION = Common::NORTH;
            static const int Common::LAST_DIRECTION = Common::WEST;

            /// The hierarchical base level is starting at 0
            static const int Common::BASE_LEVEL = 0;
        };
    }
}

#endif // PREPATHFINDINGCOMMON_H
