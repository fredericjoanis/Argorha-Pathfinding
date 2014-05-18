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

#ifndef __PATHFINDINGPREREQUISTE_H__
#define __PATHFINDINGPREREQUISTE_H__

#include "Math/PathfindingVector3.h"
#include "Math/PathfindingAxisAlignedBox.h"

namespace Pathfinding 
{
    // Forward declaration
    
    class Action;
    class Actor;
    class AStar;
    class AStarNode;
    class Data;
    class Mechanism;
    class Portal;
    class Sector;
    namespace Pre
    {
        class Cell;
        class Common;
        class Compute;
        class Physic;
        class Sector;
    }
}

#ifdef WIN32
#   ifdef PATHFINDING_EXPORTS
#       define _PathfindingExport __declspec(dllexport) 
#   else 
#       define _PathfindingExport __declspec(dllimport) 
#   endif 
#else
#define _PathfindingExport
#endif

#endif //__PATHFINDINGPREREQUISTE_H__
