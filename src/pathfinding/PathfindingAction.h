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

#ifndef _PATHFINDINGACTION_H
#define _PATHFINDINGACTION_H

#include "PathfindingPrerequisites.h"

namespace Pathfinding {

    /** @class PathfindingAction PathfindingAction.h "include/PathfindingAction.h"
    *
    * This class is not yet implemented.  
    * The class is gonna be implemented with systems.
    */
    class _PathfindingExport Action {

    private:
        float mCost; /// The cost of the action

    public:
        /// Copy constructor, so it can be added in containers
        Action( const Action &act );

        /**
        @return mCost
        */
        inline const float getCost() const;

        /**
        compare two actions
        */
        bool operator<(const Action &in_act) const;

        /**
        @param in_value new mCost value
        */
        void setCost(float in_value);

    };

    inline const float Action::getCost() const {
        return mCost;
    }

} // namespace Pathfinding
#endif
