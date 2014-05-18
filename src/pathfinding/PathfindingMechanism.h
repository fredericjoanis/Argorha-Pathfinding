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

#ifndef _PATHFINDINGMECHANISM_H
#define _PATHFINDINGMECHANISM_H

#include "PathfindingPrerequisites.h"

namespace Pathfinding {

    /** @class PathfindingMechanism PathfindingMechanism.h "include/PathfindingMechanism.h"
    *
    * This class is not yet implemented.  
    * The class is gonna be implemented with systems.
    */

    class _PathfindingExport Mechanism {

    private:
        /// The cost of the mechanism
        float mCost;

        /// The list of actions needed to be made for the mechanism to be passed
        std::list<const Action*> mActions;


    public:
        /**
        @return mCost
        */
        inline const float getCost() const;

        /**
        @param in_value new value for mCost
        */
        void setCost(float in_value);

        /**
        @return list of needed action to be performed by the mechanism to be reached.
        */
        inline const std::list<const Action *> getActions() const;

        /**
        @param io_action add this action to the list of needed action to be performed by the mechanism to be reached.
        */
        void addAction( const Action * in_action);

        /**
        @param io_action remove this action from the list of needed action to be performed by the mechanism to be reached.
        */
        void removeAction( Action * io_action);

    };
    inline const float Mechanism::getCost() const {
        return mCost;
    }

    inline const std::list<const Action*> Mechanism::getActions() const {
        return mActions;
    }


} // namespace Pathfinding
#endif
