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

#include "PathfindingMechanism.h"
#include "PathfindingAction.h"

namespace Pathfinding {

    void Mechanism::setCost(float value) {
        mCost = value;
    }

    //Return true if the element has been added
    void Mechanism::addAction(const Action * action)
    {
        this->mActions.push_front( action );
        this->mActions.unique();
    }

    //Return true if the element has been deleted
    void Mechanism::removeAction(Action *)
    {
        //this->mActions.erase(action);
    }


} // namespace Pathfinding
