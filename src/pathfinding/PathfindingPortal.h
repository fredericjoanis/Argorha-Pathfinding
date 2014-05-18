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

#ifndef _PATHFINDINGPORTAL_H
#define _PATHFINDINGPORTAL_H


#include "PathfindingPrerequisites.h"
#include "PathfindingMechanism.h"
#include "Math/PathfindingAxisAlignedBox.h"
#include "PathfindingSector.h"

namespace Pathfinding {

    /** @class PathfindingPortal PathfindingPortal.h "include/PathfindingPortal.h"
    *
    * This class is used to represent a portal.
    */

    class _PathfindingExport Portal {
    private:
        /// The box defining the portal
        Pathfinding::Math::AxisAlignedBox mBox;

        /// The destination sector 1
        Sector* mDestSector1;

        /// The destination sector 2
        Sector* mDestSector2;

        /// The mechanism that the portal is defining.
        Mechanism mMechanism;

        /// Actions needed to be performed to go trough the portal.
        std::list<const Action*> mActions;

    public:

        Portal();
        Portal(const Portal &in_portal);
        ~Portal();

        /**
        * Compare if this portal is at a lower position than the input one.  It is used to do a discrimination between portals.
        * This is to find portals faster.
        * 
        * @param in_portal the portal to compare with
        * @return true if the actual portal is at a lower position than the other.
        */
        bool operator<(const Portal &in_portal) const;

        /**
        * Compare if this portal is at an equal position position than the input one.
        * 
        * @param in_portal the portal to compare with
        * @return true if the actual portal is at same position than the other.
        */
        bool operator==(const Portal &in_portal) const;

        /**
        @return mBox
        */
        inline const Pathfinding::Math::AxisAlignedBox& getBox() const;

        /**
        @param in_value new value for mBox
        */
        void setBox(const Pathfinding::Math::AxisAlignedBox & in_value );

        /**
        @return mDestSector1. Version with const
        */
        inline const Sector* getDestSector1() const;

        /**
        @param in_value new value for mDestSector1
        */
        void setDestSector1( Sector * in_value );

        /**
        @return mDestSector2. Version with const
        */
        inline const Sector* getDestSector2() const;

        /**
        @param in_value new value for mDestSector2
        */
        void setDestSector2( Sector * in_value );

        /**
        @return mMechansism
        */
        inline const Mechanism& getMechanism() const;

        /**
        @param in_value new value for mMechanism
        */
        void setMechanism(const Mechanism & in_value );

        /**
        @return mActions
        */
        inline const std::list<const Action*> getActions() const;

        /** 
        Add the action in the list of action
        */
        void addAction(const Action * in_action);

        /**
        @return mDestSector1. Version without const
        */
        Sector* getDestSector1();

        /**
        @return mDestSector2. Version without const
        */
        Sector* getDestSector2();

    };


    inline const Pathfinding::Math::AxisAlignedBox& Portal::getBox() const 
    {
        return mBox;
    }

    inline const Sector* Portal::getDestSector1() const 
    {
        return mDestSector1;
    }

    inline const Sector* Portal::getDestSector2() const 
    {
        return mDestSector2;
    }

    inline const Mechanism& Portal::getMechanism() const 
    {
        return mMechanism;
    }

    inline const std::list<const Action*> Portal::getActions() const 
    {
        return mActions;
    }

} // namespace Pathfinding
#endif
