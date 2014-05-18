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

#ifndef _PREPATHFINDINGSECTOR_H
#define _PREPATHFINDINGSECTOR_H

#include "PathfindingPrerequisites.h"
#include "Math/PathfindingAxisAlignedBox.h"
#include "PathfindingPreCell.h"

namespace Pathfinding {

    namespace Pre
    {
        /** @class PrePathfindingSector PathfindingPreSector.h "include/PathfindingPreSector.h"
        *
        * This class is used to represent a PrePathfindingSector. The difference between
        * a PrePathfindingSector and a Pathfinding::PathfindingSector is that a PrePathfindingSector contains cells which
        * defines the sector and is not used at real time.  It is used to create portals between sectors.
        * The pathfindingSector contains the portals and the information needed to be used at real time.
        */

        class _PathfindingExport Sector {
        private:
            std::list<Cell*> mListNorth;   /// All the cells that are at the north of the portal
            std::list<Cell*> mListSouth;   /// All the cells that are at the south of the portal
            std::list<Cell*> mListEast;    /// All the cells that are at the east of the portal
            std::list<Cell*> mListWest;    /// All the cells that are at the west of the portal
            std::list<Cell*> mListCell;    /// All the cells that are in the PrePathfindingSector

            Pathfinding::Math::Real mTotalCost;                       /// The sum of all the cells into this PrePathfindingSector

        public:
            Sector() : mTotalCost( 0.f ) {}
            Sector( Cell* cell ) : mTotalCost( 0.f ) { addCell( cell ); }

            /**
            * @return a box of the size of the extrema cells
            */
            inline Pathfinding::Math::AxisAlignedBox getBox() const;

            /** 
            * Add one cell in the PrePathfindingSector, only possible for the first cell 
            *
            * @param io_cell the first cell to be added
            */
            void addCell( Cell* io_cell );

            /** 
            *    Add a list of cell in the PrePathfindingSector, it must be a new line connected to the previous one
            * The result will be put directly in the list of the direction
            *
            *    @param[in] in_dir the direction where it's going
            *    @param[in] in_listCell the list of PrePathfindingCell before moving
            *    @return true if the line has been added
            */
            bool addCellLine( int in_dir, const std::list<Cell*> in_listCell );

            /**
            * @return mTotalCost
            */
            inline Pathfinding::Math::Real Sector::getTotalCost() const;

            /**
            * @return average cost of the PrePathfindingSector
            */
            inline Pathfinding::Math::Real Sector::getAverageCost() const;

            /**
            * @return mListNorth
            */
            inline const std::list<Cell*> & getNorthList() const;

            /**
            * @return mListSouth
            */
            inline const std::list<Cell*> & getSouthList() const;

            /**
            * @return mListEast
            */
            inline const std::list<Cell*> & getEastList() const;

            /**
            * @return mListWest
            */
            inline const std::list<Cell*> & getWestList() const;

            /**
            * Set the cells that are the border of the zone to be put as border cells
            */ 
            void setCellsInZone();

        private:
            /** 
            * Add the cell in the other directions. I.E : When a cell line is added in the north, it add a cell at the east and west
            *
            * @param[in] in_dir the direction where it's going
            */
            void addCellsInOthersDirections( int in_dir );
        };


        inline Pathfinding::Math::AxisAlignedBox Sector::getBox() const
        {
            Pathfinding::Math::AxisAlignedBox secBox;
            if( mListSouth.size() > 0 && mListNorth.size() > 0 )
            {
                secBox = mListSouth.front()->getBox();
                secBox.merge( mListNorth.back()->getBox() );
            }
            return secBox;
        }

        inline Pathfinding::Math::Real Sector::getTotalCost() const
        {
            return mTotalCost;
        }

        inline Pathfinding::Math::Real Sector::getAverageCost() const
        {
            Pathfinding::Math::Real ret = 0;
            if( mListCell.size() > 0 )
            {
                ret = mTotalCost / mListCell.size();
            }
            return ret;
        }

        inline const std::list<Cell*> & Sector::getNorthList() const
        {
            return mListNorth;
        }

        inline const std::list<Cell*> & Sector::getSouthList() const
        {
            return mListSouth;
        }

        inline const std::list<Cell*> & Sector::getEastList() const
        {
            return mListEast;
        }

        inline const std::list<Cell*> & Sector::getWestList() const
        {
            return mListWest;
        }

    }//namespace Pre
}// namespace Pathfinding

#endif //_PREPATHFINDINGSECTOR_H
