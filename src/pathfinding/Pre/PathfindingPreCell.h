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

#ifndef _PREPATHFINDINGCELL_H
#define _PREPATHFINDINGCELL_H

#include "PathfindingPrerequisites.h"
#include "Math/PathfindingAxisAlignedBox.h"

namespace Pathfinding 
{
    namespace Pre
    {
        /** @class PrePathfindingCell PathfindingPreCell.h "include/PathfindingPreCell.h"
        *
        * This class is used to represent a cell, which is one position that an actor can go into the world.
        * With cells we create sectors.
        */

        class _PathfindingExport Cell {
        private:
            /// The world representation of the cell
            Pathfinding::Math::AxisAlignedBox mBox;

            Cell*    mEastStep;        /// The cell at the east of this one
            Cell*    mNorthStep;       /// The cell at the north of this one
            Cell*    mSouthStep;       /// The cell at the south of this one
            Cell*    mWestStep;        /// The cell at the west of this one

            //Other infos
            bool                    mBoarderSector;   /// If the Cell is the boarder of a zone
            Pathfinding::Math::Real mCost;            /// The cost of the cell to move from it.  It is an approximation of the 4 moves made.
            bool                    mInZone;          /// A zone has been assigned to this Cell
            bool                    mProcessed;       /// The cell has been processed.  Used by the portal generation algorithm to know if it can continue or not.

        public:
            Cell()
                : mProcessed( false ), mInZone( false ), mBoarderSector( false ), mCost( 0.f ), mNorthStep( NULL ), mSouthStep( NULL ), mEastStep( NULL ), mWestStep( NULL ) {}

            Cell( Pathfinding::Math::AxisAlignedBox & box )
                : mBox( box ), mProcessed( false ), mInZone( false ), mBoarderSector( false ), mCost( 0 ), mNorthStep( NULL ), mSouthStep( NULL ), mEastStep( NULL ), mWestStep( NULL ) {}

            Cell( Pathfinding::Math::AxisAlignedBox & box, Cell * NorthStep, Cell * SouthStep, Cell * EastStep, Cell * WestStep )
                : mBox( box ), mProcessed( false ), mInZone( false ),  mBoarderSector( false ), mCost( 0 ), mNorthStep( NorthStep ), mSouthStep( SouthStep ), mEastStep( EastStep ), mWestStep( WestStep ) {}

            /**
            @return mProcessed
            */
            inline bool getProcessed() const;

            /**
            @param in_processed new value for mProcessed
            */
            void setProcessed( bool in_processed );

            /**
            @return mBox
            */
            inline const Pathfinding::Math::AxisAlignedBox & getBox() const;

            /**
            @param in_processed new value for mProcessed
            */
            void setBox( const Pathfinding::Math::AxisAlignedBox & in_box );

            /**
            @return mCost
            */
            inline const Pathfinding::Math::Real getCost() const;

            /**
            @param in_cost new value for mCost
            */
            void setCost( Pathfinding::Math::Real in_cost ) { mCost = in_cost; }

            /**
            @return mEastStep
            */
            inline Cell* getEastStep() const;

            /**
            @param io_step new value for mEastStep
            */
            void setEastStep( Cell* io_step );

            /**
            @return mWestStep
            */
            inline Cell* getWestStep() const;

            /**
            @param io_step new value for mWestStep
            */
            void setWestStep( Cell* io_step );

            /**
            @return mNorthStep
            */
            inline Cell* getNorthStep() const;

            /**
            @param io_step new value for mNorthStep
            */
            void setNorthStep( Cell* io_step );

            /**
            @return mSouthStep
            */
            inline Cell* getSouthStep() const;

            /**
            @param io_step new value for mSouthStep
            */
            void setSouthStep( Cell* io_step );

            /**
            @return mInZone
            */
            inline const bool getInZone() const;

            /**
            @param in_value new value for mInZone
            */
            void setInZone( bool in_value );
        };

        inline bool Cell::getProcessed() const { return mProcessed; } 

        inline const Pathfinding::Math::AxisAlignedBox & Cell::getBox() const { return mBox; }

        inline const Pathfinding::Math::Real Cell::getCost() const { return mCost; }

        inline Cell* Cell::getEastStep() const { return mEastStep; }

        inline Cell* Cell::getWestStep() const { return mWestStep; }

        inline Cell* Cell::getNorthStep() const { return mNorthStep; }

        inline Cell* Cell::getSouthStep() const { return mSouthStep; }

        inline const bool Cell::getInZone() const { return mInZone; }

    } //namespace Pre
} // namespace Pathfinding
#endif
