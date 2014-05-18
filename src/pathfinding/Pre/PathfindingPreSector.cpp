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

#include "Pre/PathfindingPreSector.h"
#include "Pre/PathfindingPreCommon.h"

namespace Pathfinding {

    namespace Pre
    {
        /** 
        Add one cell in the PrePathfindingSector 
        */
        void Sector::addCell( Cell* cell )
        {
            if( mListCell.size() == 0 )
            {
                mListCell.push_front( cell );
                mListNorth.push_front( cell );
                mListSouth.push_front( cell );
                mListEast.push_front( cell );
                mListWest.push_front( cell );
                mTotalCost = cell->getCost();
            }
        }

        /** 
        Add a list of cell in the PrePathfindingSector 
        */
        bool Sector::addCellLine( int direction, const std::list<Cell*> listCell )
        {
            bool lineAdded = false;
            if( listCell.size() > 0 )
            {
                switch( direction )
                {
                case Common::NORTH:
                    if( listCell.front()->getBox().intersects( mListNorth.front()->getBox() ) && listCell.back()->getBox().intersects( mListNorth.back()->getBox() ) )
                    {
                        mListNorth = listCell;
                        lineAdded = true;
                    }
                    break;
                case Common::SOUTH:
                    if( listCell.front()->getBox().intersects( mListSouth.front()->getBox() ) && listCell.back()->getBox().intersects( mListSouth.back()->getBox() ) )
                    {
                        mListSouth = listCell;
                        lineAdded = true;
                    }
                    break;
                case Common::EAST:
                    if( listCell.front()->getBox().intersects( mListEast.front()->getBox() ) && listCell.back()->getBox().intersects( mListEast.back()->getBox() ) )
                    {
                        mListEast = listCell;
                        lineAdded = true;
                    }
                    break;
                case Common::WEST:
                    if( listCell.front()->getBox().intersects( mListWest.front()->getBox() ) && listCell.back()->getBox().intersects( mListWest.back()->getBox() ) )
                    {
                        mListWest = listCell;
                        lineAdded = true;
                    }
                    break;
                }
            }

            if( lineAdded )
            {
                for( std::list<Cell*>::const_iterator iter = listCell.begin(); iter != listCell.end(); iter++ )
                {
                    mListCell.push_front( *iter );
                    mTotalCost += (*iter)->getCost();
                }

                addCellsInOthersDirections( direction );

            }

            return lineAdded;
        }

        /** 
        Add the cell in the other directions. I.E : When a cell line is added in the north, it add a cell at the east and west
        */
        void Sector::addCellsInOthersDirections( int direction )
        {
            switch( direction )
            {
            case Common::NORTH:
                mListWest.push_front( mListNorth.front() );
                mListEast.push_front( mListNorth.back() );
                break;

            case Common::SOUTH:
                mListWest.push_back( mListSouth.front() );
                mListEast.push_back( mListSouth.back() );
                break;

            case Common::EAST:
                mListNorth.push_back( mListEast.front() );
                mListSouth.push_back( mListEast.back() );
                break;

            case Common::WEST:
                mListNorth.push_front( mListWest.front() );
                mListSouth.push_front( mListWest.back() );
                break;
            }    
        }

        void Sector::setCellsInZone()
        {
            for( std::list<Cell*>::const_iterator iter = mListCell.begin(); iter != mListCell.end(); iter++ )
            {
                (*iter)->setInZone( true );
            }
        }
    }
} // namespace 