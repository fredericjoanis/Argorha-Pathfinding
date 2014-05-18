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

#ifndef _PREPATHFINDINGCOMPUTE_H
#define _PREPATHFINDINGCOMPUTE_H

#include "../PathfindingPrerequisites.h"
#include "../Math/PathfindingAxisAlignedBox.h"

namespace Pathfinding 
{
    namespace Pre
    {
        /** @class PrePathfindingCompute PathfindingPreCompute.h "include/PathfindingPreCompute.h"
        *
        * This class is the brain of the prepathfinding, containing all the algorithms needed to create sectors and portals.
        */

        class _PathfindingExport Compute 
        {
        public:
            // Default constructor
            Compute() {}

            virtual ~Compute() {}

        public:


            /**
            * This function is gonna create the sectors and portals. For level 0, it use a fillAll algorithm.
            * 
            * @param io_physic is used to move the actor around the zone
            * @param io_actor is the actor used to move around the zone. The new sectors and portals will be put into it's data.
            * Note that it will create the sectors ans portals with a new and they should be delete in the data structure.
            * @param[in] in_zone the box will be used at it's boarder to know where to start and end the high-level portal.
            * @param in_hLevel the hierarchical level into which the portals needs to be created.
            */
            void                             generateSectorsAndPortals( Physic* io_physic, Actor* io_actor, const Pathfinding::Math::AxisAlignedBox * in_zone, const int in_hLevel );

            /**
            * This function is gonna create the sectors and portals with a floodFill algorithm.  This function is only valid for level 0 in the hierarchy.
            * 
            * @param io_physic is used to move the actor around the zone
            * @param io_actor is the actor used to move around the zone. The new sectors and portals will be put into it's data.
            * Note that it will create the sectors ans portals with a new and they should be delete in the data structure.
            * @param[in] in_zone the box will be used at it's boarder to know where to start and end the high-level portal.
            * @param in_hLevel the hierarchical level into which the portals needs to be created.
            * @param in_pos the position to start the flood fill
            */
            void                             generateSectorsAndPortals( Physic* io_physic, Actor* io_actor, const Pathfinding::Math::AxisAlignedBox * in_zone, const Pathfinding::Math::Vector3 & in_pos );

            /**
            * This function will take the zone at input and check for any portals that could be created with sectors that are at the boarder of the zone.
            * Used for partitions based world. The data will be put in the actor
            * 
            * @param io_physic is used to move the actor around the zone
            * @param io_actor is the actor used to move around the zone. The new portals will be put into it's data.
            * Note that it will create the portals with a new and they should be delete in the data structure.
            * @param[in] in_zone the box will be used at it's boarder to know if there is something at the exterior of that box.
            * @param in_hLevel the hierarchical level into which the portals needs to be created
            */
            void                             generatePortalsAroundZone(  Physic* io_physic, Actor* io_actor, const Pathfinding::Math::AxisAlignedBox * in_zone, const int in_hLevel );


        private:
            /**
            * Add the cell in the data structure
            *
            * @param io_cell the cell to be added into mPathCells
            */
            void                              addPathCell( Cell * io_cell );

            /**
            * Add the cell in the data structure.
            * The need for this function is when a cell is bigger than the other one to complete the box
            *
            * @param io_cell the cell to be added into mPathCells
            * @param in_pos the position of the cell
            */
            void                              addPathCell( Cell * io_cell, const Pathfinding::Math::Vector3 & in_pos );

            /**
            * The function reset all the data to it's initial value.
            * The function should be called before starting to make sure we have proper data.
            */ 
            void                             clearData();

            /**
            * Find the cell in the given direction.  If no cell is found in that direction, a new cell is created.
            *
            * @param[in] in_dir the direction where it's going
            * @param io_step the step to check it's neighbors in the direction given
            */
            Cell*              processPath( int in_dir, Cell* io_step );

            /**
            * This function process the 4 cells connected to the step in the task list and add them back
            * to the list, until there is no more. It generate the connectivity for the steps
            *
            * @param io_firstStep is the first step, from where all the other cells will be connected
            */
            void                               processTaskList( Cell * io_firstStep );

            /**
            * @param in_pos the position to find the cell
            * @return the cell at the position
            */
            Cell*                findCell( const Pathfinding::Math::Vector3 & in_pos );

            /**
            * This function takes the cells that are the boarder of a sector and transform them into one and two ways portals.
            * The portals are added to the sector or both sectors.  The hierarchical level of this function is 0.
            *
            * @param[in] in_dir the direction in which the portal needs to be created
            * @param[in] in_listCell the list of cell that can create a portal in the given direction
            * @param io_startSec the sector where the cells are
            */
            void                               createPortals( const int in_dir, const std::list<Cell*> in_listCell, Pathfinding::Sector * io_startSec );

            /**
            * This function takes the PrePathfindingSector and generate portals for the given Pathfinding::PathfindingSector
            *
            * @param io_preSector contains the list of cell to create the portals
            * @param io_sector contains the sectors to add the portals
            */
            void                               generatePortals( std::list<Sector*> & io_preSector, std::list<Pathfinding::Sector*> & io_sector );

            /**
            * Generate the sectors on the terrain.
            * The function call in a loop all the sectors that can be created.
            *
            * @param[out] out_preSector the sector with all the cells in it
            * @param[out] out_listSector the created sector from the preSector
            */
            void                             generateSectors(std::list<Sector*> & out_preSector, std::list<Pathfinding::Sector*> & out_listSector);

            /**
            * Generate the preSector to get it's largest possible BoundingBox
            * What I've done is a sort of spiral.  To understand this function, take a pen and a paper and draw what I'm telling you.  
            * For the example we will consider you can walk both ways always.
            *
            * Draw one cell in the middle of the paper. Draw one cell north.  Then 2 cells at the east of those cells, 
            * then 2 cells south of the 4 previous.  Then 3 cells at the west of those 6 previous.  
            * Then you start back north, draw 3 cells north ...
            *
            * When you can't continue on a direction with ALL the cells, break the algorithm on that direction.  
            * It could be 100 cells connected and one which is a one way and all others are 2 ways, you still need to break the 
            * algorithm on that direction.  Say it's North, you should continue East South and West until they break too.
            *
            * When all the directions are stopped, you've got your prePathfindingSector with all the cells containing it.
            *
            * @param io_firstCell the first cell of the sector
            * @return the generated PrepathfindingSector
            */
            Sector*            generatePreSector( Cell *io_firstCell );

            /**
            * This function do a check to see if it's possible to add a line of cell in the direction given.
            *
            * @param[in] in_dir the direction in which it's gonna try to make a new line of PrePathfindingCell
            * @param[in] in_actual the actual PrePathfindingCell list of the direction where it's going
            *
            * @return The new list of cell in the given direction
            */
            std::list<Cell*>   sectorDirection( int in_dir, const std::list<Cell*> in_actual );

            /**
            * @param[in] in_pos the position where the actor is gonna try to be set
            * @return the actor has been changed of position
            */
            bool                             setActorPos( const Pathfinding::Math::Vector3 &in_pos );

            /**
            * Generate the sectors and portals next to the input portal.
            * 
            * @param[out] out_nextPortals the portals next to the in_actualPortal
            * @param io_together the portals that are connected together.  Sectors connected will be add in this list
            * @param[in] in_zone the zone in which the portal is going
            * @param[in] in_actualPortal the portal to generate the successors from
            * @param io_portalsInZone the list of all the portals in the list.  The portals found will be removed from this list
            */
            void generateSuccessorsList( std::set<const Pathfinding::Portal*>& out_nextPortals, std::set<const Pathfinding::Sector*> & io_together, const Pathfinding::Math::AxisAlignedBox * in_zone, const Pathfinding::Portal * in_actualPortal, std::vector<const Pathfinding::Portal*> & io_portalsInZone );

            /**
            * Creates an imaginary sector with the box and put it in the two lists.
            * 
            * @param[in] in_box the box to create the imaginary sector.
            * @param[out] out_imaginarySectors the list of imaginary sectors
            * @param[out] out_currentSectors the list of the sectors that are currently being processed
            * @param[in] in_sectorModified the sector that is being split into imaginary sectors
            */
            void createImaginarySector( const Pathfinding::Math::AxisAlignedBox & in_box, std::list<Pathfinding::Sector*> & out_imaginarySectors, std::list<const Pathfinding::Sector*> & out_currentSectors, const Pathfinding::Sector* in_sectorModified );



            /** 
            * Make the flood fill algorithm from the point defined by pos
            * Once the flood fill is finished to get sectors and portals, the function generateSectorsAndPortals should be called
            * The hierarchical level of this function is 0.
            * 
            * @param[in] in_pos is used to tell the position where the flood fill will start
            */
            void                             floodFillPoint( const Pathfinding::Math::Vector3 &in_pos );

            /**
            * Calculate all the possible place to go into this zone.  The hierarchical level of this function is 0.
            * 
            */
            void                             fillAll();

            /**
            * Generate all the sectors and portals from the data calculated in the fillAll or floodFillPoint method
            * Note : This function will call new for portals and sectors, but will not call delete.  The delete must be called on sector suppression in the data structure.
            * The hierarchical level of this function is 0.
            *
            */
            void                             generateSectorsAndPortalsBase();

            /**
            * This function is gonna create the sectors and portals related to the sectors and portals in the lower level of hierarchy.
            * 
            * @param io_physic is used to move the actor around the zone
            * @param io_actor is the actor used to move around the zone. The new sectors and portals will be put into it's data.
            * Note that it will create the portals with a new and they should be delete in the data structure.
            * @param[in] in_zone the box will be used at it's boarder to know where to start and end the high-level portal.
            * @param in_hLevel the hierarchical level into which the portals needs to be created.
            */
            void                             generateHierarchySectorsAndPortals( const int in_hLevel );

        protected:
            /**
            * This function checks whether the sector can continue in this direction.  This function is in this inherited class because the decision
            * must be taken by the designer to know when a sector should stop.  I.E A new surface at that place, too much slope, etc.
            * 
            * @param[in] in_dir the direction in which the sector is being created
            * @param[in] in_sector the actual sector, before adding the line
            * @param[in] in_listCell the list of cells that is going to be added to the sector
            */
            virtual bool verifySectorContinuity( int dir, const Sector * pSector, const std::list<Cell*> listCell );

            /** An abstract method so if a brake needs to be make for any reason in the algorithm and stop the execution,
            * it can be done outside of this class
            *
            * @return true if the algorithm can continue
            */
            virtual bool verifyFloodFillContinuity();

            /**
            If anything needs to be precalculated, it can be done in this method which is call at the start of a flood fill.
            */
            virtual void preCalculate();

            /** 
            This is call once the floodFill or fillAll is completed.
            */
            virtual void fillFinish();

            /// The filling will add the sectors and portals created in this variable
            Data*                                        mData;
            Physic*                mPhysic;
            /// The zone to compute, permit to not make all the pathfinding at once
            const Pathfinding::Math::AxisAlignedBox*     mZone;
            Actor*                                       mActor;
            /**
            * A 2 dimensional array of a linked list of PrePathfindingCell
            * The polygon soup can be represent by this.
            */ 
            typedef std::map<int, std::list<Cell*>>     mMapCell;
            /// This is a map of map of list of cells, used to quickly find the cells
            std::map<int, mMapCell>                    mPathCells;
        };
    }
} // namespace Pathfinding
#endif
