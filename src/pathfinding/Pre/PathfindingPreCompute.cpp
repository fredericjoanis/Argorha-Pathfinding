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

#include "Pre/PathfindingPreCompute.h"
#include "Pre/PathfindingPreCommon.h"
#include "PathfindingSector.h"
#include "PathfindingPortal.h"
#include "Pre/PathfindingPrePhysic.h"
#include "Pre/PathfindingPreCell.h"
#include "PathfindingData.h"
#include "PathfindingAction.h"
#include "Pre/PathfindingPreSector.h"

namespace Pathfinding 
{
    namespace Pre
    {
        //----------------------------------------------------------------------------------------------------------------

        void Compute::generateSectorsAndPortals( Physic* io_physic, Actor* io_actor, const Pathfinding::Math::AxisAlignedBox * in_zone, const int in_hLevel )
        {
            assert( io_actor != NULL && io_physic != NULL && in_zone != NULL && io_actor->getData() != NULL && in_hLevel >= 0 );
            mPhysic  = io_physic;
            mActor   = io_actor;
            mData    = mActor->getData();
            mZone    = in_zone;

            if( in_hLevel == 0 )
            {
                fillAll();
                fillFinish();
                // Make sure we can continue.
                if( verifyFloodFillContinuity() )
                {
                    generateSectorsAndPortalsBase();
                }
            }
            else
            {
                generateHierarchySectorsAndPortals( in_hLevel );
            }
        }

        //----------------------------------------------------------------------------------------------------------------

        void Compute::generateSectorsAndPortals( Physic* io_physic, Actor* io_actor, const Pathfinding::Math::AxisAlignedBox * in_zone, const Pathfinding::Math::Vector3 & in_pos )
        {
            assert( io_actor != NULL && io_physic != NULL && in_zone != NULL && io_actor->getData() != NULL );

            mPhysic  = io_physic;
            mActor   = io_actor;
            mData    = mActor->getData();
            mZone    = in_zone;

            floodFillPoint( in_pos );
            fillFinish();

            if( verifyFloodFillContinuity() )
            {
                generateSectorsAndPortalsBase();
            }
        }

        //----------------------------------------------------------------------------------------------------------------
        void Compute::generatePortalsAroundZone( Physic* io_physic, Actor* io_actor, const Pathfinding::Math::AxisAlignedBox * in_zone, const int in_hLevel )
        {
            assert( io_physic != NULL && io_actor != NULL && io_actor->getData() != NULL && in_zone != NULL && in_hLevel >= 0 );

            typedef std::map<Pathfinding::Sector*,Pathfinding::Sector*>  tdMapSector;
            typedef std::map<Pathfinding::Sector*,int>                 tdMapSectorNbWays;
            typedef std::map<Pathfinding::Sector*,Portal*>  tdMapPortalInCreation;
            typedef std::map<Pathfinding::Sector*,bool>                tdMapSectorContinue;

            // Some precalculations
            float smallVectorX   = io_actor->getSmallVector().x;
            float smallVectorZ   = io_actor->getSmallVector().z;

            float zoneMinX       = in_zone->getMinimum().x;
            float zoneMinZ       = in_zone->getMinimum().z;
            float zoneMaxX       = in_zone->getMaximum().x;
            float zoneMaxZ       = in_zone->getMaximum().z;

            Pathfinding::Math::Vector3 v3Max = io_actor->getSmallVector();
            Pathfinding::Math::Vector3 v3MaxOn2 = v3Max * 0.5f;

            float posStart = 0.f;
			float posEnd = 0.f;
			float smallVector = 0.f;

            // Go trough the directions around the zone
            for( unsigned int dir = Common::FIRST_DIRECTION; dir <= Common::LAST_DIRECTION; dir++ )
            {
                switch( dir )
                {
                case Common::NORTH:
                case Common::SOUTH:
                    posStart = zoneMinX + v3MaxOn2.x;
                    posEnd = zoneMaxX;
                    smallVector = smallVectorX;
                    break;
                case Common::EAST:
                case Common::WEST:
                    posStart = zoneMinZ + v3MaxOn2.z;
                    posEnd = zoneMaxZ;
                    smallVector = smallVectorZ;
                    break;
                }

                tdMapSector mapSector;
                tdMapSectorNbWays mapSectorNbWays;
                tdMapPortalInCreation mapPortal;
                tdMapSectorContinue mapContinue;
                tdMapSectorContinue mapReset;

                for( Pathfinding::Math::Real pos = posStart; pos <= posEnd; pos += smallVector )
                {
                    Pathfinding::Sector* sector1 = NULL;
                    Pathfinding::Sector* sector2 = NULL;
                    Pathfinding::Math::Vector3 disp;

                    // Set the displacement to check
                    switch( dir )
                    {
                    case Common::NORTH:
                        io_actor->setPosition( Pathfinding::Math::Vector3( pos, 0.f, zoneMaxZ - v3MaxOn2.z ) );
                        disp = Pathfinding::Math::Vector3( 0, 0, v3Max.z );
                        break;
                    case Common::SOUTH:
                        io_actor->setPosition( Pathfinding::Math::Vector3( pos, 0.f, zoneMinZ + v3MaxOn2.z ) );
                        disp = Pathfinding::Math::Vector3( 0, 0, -v3Max.z );
                        break;
                    case Common::EAST:
                        io_actor->setPosition( Pathfinding::Math::Vector3( zoneMaxX - v3MaxOn2.x, 0.f, pos ) );
                        disp = Pathfinding::Math::Vector3( v3Max.x, 0, 0 );
                        break;
                    case Common::WEST:
                        io_actor->setPosition( Pathfinding::Math::Vector3( zoneMinX + v3MaxOn2.x, 0.f, pos ) );
                        disp = Pathfinding::Math::Vector3( -v3Max.x, 0, 0 );
                        break;
                    }

                    // Reset the data to check the continuity of the sector
                    for( tdMapSectorContinue::iterator iterReset = mapReset.begin(); iterReset != mapReset.end(); iterReset++ )
                    {
                        iterReset->second = true;
                    }

                    std::list<Pathfinding::Math::Real> heights = io_physic->getStandOnHeights( io_actor );

                    for( std::list<Pathfinding::Math::Real>::iterator iterHeights = heights.begin(); iterHeights != heights.end(); iterHeights++ )
                    {
                        int nbDirection = 0;
                        sector2 = NULL;
                        Pathfinding::Math::Vector3 initialPos = io_actor->getPosition();
                        initialPos.y = *iterHeights;
                        io_actor->setPosition( initialPos );
                        // Tricky thing because the actor for the physic is positionned at the feet level, and the find for the sector is made for the mid sector.
                        initialPos.y += io_actor->getSmallVector().y * 0.5f;
                        sector1 = mData->getSectorAtPos( initialPos, in_hLevel );
                        
                        // Sector one should always have a data in this case.
                        //assert( sector1 != NULL );

                        // We won't need to reset this portals since there is a continuity
                        tdMapSectorContinue::iterator iterReset = mapReset.find( sector1 );
                        if( iterReset == mapReset.end() )
                        {
                            mapReset.insert( tdMapSectorContinue::value_type( sector1, false ));
                        }
                        else
                        {
                            iterReset->second = false;
                        }

                        // Try the bottom of the cell if there is a cell
                        Pathfinding::Math::Real finalHeight;
                        Pathfinding::Math::Real nbIteration = io_physic->move( disp, io_actor, finalHeight );
                        Pathfinding::Math::Vector3 finalPos = io_actor->getPosition() + disp;
                        finalPos.y = finalHeight + io_actor->getSmallVector().y * 0.5f;

                        if( nbIteration > 0 )
                        {
                            nbDirection++;
                            sector2 = mData->getSectorAtPos( finalPos, in_hLevel );
                            nbIteration = io_physic->move( -disp, io_actor, finalHeight );
                            // No need to have another final height since if you go one way and come back, you should have come back to the same point.
                            // Might be a problem here, if the actor fall then go back under it's previous place?!
                            if( nbIteration > 0 )
                            {
                                nbDirection++;
                            }
                        }

                        // Make sure there is portals to move
                        if( sector1 != NULL && sector2 != NULL && sector1 != sector2 )
                        {
                            // Get the sector associated to this sector.  This is a map since there can be multiples portals onto one position
                            tdMapSector::iterator iterSector = mapSector.find( sector1 );
                            tdMapSectorNbWays::iterator iterNbWays = mapSectorNbWays.find( sector1 );
                            tdMapSectorContinue::iterator iterContinu = mapContinue.find( sector1 );
                            tdMapPortalInCreation::iterator iterPortal = mapPortal.find( sector1 );
                            // The portal that was being created needs to be completed
                            if( iterContinu == mapContinue.end() || iterSector == mapSector.end() || iterNbWays == mapSectorNbWays.end() || iterSector->second != sector2 || iterNbWays->second != nbDirection  || !iterContinu->second )
                            {
                                // A new portal need to be created
                                Portal* portal = new Portal();

                                if( iterSector == mapSector.end() )
                                {
                                    mapSector.insert( tdMapSector::value_type( sector1, sector2 ) );
                                    mapSectorNbWays.insert( tdMapSectorNbWays::value_type( sector1, nbDirection ) );
                                    mapPortal.insert( tdMapPortalInCreation::value_type( sector1, portal ));
                                    mapContinue.insert( tdMapSectorContinue::value_type( sector1, true ));
                                }
                                else
                                {
                                    iterSector->second = sector2;
                                    iterNbWays->second = nbDirection;
                                    iterPortal->second = portal;
                                    iterContinu->second = true;
                                }

                                Pathfinding::Math::Vector3 vecStart = (*iterHeights) - v3MaxOn2;
                                Pathfinding::Math::Vector3 vecEnd = (*iterHeights) + disp + v3MaxOn2;

                                Pathfinding::Math::AxisAlignedBox portalBox;

                                // Find the length of the box.
                                switch( dir )
                                {
                                case Common::NORTH:
                                    portalBox.setExtents( Pathfinding::Math::Vector3( vecStart.x, vecStart.y, zoneMaxZ - smallVectorZ ), Pathfinding::Math::Vector3( vecEnd.x, vecEnd.y, zoneMaxZ + smallVectorZ ) );
                                    break;
                                case Common::SOUTH:
                                    portalBox.setExtents( Pathfinding::Math::Vector3( vecStart.x, vecStart.y, zoneMinZ - smallVectorZ ), Pathfinding::Math::Vector3( vecEnd.x, vecEnd.y, zoneMinZ + smallVectorZ ) );
                                    break;
                                case Common::EAST:
                                    portalBox.setExtents( Pathfinding::Math::Vector3( zoneMaxX - smallVectorX, vecStart.y, vecStart.z ), Pathfinding::Math::Vector3( zoneMaxX + smallVectorX, vecEnd.y, vecEnd.z ) );
                                    break;
                                case Common::WEST:
                                    portalBox.setExtents( Pathfinding::Math::Vector3( zoneMinX - smallVectorX, vecStart.y, vecStart.z ), Pathfinding::Math::Vector3( zoneMinX + smallVectorX, vecEnd.y, vecEnd.z ) );
                                    break;
                                }

                                portal->setBox( portalBox );

                                io_actor->getData()->addPortal( portal, in_hLevel );

                                // Set the portal to the sector and vice-versa
                                portal->setDestSector1( sector1 );
                                sector1->addPortal( portal );

                                portal->setDestSector2( sector2 );
                                if( nbDirection > 1 )
                                {
                                    // The sector is bidirectional, so we add it in the 2 sectors.
                                    sector2->addPortal( portal );
                                }
                            }
                            else
                            {
                                // Else the portal already exist, but can be bigger
                                tdMapPortalInCreation::iterator iterPortal = mapPortal.find( sector1 );
                                Pathfinding::Math::AxisAlignedBox portalBox = iterPortal->second->getBox();

                                Pathfinding::Math::Vector3 posHeight = initialPos;

                                // It's the last one, extend it
                                if( pos + smallVector > posEnd )
                                {
                                    switch( dir )
                                    {
                                    case Common::NORTH:
                                        posHeight.x = posEnd - v3MaxOn2.x;
                                        break;
                                    case Common::SOUTH:
                                        posHeight.x = posEnd - v3MaxOn2.x;
                                        break;
                                    case Common::EAST:
                                        posHeight.z = posEnd - v3MaxOn2.z;
                                        break;
                                    case Common::WEST:
                                        posHeight.z = posEnd - v3MaxOn2.z;
                                        break;
                                    }
                                }

                                portalBox.merge( posHeight + v3MaxOn2 );
                                iterPortal->second->setBox( portalBox );
                            }
                        }
                        else
                        {
                            // There is something wrong, stop processing
                            tdMapSectorContinue::iterator iterContinu = mapContinue.find( sector1 );
                            if( iterContinu == mapContinue.end() )
                            {
                                // The algorithm has not been passed this time, next iteration it needs to be reset
                                mapContinue.insert( tdMapSectorContinue::value_type( sector1, false ));
                            }
                            else
                            {
                                iterContinu->second = false;
                            }
                        }
                    }
                    // Check which portals needs to be reset
                    for( tdMapSectorContinue::iterator iterReset = mapReset.begin(); iterReset != mapReset.end(); iterReset++ )
                    {
                        if( iterReset->second )
                        {
                            tdMapSectorContinue::iterator iterContinu = mapContinue.find( iterReset->first );
                            if( iterContinu == mapContinue.end() )
                            {
                                // The algorithm has not been passed this time, next iteration it needs to be reset
                                mapContinue.insert( tdMapSectorContinue::value_type( iterReset->first, false ));
                            }
                            else
                            {
                                iterContinu->second = false;
                            }
                        }
                    }
                }
            }
        }

        //----------------------------------------------------------------------------------------------------------------

        void Compute::floodFillPoint( const Pathfinding::Math::Vector3 &in_pos )
        {
            // Set theses variables in the class for cleanness of code instead of passing those arguments everywhere.      
            mActor->setPosition( in_pos );

            preCalculate();
            clearData();
            mPhysic->prepare( mActor );

            if( mPhysic->canStandOn( mActor ) && mZone->intersects( in_pos ) )
            {
                setActorPos( in_pos );

                Pathfinding::Math::Vector3 v3Max = mActor->getSmallVector() * 0.5f;
				Pathfinding::Math::AxisAlignedBox actorPos(mActor->getPosition() - v3Max, mActor->getPosition() + v3Max);
				Pathfinding::Pre::Cell * cell = new Cell(actorPos);

                addPathCell( cell );

                processTaskList( cell );
            }
        }

        //----------------------------------------------------------------------------------------------------------------

        void Compute::fillAll()
        {
            mActor->setPosition( mZone->getCenter() );

            preCalculate();
            clearData();
            mPhysic->prepare( mActor );

            Pathfinding::Math::Vector3 v3Max = mActor->getSmallVector();
            Pathfinding::Math::Vector3 v3MaxOn2 = v3Max * 0.5f;

            Pathfinding::Math::Vector3 v3MaxOffset = Pathfinding::Math::Vector3::ZERO;

            float smallVectorX   = mActor->getSmallVector().x;
            float smallVectorZ   = mActor->getSmallVector().z;
            float zoneMinX       = mZone->getMinimum().x;
            float zoneMinZ       = mZone->getMinimum().z;
            float zoneMaxX       = mZone->getMaximum().x;
            float zoneMaxZ       = mZone->getMaximum().z;


            // Iterate to cover all the possibles places in x and z with respect to the small box
            for( float rowPos = zoneMinX; rowPos < zoneMaxX - smallVectorX && verifyFloodFillContinuity(); rowPos += smallVectorX )
            {
                // The cells on the boarder are a bit bigger to fit the zone length.
                if( rowPos + smallVectorX <= zoneMaxX - smallVectorX )
                {
                    v3MaxOffset.x = 0.f;
                }
                else
                {
                    v3MaxOffset.x = zoneMaxX - rowPos - smallVectorX;
                }

                for( float linePos = zoneMinZ; linePos < zoneMaxZ - smallVectorZ; linePos += smallVectorZ )
                {
                    // The cells on the boarder are a bit bigger to fit the zone length.
                    if( linePos + smallVectorZ <= zoneMaxZ - smallVectorZ )
                    {
                        v3MaxOffset.z = 0.f;
                    }
                    else
                    {
                        v3MaxOffset.z = zoneMaxZ - linePos - smallVectorZ;
                    }

                    Pathfinding::Math::Vector3 initialPos( rowPos + v3MaxOn2.x, 0.f, linePos + v3MaxOn2.z );

                    mActor->setPosition( initialPos );

                    // Create the box with the actor and add it
                    std::list<Pathfinding::Math::Real> heights = mPhysic->getStandOnHeights( mActor );

                    for( std::list<Pathfinding::Math::Real>::iterator iter = heights.begin(); iter != heights.end(); iter++ )
                    {
                        initialPos.y = *iter + mActor->getSmallVector().y * 0.5f;
						Pathfinding::Math::AxisAlignedBox actorPos(initialPos - v3MaxOn2, initialPos + v3MaxOn2 + v3MaxOffset);
						Cell* cellNew = new Cell(actorPos);

                        // if the new algorithm works well, that boolean should be removed
                        cellNew->setProcessed( true );

                        // Put the box at the place it should be to find it back.
                        addPathCell( cellNew, initialPos );
                    }
                }
            }

            // Iterate to make links between all the boxes
            for( std::map<int, mMapCell>::iterator lineIter = mPathCells.begin(); lineIter != mPathCells.end(); ++lineIter  )
            {
                for( mMapCell::iterator colIter = (*lineIter).second.begin(); colIter != (*lineIter).second.end(); ++colIter  )
                {
                    for( std::list<Cell*>::iterator listIter = (*colIter).second.begin(); listIter != (*colIter).second.end(); ++listIter  )
                    {
                        // Create the box with the actor and add it

                        int nbIter = 0;
                        Pathfinding::Math::Real cellCost = 0.f;
                        Pathfinding::Math::Vector3 disp;
                        for( int dir = Common::FIRST_DIRECTION; dir <= Common::LAST_DIRECTION; dir++ )
                        {
                            mActor->setPosition( (*listIter)->getBox().getMinimum() + v3MaxOn2 );

                            switch( dir )
                            {
                            case Common::NORTH:
                                disp = Pathfinding::Math::Vector3( 0, 0, -v3Max.z );
                                break;
                            case Common::SOUTH:
                                disp = Pathfinding::Math::Vector3( 0, 0, v3Max.z );
                                break;
                            case Common::EAST:
                                disp = Pathfinding::Math::Vector3( v3Max.x, 0, 0 );
                                break;
                            case Common::WEST:
                                disp = Pathfinding::Math::Vector3( -v3Max.x, 0, 0 );
                                break;
                            }

                            // Try the bottom of the cell if there is a cell
                            Pathfinding::Math::Vector3 movedPos = mActor->getPosition() + disp;
                            Pathfinding::Math::Real nbIteration = mPhysic->move( disp, mActor, movedPos.y );
                            // The returned height of move is the actor feet and it's the center of the cell which is needed.
                            movedPos.y += mActor->getSmallVector().y * 0.5f;

                            if( nbIteration > 0 )
                            {
                                nbIter++;
                                cellCost += nbIteration;
                                Cell* cellConnected = findCell( movedPos );

                                if( cellConnected )
                                {
                                    // Should not happen, if that happen, there will be an infinite loop in sectors generating.
                                    assert( cellConnected != (*listIter) );

                                    switch( dir )
                                    {
                                    case Common::NORTH:
                                        (*listIter)->setNorthStep( cellConnected );
                                        break;
                                    case Common::SOUTH:
                                        (*listIter)->setSouthStep( cellConnected );
                                        break;
                                    case Common::EAST:
                                        (*listIter)->setEastStep( cellConnected );
                                        break;
                                    case Common::WEST:
                                        (*listIter)->setWestStep( cellConnected );
                                        break;
                                    }
                                }
                            }
                        }

                        // Calculate the cost of the cell
                        if( nbIter != 0 )
                        {
                            cellCost /= nbIter;
                            (*listIter)->setCost( nbIter );
                        }
                    }
                }
            }
        }

        //----------------------------------------------------------------------------------------------------------------
        void Compute::generateHierarchySectorsAndPortals( const int in_hLevel )
        {
            assert( in_hLevel > 0 );
            std::vector<const Pathfinding::Portal*> portalsInZone = mActor->getData()->getPortalsInZone( *mZone, in_hLevel - 1 );
            std::list<std::list<const Pathfinding::Sector*>> sectorsSplitted;

            // Sort the sectors in list that are portals connected to each other within the box
            while( !portalsInZone.empty() )
            {
                // Always take the first element and find all the neighbors from this one
                const Pathfinding::Portal* firstPortal = *portalsInZone.begin();

                std::set<const Pathfinding::Sector*> sectorsTogether;
                std::set<const Pathfinding::Portal*> nextPortals;

                nextPortals.insert( firstPortal );

                // Find the neighbors in this loop
                while( !nextPortals.empty() )
                {
                    /* TODO: There might be an error if there is a just one way portal after a two way portals ... to be checked
                    because this mean we can go on one way, but cannot go back.
                    */
                    const Pathfinding::Portal* actualPortal = *nextPortals.begin();
                    generateSuccessorsList( nextPortals, sectorsTogether, mZone, actualPortal, portalsInZone );

                    // Remove the first element
                    nextPortals.erase( nextPortals.begin() );
                }

                std::list<const Pathfinding::Sector*> listTogether;
                // Make it as a list
                listTogether.insert( listTogether.begin(), sectorsTogether.begin(), sectorsTogether.end() );
                sectorsSplitted.push_back( listTogether );
            }

            // Temporary sectors that are created when a division is made and the low-level sector is being cut by a high-level sector
            std::list<Pathfinding::Sector*> imaginarySectors;
            std::list<Pathfinding::Sector*> highLevelSector;

            // From here the sectors are separated into distinct lists
            for( std::list<std::list<const Pathfinding::Sector*>>::iterator iterList = sectorsSplitted.begin(); iterList != sectorsSplitted.end(); iterList++ )
            {
                // Iterate trough each sectors of a given list.  That list is each sectors that are connected together within the zone
                while( iterList->size() > 0 )
                {
                    const Pathfinding::Sector* actualSector = *(iterList->begin());
                    iterList->remove(actualSector);

                    Pathfinding::Sector* sectorInCreation = new Pathfinding::Sector();
                    sectorInCreation->setBox( actualSector->getBox() );

                    for( std::list<const Pathfinding::Sector*>::iterator iterSector = iterList->begin(); iterSector != iterList->end(); )
                    {
                        const Pathfinding::Sector* sectorChecked = *iterSector;
                        bool canBeAdded = true;
                        // If the sector is already in the higher level sector, no need to do any computation
                        if( !sectorInCreation->getBox().contains( sectorChecked->getBox() ) )
                        {
                            // The box to test if that box can be set as the new box for the sectorInCreation
                            Pathfinding::Math::AxisAlignedBox testBox = sectorInCreation->getBox();

                            testBox.merge( sectorChecked->getBox() );

                            // Check the others list of sectors for collision.  If there is a collision, that means that the testBox
                            // is too large and is going over another sector that's going to be created later.
                            std::list<std::list<const Pathfinding::Sector*>>::iterator iterListAfter = iterList;
                            iterListAfter++;
                            while( iterListAfter != sectorsSplitted.end() && canBeAdded )
                            {
                                for( std::list<const Pathfinding::Sector*>::iterator iterSectorAfter = iterListAfter->begin(); iterSectorAfter != iterListAfter->end() && canBeAdded; iterSectorAfter++ )
                                {
                                    if( (*iterSectorAfter)->getBox().intersectsNotEquals( testBox ) )
                                    {
                                        // A sector into another package 
                                        canBeAdded = false;
                                    }
                                }
                                iterListAfter++;
                            }

                            // Now that we know that it's not intersecting a box into another list, check if the testBox is not intersecting a high-level box already created
                            for( std::list<Pathfinding::Sector*>::iterator iterHighSector = highLevelSector.begin(); iterHighSector != highLevelSector.end() && canBeAdded; iterHighSector++ )
                            {
                                if( (*iterHighSector)->getBox().intersectsNotEquals( testBox ) )
                                {
                                    // A sector into another package 
                                    canBeAdded = false;
                                }
                            }
                            if(canBeAdded)
                            {
                                sectorInCreation->setBox( testBox );
                            }
                        }
                        // Move the iterator before having a chance to delete the element from the list
                        iterSector++;
                        if( canBeAdded )
                        {
                            // The box can be removed from the list, it has been added to the actual sector.
                            iterList->remove( sectorChecked );
                        }
                        else if( sectorInCreation->getBox().intersectsNotEquals( sectorChecked->getBox() ) )
                        {
                            /* Here there is a problem, since the sector tested is intersecting the higher-level box and it's not possible
                            to make a bigger box with that sector.
                            It is needed to subdivide this sector into smaller sector for the calculating of the next box
                            Those new sectors are somewhat imaginary sectors since they will never really exist
                            */
                            iterList->remove( sectorChecked );
                            Pathfinding::Math::AxisAlignedBox intersectionBox = sectorInCreation->getBox().intersection( sectorChecked->getBox() );

                            // Precalculate and less long to type.
                            Pathfinding::Math::Vector3 aMin = intersectionBox.getMinimum();
                            Pathfinding::Math::Vector3 bMin = sectorChecked->getBox().getMinimum();

                            Pathfinding::Math::Vector3 aMax = intersectionBox.getMaximum();
                            Pathfinding::Math::Vector3 bMax = sectorChecked->getBox().getMaximum();

                            // With an intersection, there is 4 possibles box.
                            if( aMin.z > bMin.z )
                            {
                                Pathfinding::Math::AxisAlignedBox boxSector( bMin, Pathfinding::Math::Vector3( bMax.x, bMax.y, aMin.z ) );
                                createImaginarySector( boxSector, imaginarySectors, *iterList, sectorChecked );
                            }

                            if( aMin.x > bMin.x )
                            {
                                Pathfinding::Math::AxisAlignedBox boxSector( Pathfinding::Math::Vector3( bMin.x, bMin.y, aMin.z ), Pathfinding::Math::Vector3( aMin.x, bMax.y, aMax.z ) );
                                createImaginarySector( boxSector, imaginarySectors, *iterList, sectorChecked );
                            }

                            if( aMax.x < bMax.x )
                            {
                                Pathfinding::Math::AxisAlignedBox boxSector( Pathfinding::Math::Vector3( aMax.x, bMin.y, aMin.z ), Pathfinding::Math::Vector3( bMax.x, bMax.y, aMax.z ) );

                                createImaginarySector( boxSector, imaginarySectors, *iterList, sectorChecked );
                            }

                            if( aMax.z < bMax.z )
                            {
                                Pathfinding::Math::AxisAlignedBox boxSector( Pathfinding::Math::Vector3( bMin.x, bMin.y, aMax.z ), bMax );

                                createImaginarySector( boxSector, imaginarySectors, *iterList, sectorChecked );
                            }
                        }
                    }

                    /* Finally, after iterating everywhere, the sector is the biggest that it can.  Make sure that the sectorInCreation is within
                    the zone.
                    */
                    sectorInCreation->setBox( mZone->intersection( sectorInCreation->getBox() ) );

                    //Add it to the actor.
                    mActor->getData()->addSector( sectorInCreation, in_hLevel );

                    // That done, portals must be added.
                    generatePortalsAroundZone( mPhysic, mActor, &sectorInCreation->getBox(), in_hLevel );

                    highLevelSector.push_back( sectorInCreation );
                }
            }

            // Release the memory
            for( std::list<Pathfinding::Sector*>::iterator iterImaginary = imaginarySectors.begin(); iterImaginary != imaginarySectors.end();  )
            {
                Pathfinding::Sector* secToDelete = *iterImaginary;
                iterImaginary++;
                delete secToDelete;
            }
        }

        void Compute::createImaginarySector( const Pathfinding::Math::AxisAlignedBox & in_box, std::list<Pathfinding::Sector*> & out_imaginarySectors, std::list<const Pathfinding::Sector*> & out_currentSectors, const Pathfinding::Sector* in_sectorModified )
        {
            // Keep all info of the previous sector
            Pathfinding::Sector* secImaginary = new Pathfinding::Sector( *in_sectorModified );
            // Set the new box
            secImaginary->setBox( in_box );

            out_imaginarySectors.push_back( secImaginary );
            // Add it as any other sector in the elements
            out_currentSectors.push_back( secImaginary );
        }

        void Compute::generateSuccessorsList( std::set<const Pathfinding::Portal*>& out_nextPortals, std::set<const Pathfinding::Sector*> & io_together, const Pathfinding::Math::AxisAlignedBox * in_zone, const Pathfinding::Portal * in_actualPortal, std::vector<const Pathfinding::Portal*> & io_portalsInZone )
        {
            std::list<const Pathfinding::Portal*> potPortals;
            potPortals.push_back( in_actualPortal );

            if( in_zone->intersectsNotEquals( in_actualPortal->getDestSector1()->getBox() ) && ( io_together.find( in_actualPortal->getDestSector1() ) == io_together.end() ) )
            {
                potPortals.insert( potPortals.begin(), in_actualPortal->getDestSector1()->getPortals().begin(), in_actualPortal->getDestSector1()->getPortals().end()  );
                io_together.insert( in_actualPortal->getDestSector1() );
            }
            if( in_actualPortal->getDestSector2() && in_zone->intersectsNotEquals( in_actualPortal->getDestSector2()->getBox() ) && ( io_together.find( in_actualPortal->getDestSector2() ) == io_together.end() ) )
            {
                potPortals.insert( potPortals.end(), in_actualPortal->getDestSector2()->getPortals().begin(), in_actualPortal->getDestSector2()->getPortals().end() );
                io_together.insert( in_actualPortal->getDestSector2() );
            }

            for( std::list<const Pathfinding::Portal*>::const_iterator iterPortal = potPortals.begin(); iterPortal != potPortals.end(); ++iterPortal ) 
            {
                bool bIsInList = false;
                std::vector<const Pathfinding::Portal*>::iterator iterZonePortal = io_portalsInZone.begin();
                while( iterZonePortal != io_portalsInZone.end() && !bIsInList )
                {
                    if( *iterZonePortal == *iterPortal )
                    {
                        bIsInList = true;
                    }
                    else
                    {
                        iterZonePortal++;
                    }
                }
                // Conditions to know if there must be a successors or not. Check if the portal is not the current and the portal is in the zone
                if( bIsInList )
                {
                    // The node is not inserted yet.  Remove it
                    io_portalsInZone.erase( iterZonePortal );
                    if( in_actualPortal != (*iterPortal) )
                    {
                        out_nextPortals.insert( *iterPortal );
                    }
                }
            }
        }


        //----------------------------------------------------------------------------------------------------------------

        void Compute::clearData()
        {
            for( std::map<int, mMapCell>::iterator lineIter = mPathCells.begin(); lineIter != mPathCells.end(); ++lineIter  )
            {
                for( mMapCell::iterator colIter = (*lineIter).second.begin(); colIter != (*lineIter).second.end(); ++colIter  )
                {
                    for( std::list<Cell*>::iterator listIter = (*colIter).second.begin(); listIter != (*colIter).second.end(); ++listIter  )
                    {
                        if( (*listIter) != NULL )
                        {
                            delete (*listIter);
                        }
                    }
                    colIter->second.clear();
                }
                (*lineIter).second.clear();
            }
            mPathCells.clear();
        }

        //----------------------------------------------------------------------------------------------------------------

        void Compute::generateSectorsAndPortalsBase()
        {
            // set the data and reset others parameters
            std::list<Sector*> listPreSector;
            std::list<Pathfinding::Sector*> listSector;
            generateSectors(listPreSector, listSector);

            for( std::list<Pathfinding::Sector*>::iterator iter = listSector.begin(); iter != listSector.end(); iter++ )
            {
                mData->addSector( (*iter), Common::BASE_LEVEL );
            }

            generatePortals(listPreSector, listSector);

            clearData();
        }

        //----------------------------------------------------------------------------------------------------------------

        void Compute::processTaskList( Cell * firstStep )
        {
            std::list<Cell*> taskList;
            taskList.push_front( firstStep );

            // START OF TIME CRITICAL CODE
            while( taskList.size() > 0 && verifyFloodFillContinuity() )
            {
                Cell* step = taskList.front();
                // Next two lines are to make sure that a step won't be processed 2 times
                if( !step->getProcessed() )
                {
                    step->setProcessed( true );

                    int nbDirection = 0;

                    for( int dirInd = Common::FIRST_DIRECTION; dirInd <= Common::LAST_DIRECTION; dirInd++ )
                    {
                        Cell *stepExecution = processPath( dirInd, step );
                        if( stepExecution != NULL )
                        {
                            nbDirection++;
                            if( !stepExecution->getProcessed() )
                            {
                                taskList.push_back( stepExecution );
                            }
                        }
                    }

                    // Make the average of the sector cost
                    if( nbDirection != 0 )
                    {
                        step->setCost( step->getCost() / nbDirection );
                    }
                }
                taskList.pop_front();
            }
            // END OF TIME CRITICAL CODE
        }

        //----------------------------------------------------------------------------------------------------------------

        void Compute::addPathCell( Pathfinding::Pre::Cell * cell )
        {
            if( mZone->intersects( cell->getBox() ) )
            {
                addPathCell( cell, cell->getBox().getCenter() );
            }
        }

        //----------------------------------------------------------------------------------------------------------------

        void Compute::addPathCell( Cell * cell, const Pathfinding::Math::Vector3 & pos )
        {
            if( mZone->intersects( pos ) )
            {
                int x = static_cast<int>( pos.x - mZone->getMinimum().x );
                int z = static_cast<int>( pos.z - mZone->getMinimum().z );

                mPathCells[x][z].push_back( cell );
            }
        }

        //----------------------------------------------------------------------------------------------------------------

        Cell* Compute::findCell( const Pathfinding::Math::Vector3 & pos )
        {
            Cell *ret = NULL;
            if( mZone->intersects( pos ) )
            {
                int x = static_cast<int>( pos.x - mZone->getMinimum().x );
                int z = static_cast<int>( pos.z - mZone->getMinimum().z );

                for( std::list<Cell*>::iterator iter = mPathCells[x][z].begin(); iter != mPathCells[x][z].end() && ret == NULL; iter++ )
                {
                    if( (*iter)->getBox().intersects( pos ) )
                    {
                        // The cell has been found
                        ret = *iter;
                    }
                }
            }
            return ret;
        }

        //----------------------------------------------------------------------------------------------------------------

        bool Compute::setActorPos( const Pathfinding::Math::Vector3 &in_pos )
        {
            bool ret = false;
            Pathfinding::Math::Vector3 posBefore = mActor->getPosition();
            mActor->setPosition( in_pos );

            if( mPhysic->canStandOn( mActor ) )
            {
                mActor->setPosition( in_pos );
                ret = true;
            }
            else
            {
                mActor->setPosition( posBefore );
            }

            return ret;
        }

        //----------------------------------------------------------------------------------------------------------------

        Cell* Compute::processPath( int dir, Cell* step )
        {
            Pathfinding::Math::Vector3 centerPos = step->getBox().getCenter();

            if( !setActorPos( centerPos ) ) { return NULL; }

            Cell *pathRet = NULL; // The cell where it's moving
            int nbIteration = 0;

            Pathfinding::Math::Vector3 disp; // The displacement made

            switch( dir )
            {
            case Common::NORTH:
                disp = Pathfinding::Math::Vector3( 0, 0, -mActor->getSmallVector().z );
                break;
            case Common::SOUTH:
                disp = Pathfinding::Math::Vector3( 0, 0, mActor->getSmallVector().z );
                break;
            case Common::EAST:
                disp = Pathfinding::Math::Vector3( mActor->getSmallVector().x, 0, 0 );
                break;
            case Common::WEST:
                disp = Pathfinding::Math::Vector3( -mActor->getSmallVector().x, 0, 0 );
                break;
            }

            // The height after the movement happened.
            centerPos += disp;
            nbIteration = mPhysic->move( disp, mActor, centerPos.y );
            // The returned height of move is the actor feet and it's the center of the cell which is needed.
            centerPos.y += mActor->getSmallVector().y * 0.5f;

            //Make sure that the flood fill is in the filling zone
            if( nbIteration > 0 && mZone->intersects( centerPos ) )
            {
                // See if the cell exist
                pathRet = findCell( centerPos );
                // The cell does not exist, create it
                if( !pathRet )
                {
                    Pathfinding::Math::Vector3 v3MaxOn2 = mActor->getSmallVector() * 0.5f;
                    // Add a new pathfinding Cell at the position of the actor, of the size of the small box
					Pathfinding::Math::AxisAlignedBox pos(centerPos - v3MaxOn2, centerPos + v3MaxOn2);
					pathRet = new Cell(pos);
                    addPathCell( pathRet );
                }

                step->setCost( step->getCost() + nbIteration );

                switch( dir )
                {
                case Common::NORTH:
                    step->setNorthStep( pathRet );
                    break;
                case Common::SOUTH:
                    step->setSouthStep( pathRet );
                    break;
                case Common::EAST:
                    step->setEastStep( pathRet );
                    break;
                case Common::WEST:
                    step->setWestStep( pathRet );
                    break;
                }
            }
            return pathRet;
        }

        //----------------------------------------------------------------------------------------------------------------

        void Compute::generateSectors(std::list<Sector*> & out_preSector, std::list<Pathfinding::Sector*> & out_listSector)
        {
            //Pass trough each cell
            for( std::map<int, mMapCell>::iterator lineIter = mPathCells.begin(); lineIter != mPathCells.end(); ++lineIter  )
            {
                for( mMapCell::iterator colIter = (*lineIter).second.begin(); colIter != (*lineIter).second.end(); ++colIter  )
                {
                    for( std::list<Cell*>::iterator listIter = (*colIter).second.begin(); listIter != (*colIter).second.end(); ++listIter  )
                    {
                        // If the cell has been processed and is not in the a zone, process it
                        if( (*listIter)->getProcessed() && !(*listIter)->getInZone() )
                        {
                            // Put the results of the generatePreSector function in the return list
                            // This creates all the pre sectors.
                            out_preSector.push_front( generatePreSector( *listIter ) );
                        }
                    }
                }
            }

            // Once all the presectors are created, take the preSector list and make them as sectors
            for( std::list<Sector*>::iterator iter = out_preSector.begin(); iter != out_preSector.end(); iter++ )
            {
                Pathfinding::Sector* sector = new Pathfinding::Sector();
                sector->setCost( (*iter)->getAverageCost() );
                sector->setBox( (*iter)->getBox() );
                out_listSector.push_back( sector );
            }
        }

        //----------------------------------------------------------------------------------------------------------------

        /* generateSectors
        * PrePathfindingCell *firstCell is the first step to create the sector.
        */
        Sector* Compute::generatePreSector( Cell *firstCell )
        {
            Sector* pSector = new Sector( firstCell );

            bool bContinue = true;

            // Set the booleans for more directions to true.  When we can't continue on one side, the variable will be set to false
            // And won't continue on that direction
            std::vector<bool> vec;
            for( unsigned int i = Common::FIRST_DIRECTION; i <= Common::LAST_DIRECTION; i++ )
            {
                vec.push_back( true );
            }

            while( bContinue )
            {
                bContinue = false;
                // Check to see if there is another direction to go see, if it exit before the end of the iterator, there is another direction to see
                for( std::vector<bool>::iterator iter = vec.begin(); iter != vec.end() && bContinue == false; iter++ )
                {
                    bContinue = (*iter);
                }

                // There is still a direction to see, continue
                if( bContinue )
                {
                    for( unsigned int dir = Common::FIRST_DIRECTION; dir <= Common::LAST_DIRECTION; dir++ )
                    {
                        // vec[dir - PrePathfindingCommon::FIRST_DIRECTION] is the boolean to see if the algorithm can continue in this direction.
                        // The -PrePathfindingCommon::FIRST_DIRECTION is the possible offset
                        if( vec[dir - Common::FIRST_DIRECTION] ) 
                        { 
                            std::list<Cell*> listCell;
                            switch( dir )
                            {
                            case Common::NORTH:
                                listCell = sectorDirection( dir, pSector->getNorthList() );
                                break;
                            case Common::SOUTH:
                                listCell = sectorDirection( dir, pSector->getSouthList() );
                                break;
                            case Common::EAST:
                                listCell = sectorDirection( dir, pSector->getEastList() );
                                break;
                            case Common::WEST:
                                listCell = sectorDirection( dir, pSector->getWestList() );
                                break;
                            }

                            vec[dir - Common::FIRST_DIRECTION] = this->verifySectorContinuity(dir, pSector, listCell );

                            if( vec[dir - Common::FIRST_DIRECTION] ) 
                            { 
                                vec[dir - Common::FIRST_DIRECTION] = pSector->addCellLine( dir, listCell ); 
                            }
                        }
                    }
                }
            }

            pSector->setCellsInZone();

            return pSector;
        }

        //----------------------------------------------------------------------------------------------------------------

        std::list<Cell*> Compute::sectorDirection( int dir, const std::list<Cell*> actual )
        {
            bool ContDir = true;

            std::list<Cell*> lineAfter;
            for(std::list<Cell*>::const_iterator iterCell = actual.begin(); iterCell != actual.end() && ContDir; ++iterCell )    
            {
                Cell* actualCell = NULL;
                switch( dir )
                {
                case Common::NORTH:
                    actualCell = (*iterCell)->getNorthStep();
                    break;
                case Common::SOUTH:
                    actualCell = (*iterCell)->getSouthStep();
                    break;
                case Common::EAST:
                    actualCell = (*iterCell)->getEastStep();
                    break;
                case Common::WEST:
                    actualCell = (*iterCell)->getWestStep();
                    break;
                }

                if( actualCell != NULL && !actualCell->getInZone() )
                {
                    lineAfter.push_back( actualCell );
                }
                else
                {
                    ContDir = false;                                                                                                
                }
            }                                                                                                                        
            if( lineAfter.size() > 0 && ContDir )                                                                                            
            {
                for(std::list<Cell*>::const_iterator iterCell = lineAfter.begin(); iterCell != lineAfter.end() && ContDir; ++iterCell )    
                {
					const Cell *fromCell = NULL;
					const Cell *nextCell = NULL;
					const Cell *prevCell = NULL;
                    switch( dir )
                    {
                    case Common::NORTH:
                        fromCell = (*iterCell)->getSouthStep();
                        nextCell = (*iterCell)->getEastStep();
                        prevCell = (*iterCell)->getWestStep();
                        break;
                    case Common::SOUTH:
                        fromCell = (*iterCell)->getNorthStep();
                        nextCell = (*iterCell)->getEastStep();
                        prevCell = (*iterCell)->getWestStep();
                        break;
                    case Common::EAST:
                        fromCell = (*iterCell)->getWestStep();
                        nextCell = (*iterCell)->getSouthStep();
                        prevCell = (*iterCell)->getNorthStep();
                        break;
                    case Common::WEST:
                        fromCell = (*iterCell)->getEastStep();
                        nextCell = (*iterCell)->getSouthStep();
                        prevCell = (*iterCell)->getNorthStep();
                        break;
                    }

                    // If there is only one cell
                    if( (*iterCell) == lineAfter.front() && lineAfter.size() == 1 )                                                    
                    {
                        if( !fromCell || fromCell->getInZone() )
                        {
                            ContDir = false;
                        }
                    }
                    // It's the first cell, but there is cell after
                    else if( (*iterCell) == lineAfter.front() )                                                                            
                    {                                                                                                                
                        if( !( fromCell && !fromCell->getInZone() ) || !( nextCell && !nextCell->getInZone() ))            
                        {
                            ContDir = false;
                        }
                    }
                    // It's not the first cell, not the last one too
                    else if( (*iterCell) != lineAfter.front() && (*iterCell) != lineAfter.back() )                                                
                    {
                        if( !( fromCell && !fromCell->getInZone()) || !( nextCell && !nextCell->getInZone() ) || !( prevCell && !prevCell->getInZone() ))    
                        {                                                                                                            
                            ContDir = false;                                                                                        
                        }                                                                                                            
                    }
                    // There is cell before and the cell is the last one
                    else if( (*iterCell) == lineAfter.back() )                                                                                
                    {
                        if( !( fromCell && !fromCell->getInZone()) || !( prevCell && !prevCell->getInZone() ))    
                        {
                            ContDir = false;
                        }
                    }
                }

                if( !ContDir )
                {
                    lineAfter.clear();    
                }
            }
            return lineAfter;
        }

        //----------------------------------------------------------------------------------------------------------------

        void Compute::generatePortals( std::list<Sector*> & preSector, std::list<Pathfinding::Sector*> & sector )
        {
            std::list<Sector*>::iterator preIter = preSector.begin();
            std::list<Pathfinding::Sector*>::iterator secIter = sector.begin();

            while( preIter != preSector.end() && secIter != sector.end() )
            {
                createPortals( Common::NORTH, (*preIter)->getNorthList(), (*secIter) );
                createPortals( Common::SOUTH, (*preIter)->getSouthList(), (*secIter) );
                createPortals( Common::EAST, (*preIter)->getEastList(), (*secIter) );
                createPortals( Common::WEST, (*preIter)->getWestList(), (*secIter) );

                preIter++;
                secIter++;
            }
        }

        //----------------------------------------------------------------------------------------------------------------

        // Create the portals from the zones.
        void Compute::createPortals( const int direction, const std::list<Cell*> listCell, Pathfinding::Sector * startSec )
        {
			Portal* portal = NULL;
            const Cell* nextCell = NULL;
            const Cell* prevCell = NULL;

            int actualNbDirection = 0;

            for(std::list<Cell*>::const_iterator cellSteps = listCell.begin(); cellSteps != listCell.end(); ++cellSteps )    
            {
                // With the direction, take the next cell and the previous cell. If the 2 are interconnected, it will do a 2 ways portal.
                switch( direction )
                {
                case Common::NORTH:
                    nextCell = (*cellSteps)->getNorthStep();
                    if(nextCell) { prevCell = (*cellSteps)->getNorthStep()->getSouthStep(); }
                    break;
                case Common::SOUTH:
                    nextCell = (*cellSteps)->getSouthStep();
                    if(nextCell) { prevCell = (*cellSteps)->getSouthStep()->getNorthStep(); }
                    break;
                case Common::EAST:
                    nextCell = (*cellSteps)->getEastStep();
                    if(nextCell) { prevCell = (*cellSteps)->getEastStep()->getWestStep(); }
                    break;
                case Common::WEST:
                    nextCell = (*cellSteps)->getWestStep();
                    if(nextCell) { prevCell = (*cellSteps)->getWestStep()->getEastStep(); }
                    break;
                }

                // Make sure the Cell is not already in a portal of this sector.
                bool bCellNotInPortal = true;

                if( nextCell ) // It can still continue in this direction.
                {
                    for( std::list<Portal*>::const_iterator iter = startSec->getPortals().begin(); iter != startSec->getPortals().end() && bCellNotInPortal; iter++ )
                    {
                        if( (*iter)->getBox().intersects( nextCell->getBox() ) )
                        {
                            bCellNotInPortal = false;
                        }
                    }
                }

                // The cell is not a portal already.
                if( bCellNotInPortal )
                {
                    // Reset the nextNbDirection to know where it's going
                    int nextNbDirection = 0;

                    // Count the number of direction for the portal made by the actual cell
                    if( nextCell )
                    {
                        // It's at least a one way portal that can be created at that position
                        nextNbDirection++;
                        if( prevCell )
                        {
                            // It's a 2 ways portal that can be put at that position
                            nextNbDirection++;
                        }
                    }

                    // The portal that was being created needs to be completed
                    if( actualNbDirection != nextNbDirection )
                    {
                        // Reset the number of directions
                        actualNbDirection = nextNbDirection;

                        if( actualNbDirection > 0 )
                        {
                            // It's a new portal, create it
                            // Create a new portal
                            portal = new Portal();

                            portal->setBox( (*cellSteps)->getBox() );

                            mData->addPortal( portal, Common::BASE_LEVEL );

                            // Set the portal to the sector and vice-versa
                            portal->setDestSector1( startSec );
                            startSec->addPortal( portal );
                        }
                        if( actualNbDirection > 1 )
                        {
                            // It's a 2 ways portal, set the second sector
                            Pathfinding::Sector* endSec = mData->getSectorAtPos( nextCell->getBox().getCenter(), Common::BASE_LEVEL );
                            // Set the portal to the sector and vice-versa
                            portal->setDestSector2( endSec );
                            endSec->addPortal( portal );
                        }
                    }
                }
                /**
                This mean it's the end of one of the sector.  Merge the last cell with the portal, making a full portal.
                */
                else if( actualNbDirection > 0 )
                {
                    // Extend the box.  
                    Pathfinding::Math::AxisAlignedBox actualPortalBox = portal->getBox();
                    actualPortalBox.merge( nextCell->getBox() );
                    portal->setBox( actualPortalBox );
                }
            }
        }
        //----------------------------------------------------------------------------------------------------------------

        /** An abstract method so if a brake need to be make for any reason, it can be done outside of this class
        @param int dir : The direction where it is going
        const PrePathfindingSector & pSector : The sector before modification
        const std::list<const PrePathfindingCell*> listCell : The list of Cell that is going to be added to pSector
        @return true if it can continue the sector in this direction
        */
        bool Compute::verifySectorContinuity( int, const Sector *, const std::list<Cell*> )
        {
            return true;
        }
        //----------------------------------------------------------------------------------------------------------------

        /** An abstract method so if a brake needs to be make for any reason in the algorithm and stop the execution,
        it can be done outside of this class
        @return true if the algorithm can continue
        */
        bool Compute::verifyFloodFillContinuity()
        {
            return true;
        }
        //----------------------------------------------------------------------------------------------------------------

        /**
        If anything needs to be precalculated, it can be done in this method which is call at the start of a flood fill.
        */
        void Compute::preCalculate()
        {
        }

        //----------------------------------------------------------------------------------------------------------------

        void Compute::fillFinish()
        {

        }
    } //namespace Pre
} // namespace Pathfinding
