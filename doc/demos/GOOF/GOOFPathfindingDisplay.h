/*
-----------------------------------------------------------------------------
This source file is part of Switch Blade

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

#ifndef _GOOFPATHFINDINGDISPLAY_H
#define _GOOFPATHFINDINGDISPLAY_H

#include "GOOFPathfindingGameSystemPrerequisites.h"
#include "GOOFPathfindingData.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include "..\Pathfinding\Utilities\PathfindingPath.h"

namespace GOOF 
{
   /** @class PathfindingDisplay GOOFPathfindingDisplay.h "include/GOOFPathfindingDisplay.h"
    *
    * This class is used to display different data about pathfinding.
    * This class is a singleton, meaning you can access it from everywhere.
    */

   class _GOOFPathfindingGameSystemExport PathfindingDisplay : public Ogre::Singleton<PathfindingDisplay>
   {
   public:

      /**
       * Function to return the singleton of the class
      */
      static PathfindingDisplay&                    getSingleton(void);

      /**
        * Display all the sectors and portals in this level
        * @param[in] in_data data to show sectors and portals
        * @param[in] in_hLevel the hierarchical level to show the sectors and portals
      */
      void showAllSectorsAndPortals( PathfindingData* in_data, int in_hLevel );

      /**
        * Display all the sectors in the zone determined by in_ID
        * @param[in] in_ID id of the partition
        * @param[in] in_data data to show sectors
        * @param[in] in_hLevel the hierarchical level to show the sectors
      */
      void showSectors( const Ogre::String in_ID, PathfindingData* in_data, int in_hLevel );
      
      /**
        * Display all the portals in the zone determined by in_ID
        * @param[in] in_ID id of the partition
        * @param[in] in_data data to show portals
        * @param[in] in_hLevel the hierarchical level to show the portals
      */
      void showPortals( const Ogre::String in_ID, PathfindingData* in_data, int in_hLevel );
      
      /**
       * Shows a path as a Bezier curve
       * @param io_lWaypoints nodes where the pathfinding is passing
       * @param[in] in_actor actor that will do the path
       * @param[in] in_nbPoints number of points to interpolate
       * @param io_node optional.  Used to group differents path together
       * @param[in] in_hLevel the hierarchical that the waypoints are
      */
      void showBezier( std::vector<Pathfinding::PathfindingAStarNode> * io_lWaypoints, const Pathfinding::PathfindingActor* in_actor, int in_nbPoints, Ogre::SceneNode* io_node, int in_hLevel );

      /**
       Shows a path as a straight line
       @param io_lWaypoints nodes where the pathfinding is passing
       @param[in] in_actor actor that will do the path
       @param[in] in_nbPoints number of points to interpolate
       @param io_node optional.  Used to group differents path together
      */
      void showStraightLine( std::vector<Pathfinding::PathfindingAStarNode> * io_lWaypoints, const Pathfinding::PathfindingActor* in_actor, int in_nbPoints, Ogre::SceneNode* io_node, int in_hLevel );

      /**
       Shows a path as a straight line
       @param io_lWaypoints nodes where the pathfinding is passing
       @param[in] in_actor actor that will do the path
       @param[in] in_nbPoints number of points to interpolate
       @param io_node optional.  Used to group differents path together
      */
      void showCardinalSpline( std::vector<Pathfinding::PathfindingAStarNode> * io_lWaypoints, const Pathfinding::PathfindingActor* in_actor, int in_nbPoints, Ogre::SceneNode* io_node, int in_hLevel );
      
      /**
       Destroy the node and the manual objects contained into
       @param io_node node to be destroyed
      */
      void destroyNode( Ogre::SceneNode* io_node );

      /** 
         @param io_gameObjectMgr gameObjectManager to be set
      */
      void setGameObjectMgr( CoreGameObjectManager* io_gameObjectMgr );

      void addPortalsToShow( const Ogre::String & in_ID );

      void addSectorsToShow( const Ogre::String & in_ID );

      void processToShow( PathfindingData* in_data, int in_hLevel );

   protected:
      /**
         The game object manager to access partitions.
      */
      CoreGameObjectManager*  mGameObjectMgr;

      MaterialPtr             mMatSector;          /// The material defined for the sectors
      MaterialPtr             mMatPortal;          /// The material defined for the portals
      SceneNode*              mNodeSector;         /// The Node to put all the sectors
      SceneNode*              mNodePortal;         /// The Node to put all the portals

      MaterialPtr             mMatLinear;          /// The material for the straight lines
      MaterialPtr             mMatBezier;          /// The material for the bezier curves
      MaterialPtr             mMatCardinalSpline;  /// The material for the cardinal splines

      PathfindingDisplay();
      virtual ~PathfindingDisplay();

   private:
      /// Private mutex, not allowed to lock from outside
	   OGRE_AUTO_MUTEX

      /**
      The function showBox is used to displayed a list of boxes in a sector.
         @param in_ID The ID of the zone in which boxes need to be displayed
         @param in_type The name of the type of boxes that need to be shown i.e. ( "Sector" or "Portal" )
         @param in_list The list containing boxes to be shown.  The parameter T is PathfindingPortal or PathfindingSector
         @param io_node The node in which the boxes belong
         @param in_material The name of the material used by the boxes to be displayed
      */
      template<class T>
      void showBox( const Ogre::String in_ID, const Ogre::String in_type, const std::list<T*> * in_list, SceneNode* io_node, const Ogre::String in_material );

      /**
       Shows a path as a straight line
       @param io_lWaypoints nodes where the pathfinding is passing
       @param[in] in_actor actor that will do the path
       @param[in] in_nbPoints number of points to interpolate
       @param io_node optional.  Used to group differents path together
      */
      void showLine( std::vector<Pathfinding::PathfindingAStarNode> * lWaypoints, const Pathfinding::PathfindingActor* in_actor, int in_nbPoints, Pathfinding::PathfindingPath* in_path, const Ogre::String in_material, Ogre::SceneNode* in_node, int in_hLevel );
      
      int mLineNumber; /// A variable kept to never have the same name for manual objects.

      std::list<Ogre::String> mIDSectorsToProcess; /// The sectors that needs to be shown
      std::list<Ogre::String> mIDPortalsToProcess; /// The portals that needs to be shown

   };


} // namespace GOOF



#endif
