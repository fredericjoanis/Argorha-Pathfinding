/*
-----------------------------------------------------------------------------
This source file is part of OGRE
(Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org/

Copyright (c) 2000-2006 Torus Knot Software Ltd
Also see acknowledgements in Readme.html

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

You may alternatively use this source under the terms of a specific version of
the OGRE Unrestricted License provided you have obtained such a license from
Torus Knot Software Ltd.
-----------------------------------------------------------------------------
*/
#ifndef __PATHFINDING_PLATFORM_H_
#define __PATHFINDING_PLATFORM_H_

#include "PathfindingMathConfig.h"

namespace Pathfinding
{
    namespace Math
    {
        /* Initial platform/compiler-related stuff to set.
        */
#define PATHFINDING_PLATFORM_WIN32 1
#define PATHFINDING_PLATFORM_LINUX 2
#define PATHFINDING_PLATFORM_APPLE 3

#define PATHFINDING_COMPILER_MSVC 1
#define PATHFINDING_COMPILER_GNUC 2
#define PATHFINDING_COMPILER_BORL 3

#define PATHFINDING_ENDIAN_LITTLE 1
#define PATHFINDING_ENDIAN_BIG 2

#define PATHFINDING_ARCHITECTURE_32 1
#define PATHFINDING_ARCHITECTURE_64 2

        /* Finds the compiler type and version.
        */
#if defined( _MSC_VER )
#   define PATHFINDING_COMPILER PATHFINDING_COMPILER_MSVC
#   define PATHFINDING_COMP_VER _MSC_VER

#elif defined( __GNUC__ )
#   define PATHFINDING_COMPILER PATHFINDING_COMPILER_GNUC
#   define PATHFINDING_COMP_VER (((__GNUC__)*100) + \
    (__GNUC_MINOR__*10) + \
    __GNUC_PATCHLEVEL__)

#elif defined( __BORLANDC__ )
#   define PATHFINDING_COMPILER PATHFINDING_COMPILER_BORL
#   define PATHFINDING_COMP_VER __BCPLUSPLUS__

#else
#   pragma error "No known compiler. Abort! Abort!"

#endif

        /* See if we can use __forceinline or if we need to use __inline instead */
#if PATHFINDING_COMPILER == PATHFINDING_COMPILER_MSVC
#   if PATHFINDING_COMP_VER >= 1200
#       define FORCEINLINE __forceinline
#   endif
#elif defined(__MINGW32__)
#   if !defined(FORCEINLINE)
#       define FORCEINLINE __inline
#   endif
#else
#   define FORCEINLINE __inline
#endif

        /* Finds the current platform */

#if defined( __WIN32__ ) || defined( _WIN32 )
#   define PATHFINDING_PLATFORM PATHFINDING_PLATFORM_WIN32

#elif defined( __APPLE_CC__)
#   define PATHFINDING_PLATFORM PATHFINDING_PLATFORM_APPLE

#else
#   define PATHFINDING_PLATFORM PATHFINDING_PLATFORM_LINUX
#endif

        /* Find the arch type */
#if defined(__x86_64__) || defined(_M_X64) || defined(__powerpc64__) || defined(__alpha__) || defined(__ia64__) || defined(__s390__) || defined(__s390x__)
#   define PATHFINDING_ARCH_TYPE PATHFINDING_ARCHITECTURE_64
#else
#   define PATHFINDING_ARCH_TYPE PATHFINDING_ARCHITECTURE_32
#endif

        // For generating compiler warnings - should work on any compiler
        // As a side note, if you start your message with 'Warning: ', the MSVC
        // IDE actually does catch a warning :)
#define PATHFINDING_QUOTE_INPLACE(x) # x
#define PATHFINDING_QUOTE(x) PATHFINDING_QUOTE_INPLACE(x)
#define PATHFINDING_WARN( x )  message( __FILE__ "(" QUOTE( __LINE__ ) ") : " x "\n" )

        // Disable unicode support on MingW at the moment, poorly supported in stdlibc++
        // STLPORT fixes this though so allow if found
        // MinGW C++ Toolkit supports unicode and sets the define __MINGW32_TOOLKIT_UNICODE__ in _mingw.h
#if defined( __MINGW32__ ) && !defined(_STLPORT_VERSION)
#   include<_mingw.h>
#   if defined(__MINGW32_TOOLBOX_UNICODE__)
#        define PATHFINDING_UNICODE_SUPPORT 1
#   else
#       define PATHFINDING_UNICODE_SUPPORT 0
#   endif
#else
#    define PATHFINDING_UNICODE_SUPPORT 1
#endif

        //For apple, we always have a custom config.h file
#if PATHFINDING_PLATFORM == PATHFINDING_PLATFORM_APPLE
#    include "config.h"
#endif

        //----------------------------------------------------------------------------

        //----------------------------------------------------------------------------
        // Endian Settings
        // check for BIG_ENDIAN config flag, set PATHFINDING_ENDIAN correctly
#ifdef PATHFINDING_CONFIG_BIG_ENDIAN
#    define PATHFINDING_ENDIAN PATHFINDING_ENDIAN_BIG
#else
#    define PATHFINDING_ENDIAN PATHFINDING_ENDIAN_LITTLE
#endif

        // Integer formats of fixed bit width
        typedef unsigned int uint32;
        typedef unsigned short uint16;
        typedef unsigned char uint8;
    } // namespace Math
} // namespace Pathfinding

#endif
