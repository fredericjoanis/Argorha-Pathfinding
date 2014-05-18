#ifndef __StdHeaders_H__
#define __StdHeaders_H__
// This file is taken from Ogre www.ogre3d.org

#ifdef __BORLANDC__
#define __STD_ALGORITHM
#endif

#if defined ( PATHFINDING_GCC_VISIBILITY )
/* Until libstdc++ for gcc 4.2 is released, we have to declare all
* symbols in libstdc++.so externally visible, otherwise we end up
* with them marked as hidden by -fvisible=hidden.
*
* See http://gcc.gnu.org/bugzilla/show_bug.cgi?id=20218
*/
#   pragma GCC visibility push(default)
#endif

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <cstdarg>
#include <cmath>

// STL containers
#include <vector>
#include <map>
#include <string>
#include <set>
#include <list>
#include <deque>
#include <queue>
#include <bitset>

// Note - not in the original STL, but exists in SGI STL and STLport
#if (PATHFINDING_COMPILER == PATHFINDING_COMPILER_GNUC) && !defined(STLPORT)
#   include <ext/hash_map>
#   include <ext/hash_set>
#else
#   include <hash_set>
#   include <hash_map>
#endif

// STL algorithms & functions
#include <algorithm>
#include <functional>
#include <limits>

// C++ Stream stuff
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>

#ifdef __BORLANDC__
namespace Pathfinding
{
    namespace Math
    {
        using namespace std;
    }
}
#endif

extern "C" {

#   include <sys/types.h>
#   include <sys/stat.h>

}

#if PATHFINDING_PLATFORM == PATHFINDING_PLATFORM_WIN32
#  undef min
#  undef max
#  if defined( __MINGW32__ )
#    include <unistd.h>
#  endif
#endif

#if PATHFINDING_PLATFORM == PATHFINDING_PLATFORM_LINUX
extern "C" {

#   include <unistd.h>
#   include <dlfcn.h>

}
#endif

#if PATHFINDING_PLATFORM == PATHFINDING_PLATFORM_APPLE
extern "C" {
#   include <unistd.h>
#   include <sys/param.h>
#   include <CoreFoundation/CoreFoundation.h>
}
#endif

#if defined ( PATHFINDING_GCC_VISIBILITY )
#   pragma GCC visibility pop
#endif
#endif
