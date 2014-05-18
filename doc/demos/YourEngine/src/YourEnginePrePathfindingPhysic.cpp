#include "YourEnginePrePathfindingPhysic.h"

namespace YourEnginePath
{
    PrePathfindingPhysic::PrePathfindingPhysic( )
    {
    }

    PrePathfindingPhysic::~PrePathfindingPhysic()
    {

    }

    Pathfinding::Math::Real PrePathfindingPhysic::move( const Pathfinding::Math::Vector3 & in_displacement, const Pathfinding::Actor* io_actor, Pathfinding::Math::Real& out_finalHeight )
    {
        // Return the actor couldn't move.
        return -1.0;
    }

    bool PrePathfindingPhysic::canStandOn( const Pathfinding::Actor* in_actor )
    {
        return false;
    }

    std::list<Pathfinding::Math::Real> PrePathfindingPhysic::getStandOnHeights( const Pathfinding::Actor* in_actor )
    {
        std::list<Pathfinding::Math::Real> ret;

        return ret;
    }

    void PrePathfindingPhysic::prepare( const Pathfinding::Actor* in_actor )
    {

    }
}
