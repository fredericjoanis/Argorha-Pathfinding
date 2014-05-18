#include "IrrPrePathfindingPhysic.h"
#include "Utilities/Math/IrrVector3Adapter.h"
#include "Utilities/Math/PathfindingVector3Adapter.h"


namespace PathIrr
{
    PrePathfindingPhysic::PrePathfindingPhysic( irr::scene::ISceneManager* scmgr, irr::scene::ITriangleSelector* selector )
    {
        mScmgr = scmgr;
        mSelector = selector;
    }

    PrePathfindingPhysic::~PrePathfindingPhysic()
    {

    }

    // Should be in_actor, float out_y.  No confusion on the actor movement.
    Pathfinding::Math::Real PrePathfindingPhysic::move( const Pathfinding::Math::Vector3 & in_displacement, const Pathfinding::Actor* io_actor, Pathfinding::Math::Real& out_finalHeight )
    {
        Pathfinding::Math::Real ret = 1.0;

        irr::core::vector3df actorPos = IrrVector3Adapter(io_actor->getPosition());
        actorPos.Y += io_actor->getSmallVector().y * 0.5f;

        irr::core::line3df lineCollision(actorPos,IrrVector3Adapter(io_actor->getPosition() + in_displacement));

        irr::core::vector3df outColl;
        irr::core::triangle3df outTri;

        if (mScmgr->getSceneCollisionManager()->getCollisionPoint( lineCollision, mSelector, outColl, outTri))
        {
            // There has been a collision
            ret = -1.0;
        }
        else
        {
            irr::core::line3df floorColl( lineCollision.end - irr::core::vector3df(0.f, io_actor->getSmallVector().y, 0.f), lineCollision.end + irr::core::vector3df(0.f, io_actor->getSmallVector().y * 0.5f, 0.f) );

            if (mScmgr->getSceneCollisionManager()->getCollisionPoint( floorColl, mSelector, outColl, outTri))
            {
                out_finalHeight = outColl.Y;
            }
            else
            {
                ret = -1.0;
            }
        }
        return ret;
    }

    bool PrePathfindingPhysic::canStandOn( const Pathfinding::Actor* in_actor )
    {
        Pathfinding::Math::Vector3 smallVec = in_actor->getSmallVector();
        smallVec *= 0.5f;

        irr::core::line3df lineCollision(IrrVector3Adapter(in_actor->getPosition() - smallVec ), IrrVector3Adapter(in_actor->getPosition() + smallVec));

        irr::core::vector3df outColl;
        irr::core::triangle3df outTri;

        return !mScmgr->getSceneCollisionManager()->getCollisionPoint( lineCollision, mSelector, outColl, outTri);
    }

    std::list<Pathfinding::Math::Real> PrePathfindingPhysic::getStandOnHeights( const Pathfinding::Actor* in_actor )
    {
        std::list<Pathfinding::Math::Real> ret;
        irr::core::vector3df height(0.f, 500.f, 0.f);
        irr::core::vector3df smallY(0.f, 0.1f, 0.f);
        irr::core::line3df lineCollision(IrrVector3Adapter(in_actor->getPosition()) - height, IrrVector3Adapter(in_actor->getPosition()) + height);

        irr::core::vector3df outColl;
        irr::core::triangle3df outTri;

        while( mScmgr->getSceneCollisionManager()->getCollisionPoint( lineCollision, mSelector, outColl, outTri) )
        {
            lineCollision.start = outColl + smallY;
            lineCollision.end = outColl + height;

            // Make sure we don't take the roof
            if( outTri.getNormal().Y > 0.f )
            {
                Pathfinding::Math::Vector3 vecRet = PathfindingVector3Adapter( outColl );

                irr::core::vector3df smallVector =  IrrVector3Adapter(in_actor->getSmallVector()) * 0.5f;

                outColl.Y += smallVector.Y;
                irr::core::line3df actorBoxCollision11( outColl - smallVector, outColl + smallVector );
                irr::core::line3df actorBoxCollision12;
                actorBoxCollision12.end = actorBoxCollision11.start;
                actorBoxCollision12.start = actorBoxCollision11.end;

                smallVector.X = -smallVector.X;
                irr::core::line3df actorBoxCollision21( outColl - smallVector + smallY, outColl + smallVector + smallY );
                irr::core::line3df actorBoxCollision22;
                actorBoxCollision22.end = actorBoxCollision21.start;
                actorBoxCollision22.start = actorBoxCollision21.end;

                bool bContinue = true;

                // Collision to make sure the actor can really stand on that place
                if( mScmgr->getSceneCollisionManager()->getCollisionPoint( actorBoxCollision11, mSelector, outColl, outTri) )
                {
                    irr::core::vector3df abc = outTri.getNormal().normalize();

                    irr::core::vector3df outColl2;
                    // Detection for a step
                    mScmgr->getSceneCollisionManager()->getCollisionPoint( actorBoxCollision12, mSelector, outColl2, outTri);

                    abc = outTri.getNormal().normalize();

                    if( outColl.X != outColl2.X || outColl.Z != outColl2.Z || ( Pathfinding::Math::Math::Abs( outColl.Y - outColl2.Y ) > in_actor->getSmallVector().y * 0.5f ) )
                    {
                        bContinue = false;
                    }
                    else
                    {
                        irr::core::vector3df abc = outTri.getNormal().normalize();
                        abc = abc;
                    }
                }

                // Collision to make sure the actor can really stand on that place
                if( bContinue && mScmgr->getSceneCollisionManager()->getCollisionPoint( actorBoxCollision21, mSelector, outColl, outTri) )
                {
                    irr::core::vector3df outColl2;

                    irr::core::vector3df abc = outTri.getNormal().normalize();

                    // Detection for a step
                    mScmgr->getSceneCollisionManager()->getCollisionPoint( actorBoxCollision22, mSelector, outColl2, outTri);
                    if( outColl.X != outColl2.X || outColl.Z != outColl2.Z || 
                        ( Pathfinding::Math::Math::Abs( outColl.Y - outColl2.Y ) > in_actor->getSmallVector().y * 0.5f ) || outTri.getNormal().normalize().X == -1.f )
                    {
                        bContinue = false;
                    }
                    else
                    {
                        irr::core::vector3df abc = outTri.getNormal().normalize();
                        abc = abc;
                    }
                }

                if( bContinue )
                {
                    ret.push_back( vecRet.y );
                }
            }
        }

        return ret;
    }

    void PrePathfindingPhysic::prepare( const Pathfinding::Actor* in_actor )
    {

    }
}
