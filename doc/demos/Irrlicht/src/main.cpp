/*
In this tutorial, I will show how to collision detection with the Irrlicht Engine. 
I will describe 3 methods: Automatic collision detection for moving through 3d worlds
with stair climbing and sliding, manual triangle picking and manual
scene node picking.

To start, we take the program from tutorial 2, which loaded and displayed a quake 3
level. We will use the level to walk in it and to pick triangles from it. In addition
we'll place 3 animated models into it for scene node picking. The following code 
starts up the engine and loads
a quake 3 level. I will not explain it, because it should already be known from tutorial
2.
*/
#include "IrrPrePathfindingPhysic.h"
#include <iostream>
#include "Pre/PathfindingPreCompute.h"
#include "IrrPathData.h"
#include "PathfindingPortal.h"
#include "Utilities/Math/IrrAxisAlignedBoxAdapter.h"
#include "Utilities/Math/PathfindingVector3Adapter.h"
#include "Path/PathfindingPathCardinalSpline.h"
#include "PathfindingAStar.h"

using namespace irr;

#ifdef _MSC_VER
#pragma comment(lib, "Irrlicht.lib")
#endif

video::IVideoDriver* mDriver;
scene::ISceneManager* mSmgr;
Pathfinding::Actor* mPathActor;
scene::ITriangleSelector* mSelector = 0;
scene::ICameraSceneNode* mCamera;

Pathfinding::Math::Vector3 mStartPos;
Pathfinding::Math::Vector3 mEndPos;

std::vector<Pathfinding::Math::Vector3> mPathPoints;
std::vector<Pathfinding::AStarNode> mAStarNodes;

/*
 * Handle the event for the pathfinding calculations
*/
class MyEventReceiver : public IEventReceiver
{
public:
    virtual bool OnEvent(const SEvent& event)
    {
        bool ret = false;
        if (event.EventType == EET_MOUSE_INPUT_EVENT && event.MouseInput.Event == EMIE_LMOUSE_PRESSED_DOWN )
        {
            irr::core::vector3df outColl;
            irr::core::triangle3df outTri;

            core::line3d<f32> lineCollision;
            lineCollision.start = mCamera->getPosition();
            lineCollision.end = lineCollision.start + (mCamera->getTarget() - lineCollision.start).normalize() * 1000.0f;

            if (mSmgr->getSceneCollisionManager()->getCollisionPoint( lineCollision, mSelector, outColl, outTri))
            {
                if( mStartPos != mEndPos )
                {
                    mStartPos = mEndPos;
                    mEndPos = PathIrr::PathfindingVector3Adapter(outColl);
                    mEndPos.y += mPathActor->getSmallVector().y * 0.5f;

                    Pathfinding::AStar AStar;

                    mAStarNodes = AStar.getPath(mStartPos, mEndPos, mPathActor->getData(), 0 );

                    Pathfinding::Path::CardinalSpline path;

                    path.reCalculatePoints(&mAStarNodes, mPathActor);

                    Pathfinding::Path::InterpolatePathData dataPath;
                    dataPath.waypoints = &mAStarNodes;
                    dataPath.nbPoints  = 15;
                    dataPath.data      = mPathActor->getData();
                    dataPath.hLevel    = 0;

                    mPathPoints = path.interpolateFromWaypoints( &dataPath );
                }
                else
                {
                    mEndPos = PathIrr::PathfindingVector3Adapter(outColl);
                    mEndPos += mPathActor->getSmallVector().y * 0.5f;
                }

            }

            ret = true;
        }

        return ret;
    }
};


int main()
{
    video::E_DRIVER_TYPE driverType = video::EDT_DIRECT3D9;


    printf("(t) Incomplete generation (Small part of the map will be generated)\nor\n(c) Complete pathfinding generation (Might take more than 1 minute)\n");

    
    Pathfinding::Math::AxisAlignedBox zone;
    
    char i = 0;
    while( i != 't' && i != 'c' && i != 'd' )
    {
        std::cin >> i;
        switch(i)
        {
            case 'c': zone = Pathfinding::Math::AxisAlignedBox( -1000, -70, -1200, 1300, 500, 1000 );
            break;
            case 't': zone = Pathfinding::Math::AxisAlignedBox( -500, -70, -600, 150, 500, 0 );
            break;
            case 'd': zone = Pathfinding::Math::AxisAlignedBox( 500, 175, 700, 665, 400, 800 );
            break;
        }
    }


    // create device
    MyEventReceiver receiver;

    IrrlichtDevice *device = createDevice(driverType, core::dimension2d<s32>(640, 480), 16, false, false, false, &receiver );

    if (device == 0)
        return 1; // could not create selected driver.

    mDriver = device->getVideoDriver();
    mSmgr = device->getSceneManager();

    device->getFileSystem()->addZipFileArchive("../media/map-20kdm2.pk3");
    
    scene::IAnimatedMesh* q3levelmesh = mSmgr->getMesh("20kdm2.bsp");
    scene::ISceneNode* q3node = 0;
    
    if (q3levelmesh)
    {
        q3node = mSmgr->addOctTreeSceneNode(q3levelmesh->getMesh(0));
        if (q3node)
        {        
            q3node->setPosition(core::vector3df(-1350,-130,-1400));

            mSelector = mSmgr->createOctTreeTriangleSelector(q3levelmesh->getMesh(0), q3node, 128);
            q3node->setTriangleSelector(mSelector);
        }
    }
   
     mCamera = mSmgr->addCameraSceneNodeFPS();
     //m_camera->setPosition(core::vector3df(500,400,600));
     mCamera->setPosition(core::vector3df(60,0,-50));

    if (mSelector)
    {
        scene::ISceneNodeAnimator* anim = mSmgr->createCollisionResponseAnimator( mSelector, mCamera, core::vector3df(30,50,30), core::vector3df(0,-3,0), core::vector3df(0,50,0));
        mSelector->drop();
    }

    // disable mouse cursor

    device->getCursorControl()->setVisible(false);

    // add billboard

    scene::IBillboardSceneNode * bill = mSmgr->addBillboardSceneNode();
    bill->setMaterialType(video::EMT_TRANSPARENT_ADD_COLOR );
    bill->setMaterialTexture(0, mDriver->getTexture("../media/particle.bmp"));
    bill->setMaterialFlag(video::EMF_LIGHTING, false);
    bill->setMaterialFlag(video::EMF_ZBUFFER, false);
    bill->setSize(core::dimension2d<f32>(20.0f, 20.0f));

    // add 3 animated faeries.

    video::SMaterial material;
    material.setTexture(0, mDriver->getTexture("../media/faerie2.bmp"));
    material.Lighting = true;

    scene::IAnimatedMeshSceneNode* node = 0;
    scene::IAnimatedMesh* faerie = mSmgr->getMesh("../media/faerie.md2");

    if (faerie)
    {
        node = mSmgr->addAnimatedMeshSceneNode(faerie);
        node->setPosition(core::vector3df(-70,0,-90));
        node->setMD2Animation(scene::EMAT_RUN);
        node->getMaterial(0) = material;

        node = mSmgr->addAnimatedMeshSceneNode(faerie);
        node->setPosition(core::vector3df(-70,0,-30));
        node->setMD2Animation(scene::EMAT_SALUTE);
        node->getMaterial(0) = material;

        node = mSmgr->addAnimatedMeshSceneNode(faerie);
        node->setPosition(core::vector3df(-70,0,-60));
        node->setMD2Animation(scene::EMAT_JUMP);
        node->getMaterial(0) = material;
    }

    material.setTexture(0, 0);
    material.Lighting = false;

    // Add a light

    mSmgr->addLightSceneNode(0, core::vector3df(-60,100,400), video::SColorf(1.0f,1.0f,1.0f,1.0f), 600.0f);

    /*
    For not making it to complicated, I'm doing picking inside the drawing loop.
    We take two pointers for storing the current and the last selected scene node and 
    start the loop.
    */

    scene::ISceneNode* selectedSceneNode = 0;
    scene::ISceneNode* lastSelectedSceneNode = 0;
    
    int lastFPS = -1;

    Pathfinding::Pre::Compute compute;
    PathIrr::PrePathfindingPhysic phys(mSmgr, mSelector);
    PathIrr::IrrPathData* pathData = new PathIrr::IrrPathData();
    mPathActor = new Pathfinding::Actor(pathData, Pathfinding::Math::AxisAlignedBox( 0.f, 0.f, 0.f, 30.f, 50.f, 30.f));

    gui::IGUIEnvironment* guienv = device->getGUIEnvironment();

    wchar_t buffer[50];
    swprintf(buffer, L"%d, %d, %d", mCamera->getPosition().X, mCamera->getPosition().Y, mCamera->getPosition().Z);

    irr::gui::IGUIStaticText* text = guienv->addStaticText(buffer, core::rect<s32>(10,10,260,22), true,true,0,-1,true);

    // Just take all the level

    bool bFirstRun = true;

    while(device->run())
    if (device->isWindowActive())
    {
        mDriver->beginScene(true, true, 0);

        mSmgr->drawAll();
        guienv->drawAll();

        core::line3d<f32> line;
        line.start = mCamera->getPosition();
        line.end = line.start + (mCamera->getTarget() - line.start).normalize() * 1000.0f;

        swprintf(buffer, L"%d, %d, %d", (int)mCamera->getPosition().X, (int)mCamera->getPosition().Y, (int)mCamera->getPosition().Z);
        text->setText(buffer);

        core::vector3df intersection;
        core::triangle3df tri;

        if (mSmgr->getSceneCollisionManager()->getCollisionPoint(line, mSelector, intersection, tri))
        {
            bill->setPosition(intersection);
                
            mDriver->setTransform(video::ETS_WORLD, core::matrix4());
            mDriver->setMaterial(material);
            mDriver->draw3DTriangle(tri, video::SColor(0,255,0,0));
        }


        if( bFirstRun )
        {
            printf( "Starting pathfinding generation" );
            if( i == 'c' )
            {
                printf( ", might be long ; please wait!" );
            }
            compute.generateSectorsAndPortals(&phys, mPathActor, &zone, 0 );
            bFirstRun = false;
        }

        for( std::list<Pathfinding::Portal*>::iterator iter = pathData->mListPortal.begin(); iter != pathData->mListPortal.end(); iter++ )
        {
            mDriver->draw3DBox(PathIrr::IrrAxisAlignedBoxAdapter((*iter)->getBox()),irr::video::SColor(255,0,0,255));
        }

        for( std::list<Pathfinding::Sector*>::iterator iter = pathData->mListSector.begin(); iter != pathData->mListSector.end(); iter++ )
        {
            mDriver->draw3DBox(PathIrr::IrrAxisAlignedBoxAdapter((*iter)->getBox()),irr::video::SColor(255,255,0,0));
        }

        std::vector<Pathfinding::Math::Vector3>::iterator iter = mPathPoints.begin();

        while( iter != mPathPoints.end() )
        {
            iter++;
            if( iter != mPathPoints.end() )
            {
                core::vector3df startPos;
                core::vector3df endPos;

                startPos = PathIrr::IrrVector3Adapter( *iter );
                iter--;
                endPos = PathIrr::IrrVector3Adapter( *iter );
                iter++;

                mDriver->draw3DLine(startPos, endPos,irr::video::SColor(255,0,255,0));
            }
        }

        /*
        Another type of picking supported by the Irrlicht Engine is scene node picking
        based on bouding boxes. Every scene node has got a bounding box, and because of
        that, it's very fast for example to get the scene node which the camera looks
        at. Again, we ask the collision manager for this, and if we've got a scene node,
        we highlight it by disabling Lighting in its material, if it is not the 
        billboard or the quake 3 level.
        */

        selectedSceneNode = mSmgr->getSceneCollisionManager()->getSceneNodeFromCameraBB(mCamera);

        if (lastSelectedSceneNode)
            lastSelectedSceneNode->setMaterialFlag(video::EMF_LIGHTING, true);

        if (selectedSceneNode == q3node || selectedSceneNode == bill)
            selectedSceneNode = 0;

        if (selectedSceneNode)
            selectedSceneNode->setMaterialFlag(video::EMF_LIGHTING, false);

        lastSelectedSceneNode = selectedSceneNode;


        /*
        That's it, we just have to finish drawing.
        */

        mDriver->endScene();

        int fps = mDriver->getFPS();

        if (lastFPS != fps)
        {
          core::stringw str = L"Collision detection example - Irrlicht Engine [";
          str += mDriver->getName();
          str += "] FPS:";
          str += fps;

          device->setWindowCaption(str.c_str());
          lastFPS = fps;
        }
    }

    device->drop();

    // m_pathActor also delete m_Data
    delete mPathActor;
    delete pathData;

    return 0;
}

