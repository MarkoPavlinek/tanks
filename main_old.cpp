
#include <irrlicht.h>
#include <btBulletDynamicsCommon.h>
#include "iostream"
#include <string>
#include <vector>
#include <LinearMath/btIDebugDraw.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <wchar.h>
    using namespace std;
    using namespace irr;
    using namespace core;
    using namespace scene;
    using namespace video;
    using namespace io;
    using namespace gui;
    using irr::core::vector2df;
    using irr::core::vector3df;
    using irr::scene::IMesh;
    using irr::scene::IMeshBuffer;
    using irr::scene::IMeshSceneNode;
    using irr::scene::IAnimatedMeshSceneNode;



static bool Done = false;
static btDiscreteDynamicsWorld *World;
static IrrlichtDevice *irrDevice;
static IVideoDriver *irrDriver;
static ISceneManager *irrScene;
static IGUIEnvironment *irrGUI;
static IFileSystem *irrFile;
static ITimer *irrTimer;
static ILogger *irrLog;
static list<btRigidBody *> Objects;
static irr::scene::ICameraSceneNode *debugCamera;
static bool draw_irr = true;
static bool draw_phy = true;


auto toBtVector = [ &]( const vector3df & vec,const vector3df & scal ) -> btVector3
    {
        btVector3 bt( vec.X * scal.X, vec.Y * scal.Y, vec.Z * scal.Z );

        return bt;
    };
class EventReceiverClass : public IEventReceiver

{

public:

	virtual bool OnEvent(const SEvent &TEvent);
};
class BulletDebugRender: public btIDebugDraw{
    int m_debugMode;

public:

    virtual void drawLine(const btVector3& from,const btVector3& to
              ,const btVector3& color);
    //virtual void drawLine(const btVector3& from,const btVector3& to, const btVector3& fromColor, const btVector3& toColor);
    virtual void   drawContactPoint(const btVector3& PointOnB
             ,const btVector3& normalOnB,btScalar distance
              ,int lifeTime,const btVector3& color){};

    virtual void   reportErrorWarning(const char* warningString){};

    virtual void   draw3dText(const btVector3& location
                             ,const char* textString){};

    virtual void   setDebugMode(int debugMode){this->m_debugMode =debugMode;}

    virtual int      getDebugMode() const { return m_debugMode; }

};



class tank
{

public:
irr::scene::ISceneNode* node;
irr::scene::IMeshSceneNode* LeftTrackNode;
irr::scene::IMeshSceneNode* RightTrackNode;
irr::scene::IMeshSceneNode* DomeNode;
irr::scene::IMeshSceneNode* CannonNode;
irr::scene::IMeshSceneNode* bodynode;
irr::scene::ICameraSceneNode* DomeCamera;
irr::scene::ICameraSceneNode* DriverCamera;
//irr::scene::ISceneNode *CamHelp;
//irr::scene::ISceneNode *CamHelpT;
btScalar mass;
btRaycastVehicle *phytank;
btRigidBody *body;
btRigidBody *LTrack;
btRigidBody *Rtrack;
btRigidBody *Dome;
btRigidBody *Cannon;



float maxEngineForce;
float LEngineForce = 0.f;
float REngineForce = 0.f;
float maxBrakeForce;
float BreakForce = 0.f;

tank()
{
}

tank(irr::scene::ISceneManager *scene,irr::core::vector3df position)
{
io::path filename = "/home/wbr/SteamTanks/bin/Debug/Models/Tank/tank.obj";
node = scene->addMeshSceneNode(scene->getMesh(filename),0,100,position);
node->setMaterialFlag(EMF_WIREFRAME, true);
//LeftTrackNode = scene->addCubeSceneNode(5.0,node,-1,irr::core::vector3df(-5,-5,0));
//RightTrackNode = scene->addCubeSceneNode(5.0,node,-1,irr::core::vector3df(5,-5,0));
//node->setMaterialFlag(irr::video::EMF_WIREFRAME,true);
//DomeNode = scene->addSphereSceneNode(2.0f,16,node,-1,irr::core::vector3df(0,5,0));
//bodynode = scene->addCubeSceneNode(10.0f,node);
//Camera = scene->addCameraSceneNodeFPS(0,100.f,0.1f);
//Camera->setPosition(vector3df(0, 4, 0));
	//Camera->setTarget(vector3df(10, 0, 0));
}
tank(irr::scene::ISceneManager *scene,irr::core::vector3df position,btScalar Tmass)
{
io::path filename;
mass = Tmass;
filename = "/home/wbr/SteamTanks/bin/Debug/Models/Tank/body.obj";
node = scene->addMeshSceneNode(scene->getMesh(filename),0,101, position);
node->setMaterialFlag(irr::video::EMF_WIREFRAME,true);


filename = "/home/wbr/SteamTanks/bin/Debug/Models/Tank/trackL.obj";
LeftTrackNode = scene->addMeshSceneNode(scene->getMesh(filename),node,100);
LeftTrackNode->setMaterialFlag(irr::video::EMF_WIREFRAME,true);
filename = "/home/wbr/SteamTanks/bin/Debug/Models/Tank/trackR.obj";
RightTrackNode = scene->addMeshSceneNode(scene->getMesh(filename),node,100);
RightTrackNode->setMaterialFlag(irr::video::EMF_WIREFRAME,true);

filename = "/home/wbr/SteamTanks/bin/Debug/Models/Tank/dome.obj";
DomeNode = scene->addMeshSceneNode(scene->getMesh(filename),node,100);
DomeNode->setMaterialFlag(irr::video::EMF_WIREFRAME,true);
filename = "/home/wbr/SteamTanks/bin/Debug/Models/Tank/cannon.obj";
CannonNode = scene->addMeshSceneNode(scene->getMesh(filename),DomeNode,100);
CannonNode->setMaterialFlag(irr::video::EMF_WIREFRAME,true);


//scene->addSphereSceneNode(.5f,16,DomeNode,-1,DomeCamera->getAbsolutePosition());
DriverCamera = scene->addCameraSceneNode(node);
DriverCamera->setPosition(irr::core::vector3df(1.5,1,0));
//DriverCamera->setTarget(DriverCamera->getAbsolutePosition()+irr::core::vector3df(1,0,0));
DriverCamera->bindTargetAndRotation(true);
//CamHelp = scene->addSphereSceneNode(.5f,16,DomeNode,100,);
//CamHelpT = scene->addSphereSceneNode(.1f,16,DomeNode,100,CamHelp->getPosition()+irr::core::vector3df(10.,0,0));
//CamHelpT->setVisible(false);
//CamHelp->setVisible(false);
DomeCamera = scene->addCameraSceneNode(DomeNode);
DomeCamera->setPosition(DomeNode->getPosition()+irr::core::vector3df(0,2.5,0));
//DomeCamera->setTarget(CamHelpT->getAbsolutePosition());
DomeCamera->bindTargetAndRotation(true);
DomeCamera->setNearValue(3);
DomeCamera->setFarValue(1000);
DomeCamera->setFOV(45);

node->setName("Active");
maxEngineForce = 50.f;
std::cout << node->getName() <<".\n";
//DomeCamera->setPosition(CamHelp->getPosition());
//DomeCamera->setTarget(CamHelpT->getPosition());



//scene->addSphereSceneNode(.5f,16,node,100,DomeNode->getAbsolutePosition());
//	Camera->setTarget(vector3df(10, 0, 0));
//node->setPosition(core::vector3df(0.,15.,0));
//std::cout << irrFile->getWorkingDirectory().c_str() << ".\n" << filename.c_str() << ".\n";
//std::cout << Camera->getTarget().X <<" "
//<< Camera->getTarget().Y <<" "
//<< Camera->getTarget().Z <<" "
// << ".\n"
// << Camera->getPosition().X <<" "
// << Camera->getPosition().Y <<" "
// << Camera->getPosition().Z
// << ".\n";

}


void updateCamera(){
vector3df eulerRotation;
eulerRotation = this->node->getRotation();

//this->DomeCamera->setTarget(this->CamHelpT->getAbsolutePosition());
vector3df camUp = eulerRotation.rotationToDirection(vector3df(0,1,0));
vector3df camT = eulerRotation.rotationToDirection(vector3df(1,0,0));
camUp.normalize();
camT.normalize();
this->DomeCamera->setUpVector(camUp);
this->DomeCamera->setTarget(this->DomeCamera->getAbsolutePosition()+camT);

this->DriverCamera->setUpVector(camUp);
this->DriverCamera->setTarget(this->DriverCamera->getAbsolutePosition()+camT);

}

void setLeftTrack(float force)
{

if (force + LEngineForce > maxEngineForce)
{
LEngineForce = maxEngineForce;
}
else
{
LEngineForce += force;
}


for(int i=0;i<5;i++)
{
phytank->applyEngineForce(LEngineForce,i);
}

std::cout<<LEngineForce<<"\n";

}
void setRightTrack(float force)
{
if (force + REngineForce > maxEngineForce)
{
REngineForce = maxEngineForce;
}
else
{
REngineForce += force;
}


for(int i=5;i<10;i++)
{
phytank->applyEngineForce(REngineForce,i);
}



}


};
static tank Player;
static tank *Dummy;

class MotionState : public btMotionState
{


public:

   MotionState(const btTransform& initalTransformation, irr::scene::ISceneNode* const node) :
      node(node), initalTransformation(initalTransformation)
   {

   }

   void getWorldTransform(btTransform& worldTrans) const
   {
      worldTrans = this->initalTransformation;
   }

   void setWorldTransform(const btTransform& worldTrans)
   {
      worldTrans.getOpenGLMatrix(matr.pointer());

      this->node->setRotation(matr.getRotationDegrees());
      this->node->setPosition(matr.getTranslation());
   }

private:

   irr::scene::ISceneNode* const node;

   irr::core::matrix4 matr;

   btTransform initalTransformation;
};


btTriangleMesh* makeMesh(irr::scene::ISceneNode * node);
static void UpdatePhysics(u32 TDeltaTime);
static void UpdateRender(btRigidBody *TObject);
static void ClearObjects();

void QuaternionToEuler(const btQuaternion &TQuat, btVector3 &TEuler) {
	btScalar W = TQuat.getW();
	btScalar X = TQuat.getX();
	btScalar Y = TQuat.getY();
	btScalar Z = TQuat.getZ();
	float WSquared = W * W;
	float XSquared = X * X;
	float YSquared = Y * Y;
	float ZSquared = Z * Z;

	TEuler.setX(atan2f(2.0f * (Y * Z + X * W), -XSquared - YSquared + ZSquared + WSquared));
	TEuler.setY(asinf(-2.0f * (X * Z - Y * W)));
	TEuler.setZ(atan2f(2.0f * (X * Y + Z * W), XSquared - YSquared - ZSquared + WSquared));
	TEuler *= core::RADTODEG;
}


void UpdatePhysics(u32 TDeltaTime) {

	World->stepSimulation(TDeltaTime * 0.001f, 60);

	btRigidBody *TObject;
	// Relay the object's orientation to irrlicht
	for(core::list<btRigidBody *>::Iterator it = Objects.begin(); it != Objects.end(); ++it) {

		//UpdateRender(*Iterator);
		scene::ISceneNode *Node = static_cast<scene::ISceneNode *>((*it)->getUserPointer());
		TObject = *it;

		// Set position
		btVector3 Point = TObject->getCenterOfMassPosition();
		Node->setPosition(core::vector3df((f32)Point[0], (f32)Point[1], (f32)Point[2]));

		// Set rotation
		btVector3 EulerRotation;
		QuaternionToEuler(TObject->getOrientation(), EulerRotation);
		Node->setRotation(core::vector3df(EulerRotation[0], EulerRotation[1], EulerRotation[2]));
		//Player->DomeCamera->setTarget(Player->CamHelpT->getAbsolutePosition());
		//std::cout << Node->getID() <<".\n";
		if (Node->getName() == "Active")
		{

		std::cout << "update target" <<".\n";

		}
		/*
		core::list<ISceneNode*>::Iterator itt = Children.begin();
            while(itt != Children.end())
            {
                if((*itt)) return (*itt);

                ++itt;
            }
*/
	}
}



void UpdateRender(btRigidBody *TObject)
 {
	ISceneNode *Node = static_cast<ISceneNode *>(TObject->getUserPointer());

	// Set position
	btVector3 Point = TObject->getCenterOfMassPosition();
	Node->setPosition(vector3df((f32)Point[0], (f32)Point[1], (f32)Point[2]));

	// Set rotation
	vector3df Euler;
	const btQuaternion& TQuat = TObject->getOrientation();
	quaternion q(TQuat.getX(), TQuat.getY(), TQuat.getZ(), TQuat.getW());
	q.toEuler(Euler);
	Euler *= RADTODEG;
	Node->setRotation(Euler);
}
bool makeVehlice(tank Tank,btScalar Tmass)
{

    //btTriangleMesh *chasisShape;
    //chasisShape = makeMesh(node);


       btTransform transform;
      //      std::cout << "Tarnsform " <<".\n";
    transform.setIdentity();
      //      std::cout << "ident " <<".\n";
    //transform.setOrigin(toBtVector(Tank.node->getAbsolutePosition(),Tank.node->getScale()));
    //transform.setOrigin(toBtVector(Tank.node->getPosition(),Tank.node->getScale()));
     transform.setOrigin(btVector3(0,0,0));
  //
    btCollisionShape *chassisShape = new btConvexTriangleMeshShape(makeMesh(Tank.node));
    btCollisionShape *LwheelShape = new btConvexTriangleMeshShape(makeMesh(Tank.LeftTrackNode));
    btCollisionShape *RwheelShape = new btConvexTriangleMeshShape(makeMesh(Tank.RightTrackNode));
    btCollisionShape *DomeShape = new btConvexTriangleMeshShape(makeMesh(Tank.DomeNode));
    btCollisionShape *CannonShape = new btConvexTriangleMeshShape(makeMesh(Tank.CannonNode));

    btCompoundShape *vehicleShape = new btCompoundShape();

    btCompoundShape *TurretShape = new btCompoundShape();



   // transform.setIdentity();
   //transform.setOrigin(btVector3(0,0,0));
   //transform.setOrigin(toBtVector(Tank.node->getAbsolutePosition(),Tank.node->getScale()));
    vehicleShape->addChildShape(transform,chassisShape);

//    transform.setIdentity();
//    transform.setOrigin(toBtVector(Tank.LeftTrackNode->getPosition(),Tank.LeftTrackNode->getScale()));
  // transform.setIdentity();
  //  transform.setOrigin(btVector3(0,-0.1,0));
   //transform.setOrigin(btVector3(0,0,0));
  // transform.setOrigin(toBtVector(Tank.node->getAbsolutePosition(),Tank.node->getScale()));
   vehicleShape->addChildShape(transform,LwheelShape);
//
  //  transform.setIdentity();
  //  transform.setOrigin(btVector3(0,-0.1,0));
//   transform.setOrigin(toBtVector(Tank.RightTrackNode->getPosition(),Tank.RightTrackNode->getScale()));
  // transform.setIdentity();
  // transform.setOrigin(toBtVector(Tank.node->getAbsolutePosition(),Tank.node->getScale()));
   vehicleShape->addChildShape(transform,RwheelShape);

   TurretShape->addChildShape(transform,DomeShape);
   TurretShape->addChildShape(transform,CannonShape);
   //





    btScalar chassisMass(Tmass);
    btVector3 chassisInertia(0.0f, 0.0f, 0.0f);
    btVector3 turretInertia(0.0f, 0.0f, 0.0f);


    //transform.setIdentity();
      //      std::cout << "ident " <<".\n";
    //transform.setOrigin(toBtVector(Tank.node->getAbsolutePosition(),Tank.node->getScale()));
//transform.setIdentity();
//transform.setOrigin(toBtVector(Tank.node->getAbsolutePosition(),Tank.node->getScale()));


    transform.setOrigin(btVector3(Player.node->getAbsolutePosition().X,Player.node->getAbsolutePosition().Y-1.f,Player.node->getAbsolutePosition().Z));
//transform.inverse();
    MotionState* vehicleMotionState = new MotionState(transform,Tank.node);//(btTransform(btQuaternion(0.0f, 0.0f, 0.0f, 0.0f), btVector3(0.0f, 0.0f, 0.0f)));
    vehicleShape->calculateLocalInertia(chassisMass, chassisInertia);
    TurretShape->calculateLocalInertia(0.5f, turretInertia);

    //std::cout<<"inertia " << chassisInertia.x()<<" "<<chassisInertia.y() <<" "<< chassisInertia.z()<<"\n";




    //vehicleBody->addChildShape(transform,chassisShape);

    //vehicleBody->addChildShape(trans,LwheelShape);
   // vehicleBody->addChildShape(trans,RwheelShape);
//vehicleShape->addChildShape(transform,TurretShape);
    btRigidBody::btRigidBodyConstructionInfo chassisRigidBodyCI(chassisMass, vehicleMotionState, vehicleShape, chassisInertia);
    btRigidBody::btRigidBodyConstructionInfo TurretRigidBodyCI(0.5f, vehicleMotionState, TurretShape,btVector3(0,0,0));

    btRigidBody* chassisRigidBody = new btRigidBody(chassisRigidBodyCI);
    btRigidBody* turretRigidBody = new btRigidBody(TurretRigidBodyCI);
    chassisRigidBody->setActivationState(DISABLE_DEACTIVATION);
    turretRigidBody->setActivationState(DISABLE_DEACTIVATION);
  //  chassisRigidBody->setCenterOfMassTransform(transform);
    //chassisRigidBody->clearForces();
    //chassisRigidBody->applyGravity();
// Be sure to add the chassis of the vehicle into the world as a rigid body



    chassisRigidBody->setUserPointer((void*)(Tank.node));
    std::cout<<Tank.node->getName()<<"\n";
    //turretRigidBody->setUserPointer((void*)(Tank.DomeNode));


    //vehicleBody->addChildShape(trans,chassisShape);
    //vehicleBody->addChildShape(trans,LwheelShape);
    //vehicleBody->addChildShape(trans,RwheelShape);
    //chassisRigidBody->setUserPointer((void*)(Tank->node));

    World->addRigidBody(chassisRigidBody);
    World->addRigidBody(turretRigidBody);

    btVector3 anchor = btVector3(0,1.5,0);
    btVector3 rv1 =  btVector3(0,0,0);
    btVector3 rv2 =  btVector3(1,0,0);



    //btHinge2Constraint *domekontakt = new btHinge2Constraint(*chassisRigidBody,*turretRigidBody,anchor,rv1,rv2);
    //btPoint2PointConstraint *domepointkontakt = new btPoint2PointConstraint(*chassisRigidBody,*turretRigidBody,rv1,rv1);

    std::cout<<" origin "<< chassisRigidBody->getCenterOfMassPosition().x() << " " << Tank.node->getPosition().X
    << chassisRigidBody->getCenterOfMassPosition().y() << " "<<" "<<Tank.node->getPosition().Y
    << chassisRigidBody->getCenterOfMassPosition().z() << " "<<" "<<Tank.node->getPosition().Z <<".\n";

    //btVector3 Point = chassisRigidBody->getCenterOfMassPosition();
	//Player.node->setPosition(vector3df((f32)Point[0], (f32)Point[1], (f32)Point[2]));
//    World->addRigidBody()
    Player.body =chassisRigidBody;
  //domekontakt->setDbgDrawSize(10.f);
   //domepointkontakt->setDbgDrawSize(15.f);

btRaycastVehicle::btVehicleTuning tuning;
btVehicleRaycaster* raycaster = new btDefaultVehicleRaycaster(World);
btRaycastVehicle* vehicle = new btRaycastVehicle(tuning, chassisRigidBody, raycaster);
vehicle->setCoordinateSystem(0, 1, 2);

btVector3 RwheelDirection(0.0f, -1.0f, 0.0f);
btVector3 LwheelDirection(0.0f, -1.0f, 0.0f);
btVector3 RwheelAxis(0.0f, 0.0f, 1.0f);
btVector3 LwheelAxis(0.0f, 0.0f, 1.0f);
btScalar suspensionRestLength(3.2f);
btScalar suspensionRestLengthShort(0.7f);
btScalar suspensionRestLengthShorter(3.f);
btScalar wheelRadius(.1f);
tuning.m_maxSuspensionForce = 2000;
tuning.m_suspensionDamping = 0.1;
tuning.m_maxSuspensionTravelCm =10;
tuning.m_suspensionDamping = 10;

// Be sure to attach the wheels not higher than the upper bounds of the hull of the vehicle chassis
//tuning.m_frictionSlip = 10.f;
//tuning.m_suspensionDamping = 10.f;
//left side
vehicle->addWheel(btVector3(3.0f, 1.0f, 1.5f), LwheelDirection, LwheelAxis, suspensionRestLengthShorter, wheelRadius, tuning, false);
vehicle->addWheel(btVector3(1.f, 1.0f, 1.5f), LwheelDirection, LwheelAxis, suspensionRestLength, wheelRadius, tuning, false);
vehicle->addWheel(btVector3(.0f, 1.0f, 1.5f), LwheelDirection, LwheelAxis, suspensionRestLength, wheelRadius, tuning, false);
vehicle->addWheel(btVector3(-1.f, 1.0f, 1.5f), LwheelDirection, LwheelAxis, suspensionRestLength, wheelRadius, tuning, false);
vehicle->addWheel(btVector3(-3.f, 1.0f, 1.5f), LwheelDirection, LwheelAxis,suspensionRestLengthShorter, wheelRadius, tuning, false);
//vehicle->addWheel(btVector3(-2.f, .0f, 1.0f), LwheelDirection, LwheelAxis,suspensionRestLengthShort, wheelRadius, tuning, false);
//vehicle->addWheel(btVector3(-1.5f, 1.0f, 1.0f), LwheelDirection, LwheelAxis,suspensionRestLengthShorter, wheelRadius, tuning, false);

vehicle->addWheel(btVector3(3.f, 1.0f, -1.5f), LwheelDirection, LwheelAxis, suspensionRestLengthShorter, wheelRadius, tuning, false);
vehicle->addWheel(btVector3(1.f, 1.0f, -1.5f), LwheelDirection, LwheelAxis, suspensionRestLength, wheelRadius, tuning, false);
vehicle->addWheel(btVector3(0.f, 1.0f, -1.5f), LwheelDirection, LwheelAxis, suspensionRestLength, wheelRadius, tuning, false);
vehicle->addWheel(btVector3(-1.f, 1.0f, -1.5f), LwheelDirection, LwheelAxis, suspensionRestLength, wheelRadius, tuning, false);
vehicle->addWheel(btVector3(-3.f, 1.0f, -1.5f), LwheelDirection, LwheelAxis,suspensionRestLengthShorter, wheelRadius, tuning, false);
//vehicle->addWheel(btVector3(-2.f, .0f, -1.0f), LwheelDirection, LwheelAxis,suspensionRestLengthShort, wheelRadius, tuning, false);
//vehicle->addWheel(btVector3(-1.5f, 1.0f, -1.0f), LwheelDirection, LwheelAxis,suspensionRestLengthShorter, wheelRadius, tuning, false);


for(int i = 0; i< vehicle->getNumWheels();i++)
{
btWheelInfo wheel = vehicle->getWheelInfo(i);


}


World->addAction(vehicle);

Player.phytank = vehicle;



    {

       /* btDefaultVehicleRaycaster *vehicleRayCaster = new btDefaultVehicleRaycaster(dWorld);
        btRaycastVehicle *vehicle = new btRaycastVehicle(vehicleTuning, vehicleRigidBody, vehicleRayCaster);

        // never deactivate vehicle
        vehicleRigidBody->setActivationState(DISABLE_DEACTIVATION);
        World->addVehicle(vehicle);

        float connectionHeight = 1.2f;
        bool isFrontWheel = true;

        vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex); // 0, 1, 2

        // add wheels
        // front left
        btVector3 connectionPointCS0(CUBE_HALF_EXTENT-(0.3*wheelWidth), connectionHeight, 2*CUBE_HALF_EXTENT-wheelRadius);
        vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, vehicleTuning, isFrontWheel);
        // front right
        connectionPointCS0 = btVector3(-CUBE_HALF_EXTENT+(0.3*wheelWidth), connectionHeight, 2*CUBE_HALF_EXTENT-wheelRadius);
        vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, vehicleTuning, isFrontWheel);
        isFrontWheel = false;
        // rear right
      //  connectionPointCS0 = btVector3(-CUBE_HALF_EXTENT+(0.3*wheelWidth), connectionHeight, -2*CUBE_HALF_EXTENT+wheelRadius);
       // vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, vehicleTuning, isFrontWheel);
        // rear left
        //connectionPointCS0 = btVector3(CUBE_HALF_EXTENT-(0.3*wheelWidth), connectionHeight, -2*CUBE_HALF_EXTENT+wheelRadius);
       // vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, vehicleTuning, isFrontWheel);

//        for (int i = 0; i < vehicle->getNumWheels(); i++)
//        {
//            btWheelInfo& wheel = vehicle->getWheelInfo(i);
//            wheel.m_suspensionStiffness = suspensionStiffness;
//            wheel.m_wheelsDampingRelaxation = suspensionDamping;
//            wheel.m_wheelsDampingCompression = suspensionCompression;
//            wheel.m_frictionSlip = wheelFriction;
//            wheel.m_rollInfluence = rollInfluence;
//        }

*/
    }



}
btTriangleMesh *makeMesh(irr::scene::ISceneNode * node)
{
using irr::core::vector2df;
    using irr::core::vector3df;
    using irr::scene::IMesh;
    using irr::scene::IMeshBuffer;
    using irr::scene::IMeshSceneNode;
    using irr::scene::IAnimatedMeshSceneNode;
btTriangleMesh * btmesh = new btTriangleMesh();

    // Handy lambda for converting from irr::vector to btVector
  //  std::cout << "lambda " <<".\n";
    auto toBtVector = [ &]( const vector3df & vec,const vector3df & scal ) -> btVector3
    {
        btVector3 bt( vec.X * scal.X, vec.Y * scal.Y, vec.Z * scal.Z );

        return bt;
    };

    irr::scene::ESCENE_NODE_TYPE type = node->getType();
//std::cout << "type " << node->getType() << ".\n";
IMesh * mesh = nullptr;
vector3df nodescale;
btVector3 position;
bool havemesh = false;

  //  std::cout << "pointer " <<".\n";
 //   IMesh * mesh =nullptr;
    switch ( type )
    {
        case irr::scene::ESNT_ANIMATED_MESH:
{
                IAnimatedMeshSceneNode * Ameshnode = nullptr;
                Ameshnode = (IAnimatedMeshSceneNode *)node;
                //mesh= (IAnimatedMeshSceneNode *)node->getMesh();
                std::cout << "animatedmeshnode " <<".\n";
                position = toBtVector( Ameshnode->getPosition(),Ameshnode->getScale() );

                mesh = Ameshnode->getMesh();
                nodescale = Ameshnode->getScale();
                havemesh = true;

            break;
            }
        case irr::scene::ESNT_MESH:
{
                IMeshSceneNode * meshnode = nullptr;
                meshnode = (IMeshSceneNode *)node;
               // mesh= (IMeshSceneNode *)node->getMesh();
                std::cout << "mesh node " <<".\n"<<"sale: "<< meshnode->getScale().X << ".\n" ;
                position = toBtVector( meshnode->getPosition(),meshnode->getScale() );

                mesh = meshnode->getMesh();
                nodescale = meshnode->getScale();
                havemesh = true;


            break;
            }
        case irr::scene::ESNT_TERRAIN:
{
                ITerrainSceneNode * Tmeshnode = nullptr;
                Tmeshnode = (ITerrainSceneNode *)node;
               // mesh= (ITerrainSceneNode *)node->getMesh();
                std::cout << "Terrain mesh node " <<".\n";
                 position = toBtVector( Tmeshnode->getPosition(),Tmeshnode->getScale() );
                 mesh = Tmeshnode->getMesh();
                 nodescale = Tmeshnode->getScale();
                 havemesh = true;


        break;
        }
            default:

            //IMeshSceneNode *mesh =nullptr;


            break;
    }

    if ( havemesh )
    {
    std::cout << "if meshnode " <<".\n";
       // const vector3df nodescale = meshnode->getScale();

       // IMesh * mesh = meshnode->getMesh();
        const size_t buffercount = mesh->getMeshBufferCount();
        std::cout << "buffcount "<< buffercount <<".\n";

        // Save position
       // btVector3 position = toBtVector( meshnode->getPosition() );

        // Save data here

        std::vector<irr::video::S3DVertex>    verticesList;
        std::vector<int>                  indicesList;

        for ( size_t i=0; i<buffercount; ++i )
        {
            // Current meshbuffer
            IMeshBuffer * buffer = mesh->getMeshBuffer( i );
            //std::cout << "i " << i <<".\n";

            // EVT_STANDARD -> video::S3DVertex
            // EVT_2TCOORDS -> video::S3DVertex2TCoords
            // EVT_TANGENTS -> video::S3DVertexTangents
            const irr::video::E_VERTEX_TYPE vertexType      = buffer->getVertexType();

            // EIT_16BIT
            // EIT_32BIT
            const irr::video::E_INDEX_TYPE  indexType       = buffer->getIndexType();

            // Get working data
            const size_t numVerts       = buffer->getVertexCount();
            const size_t numInd         = buffer->getIndexCount();
            //std::cout << "getVertex Count "<< buffer->getVertexCount() <<".\n";
           // std::cout << "getIndex Count "<< buffer->getIndexCount() <<".\n";

            // Resize save buffers
            verticesList.resize( verticesList.size() + numVerts );
            indicesList.resize( indicesList.size() + numInd );

            void * vertices             = buffer->getVertices();
            void * indices              = buffer->getIndices();

            irr::video::S3DVertex           * standard      = reinterpret_cast<irr::video::S3DVertex*>( vertices );
            irr::video::S3DVertex2TCoords   * two2coords    = reinterpret_cast<irr::video::S3DVertex2TCoords*>( vertices );
            irr::video::S3DVertexTangents   * tangents      = reinterpret_cast<irr::video::S3DVertexTangents*>( vertices );

            int16_t * ind16     = reinterpret_cast<int16_t*>( indices );
            int32_t * ind32     = reinterpret_cast<int32_t*>( indices );

            for ( size_t v = 0; v < numVerts; ++v )
            {
                auto & vert = verticesList[ v ];

                switch ( vertexType )
                {
                    case irr::video::EVT_STANDARD:
                        {
                            const auto & irrv = standard[ v ];

                            vert = irrv;
                            //std::cout << "EVT_STANDARD "<< ".\n";
                        }
                        break;
                    case irr::video::EVT_2TCOORDS:
                        {
                            const auto & irrv = two2coords[ v ];
                            (void)irrv;
                         //   std::cout << "EVT_2TCOORDS "<< ".\n";

                            // Not implemented
                        }
                        break;
                    case irr::video::EVT_TANGENTS:
                        {
                            const auto & irrv = tangents[ v ];
                            (void)irrv;

                      //  std::cout << "EVT_TANGENTS "<< ".\n";
                            // Not implemented
                        }
                        break;
                    default:
                     //   BOOST_ASSERT( 0 && "unknown vertex type" );
                    // std::cout << "nknown vertex type "<< ".\n";
                     break;
                }

            }

            for ( size_t n = 0; n < numInd; ++n )
            {
                auto & index = indicesList[ n ];

                switch ( indexType )
                {
                    case irr::video::EIT_16BIT:
                    {
                        index = ind16[ n ];
                    }
                        break;
                    case irr::video::EIT_32BIT:
                    {
                        index = ind32[ n ];
                    }
                        break;
                    default:
                      //  BOOST_ASSERT( 0 && "unkown index type" );
                      break;
                }

            }

        }

        // Make bullet rigid body
        if ( ! verticesList.empty() && ! indicesList.empty() )
        {
            // Working numbers
            const size_t numIndices     = indicesList.size();
            const size_t numTriangles   = numIndices / 3;

            // Error checking
           // BOOST_ASSERT( numTriangles * 3 == numIndices && "Number of indices does not make complete triangles" );

            // Create triangles


            // Build btTriangleMesh
            for ( size_t i=0; i<numIndices; i+=3 )
            {
                const btVector3 &A = toBtVector( verticesList[ indicesList[ i+0 ] ].Pos ,nodescale );
                const btVector3 &B = toBtVector( verticesList[ indicesList[ i+1 ] ].Pos ,nodescale );
                const btVector3 &C = toBtVector( verticesList[ indicesList[ i+2 ] ].Pos ,nodescale );

                bool removeDuplicateVertices = true;
                btmesh->addTriangle( A, B, C, removeDuplicateVertices );

            }




}

}
return btmesh;
}
btRigidBody * makeBulletMeshFromIrrlichtNode( const irr::scene::ISceneNode * node,const btScalar TMass)
{
//std::cout << "make bt obj " <<".\n";
    using irr::core::vector2df;
    using irr::core::vector3df;
    using irr::scene::IMesh;
    using irr::scene::IMeshBuffer;
    using irr::scene::IMeshSceneNode;
    using irr::scene::IAnimatedMeshSceneNode;


    // Handy lambda for converting from irr::vector to btVector
  //  std::cout << "lambda " <<".\n";


    irr::scene::ESCENE_NODE_TYPE type = node->getType();
//std::cout << "type " << node->getType() << ".\n";
IMesh * mesh = nullptr;
vector3df nodescale;
btVector3 position;
bool havemesh = false;
    btRigidBody * body = nullptr;
  //  std::cout << "pointer " <<".\n";
 //   IMesh * mesh =nullptr;
    switch ( type )
    {
        case irr::scene::ESNT_ANIMATED_MESH:
{
                IAnimatedMeshSceneNode * Ameshnode = nullptr;
                Ameshnode = (IAnimatedMeshSceneNode *)node;
                //mesh= (IAnimatedMeshSceneNode *)node->getMesh();
               // std::cout << "animatedmeshnode " <<".\n";
                position = toBtVector( Ameshnode->getPosition(),Ameshnode->getScale() );

                mesh = Ameshnode->getMesh();
                nodescale = Ameshnode->getScale();
                havemesh = true;

            break;
            }
        case irr::scene::ESNT_MESH:
{
                IMeshSceneNode * meshnode = nullptr;
                meshnode = (IMeshSceneNode *)node;
               // mesh= (IMeshSceneNode *)node->getMesh();
              //  std::cout << "mesh node " <<".\n"<<"sale: "<< meshnode->getScale().X << ".\n" ;
                position = toBtVector( meshnode->getPosition(),meshnode->getScale() );

                mesh = meshnode->getMesh();
                nodescale = meshnode->getScale();
                havemesh = true;


            break;
            }
        case irr::scene::ESNT_TERRAIN:
{
                ITerrainSceneNode * Tmeshnode = nullptr;
                Tmeshnode = (ITerrainSceneNode *)node;
               // mesh= (ITerrainSceneNode *)node->getMesh();
            //    std::cout << "Terrain mesh node " <<".\n";
                 position = toBtVector( Tmeshnode->getPosition(),Tmeshnode->getScale() );
                 mesh = Tmeshnode->getMesh();
                 nodescale = Tmeshnode->getScale();
                 havemesh = true;


        break;
        }
            default:

            //IMeshSceneNode *mesh =nullptr;


            break;
    }

    if ( havemesh )
    {
  //  std::cout << "if meshnode " <<".\n";
       // const vector3df nodescale = meshnode->getScale();

       // IMesh * mesh = meshnode->getMesh();
        const size_t buffercount = mesh->getMeshBufferCount();
      //  std::cout << "buffcount "<< buffercount <<".\n";

        // Save position
       // btVector3 position = toBtVector( meshnode->getPosition() );

        // Save data here

        std::vector<irr::video::S3DVertex>    verticesList;
        std::vector<int>                  indicesList;

        for ( size_t i=0; i<buffercount; ++i )
        {
            // Current meshbuffer
            IMeshBuffer * buffer = mesh->getMeshBuffer( i );
            //std::cout << "i " << i <<".\n";

            // EVT_STANDARD -> video::S3DVertex
            // EVT_2TCOORDS -> video::S3DVertex2TCoords
            // EVT_TANGENTS -> video::S3DVertexTangents
            const irr::video::E_VERTEX_TYPE vertexType      = buffer->getVertexType();

            // EIT_16BIT
            // EIT_32BIT
            const irr::video::E_INDEX_TYPE  indexType       = buffer->getIndexType();

            // Get working data
            const size_t numVerts       = buffer->getVertexCount();
            const size_t numInd         = buffer->getIndexCount();
       //     std::cout << "getVertex Count "<< buffer->getVertexCount() <<".\n";
       //     std::cout << "getIndex Count "<< buffer->getIndexCount() <<".\n";

            // Resize save buffers
            verticesList.resize( verticesList.size() + numVerts );
            indicesList.resize( indicesList.size() + numInd );

            void * vertices             = buffer->getVertices();
            void * indices              = buffer->getIndices();

            irr::video::S3DVertex           * standard      = reinterpret_cast<irr::video::S3DVertex*>( vertices );
            irr::video::S3DVertex2TCoords   * two2coords    = reinterpret_cast<irr::video::S3DVertex2TCoords*>( vertices );
            irr::video::S3DVertexTangents   * tangents      = reinterpret_cast<irr::video::S3DVertexTangents*>( vertices );

            int16_t * ind16     = reinterpret_cast<int16_t*>( indices );
            int32_t * ind32     = reinterpret_cast<int32_t*>( indices );

            for ( size_t v = 0; v < numVerts; ++v )
            {
                auto & vert = verticesList[ v ];

                switch ( vertexType )
                {
                    case irr::video::EVT_STANDARD:
                        {
                            const auto & irrv = standard[ v ];

                            vert = irrv;
                            //std::cout << "EVT_STANDARD "<< ".\n";
                        }
                        break;
                    case irr::video::EVT_2TCOORDS:
                        {
                            const auto & irrv = two2coords[ v ];
                            (void)irrv;
                          // std::cout << "EVT_2TCOORDS "<< ".\n";

                            // Not implemented
                        }
                        break;
                    case irr::video::EVT_TANGENTS:
                        {
                            const auto & irrv = tangents[ v ];
                            (void)irrv;

                    //    std::cout << "EVT_TANGENTS "<< ".\n";
                            // Not implemented
                        }
                        break;
                    default:
                     //   BOOST_ASSERT( 0 && "unknown vertex type" );
                  //   std::cout << "nknown vertex type "<< ".\n";
                     break;
                }

            }

            for ( size_t n = 0; n < numInd; ++n )
            {
                auto & index = indicesList[ n ];

                switch ( indexType )
                {
                    case irr::video::EIT_16BIT:
                    {
                        index = ind16[ n ];
                    }
                        break;
                    case irr::video::EIT_32BIT:
                    {
                        index = ind32[ n ];
                    }
                        break;
                    default:
                      //  BOOST_ASSERT( 0 && "unkown index type" );
                      break;
                }

            }

        }

        // Make bullet rigid body
        if ( ! verticesList.empty() && ! indicesList.empty() )
        {
            // Working numbers
            const size_t numIndices     = indicesList.size();
            const size_t numTriangles   = numIndices / 3;

            // Error checking
           // BOOST_ASSERT( numTriangles * 3 == numIndices && "Number of indices does not make complete triangles" );

            // Create triangles
            btTriangleMesh * btmesh = new btTriangleMesh();

            // Build btTriangleMesh
            for ( size_t i=0; i<numIndices; i+=3 )
            {
                const btVector3 &A = toBtVector( verticesList[ indicesList[ i+0 ] ].Pos ,nodescale );
                const btVector3 &B = toBtVector( verticesList[ indicesList[ i+1 ] ].Pos ,nodescale );
                const btVector3 &C = toBtVector( verticesList[ indicesList[ i+2 ] ].Pos ,nodescale );

                bool removeDuplicateVertices = true;
                btmesh->addTriangle( A, B, C, removeDuplicateVertices );

            }
        //    std::cout << "addtriangle "<< numTriangles <<".\n";

            // Give it a default MotionState
            btTransform transform;
      //      std::cout << "Tarnsform " <<".\n";
            transform.setIdentity();
      //      std::cout << "ident " <<".\n";
            transform.setOrigin( position );
        //    std::cout << "origin" <<".\n";
            btDefaultMotionState *motionState = new btDefaultMotionState( transform );
         //   std::cout << "motionstate" << ".\n";
            // Create the shape
            btVector3 LocalInertia;
          //  btVector3 nullInertia;
            if(TMass==0)
            {
            btCollisionShape *btShape = new btBvhTriangleMeshShape( btmesh, true );
            btShape->setMargin( 0.05f );
            btShape->calculateLocalInertia(TMass,LocalInertia);
          //  btShape->calculateLocalInertia(200,nullInertia);
         //   btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
            btRigidBody::btRigidBodyConstructionInfo info= btRigidBody::btRigidBodyConstructionInfo(TMass,motionState,btShape,LocalInertia);

            info.m_restitution= 1.0f;
            info.m_friction = 0.5f;



            // Create the rigid body object
            //btScalar mass = 0.0f;
//            if(TMass == 0)
//            {body = new btRigidBody( info );
//            body->setUserPointer((void *)(node));
//            std::cout << "mass 0" <<".\n";
//            World->addRigidBody(body);
//            Objects.push_back(body);
//            }
//            else
//           {
            body = new btRigidBody( info );
            //std::cout << "body" <<".\n";
            //body->setActivationState(DISABLE_DEACTIVATION);

            body->setUserPointer((void *)(node));
            //std::cout << "pointer"<<".\n";
            World->addRigidBody(body);
           // std::cout << " add rigid to world " <<".\n";
            Objects.push_back(body);
            //std::cout << " finish " <<".\n";
            }
            else
            {
            //btStridingMeshInterface* bmeshinfo = new btStridingMeshInterface();

            btCollisionShape *btShape = new btConvexTriangleMeshShape( btmesh);
            btShape->setMargin( 0.05f );
            btShape->calculateLocalInertia(TMass,LocalInertia);
          //  btShape->calculateLocalInertia(200,nullInertia);
         //   btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
            btRigidBody::btRigidBodyConstructionInfo info= btRigidBody::btRigidBodyConstructionInfo(TMass,motionState,btShape,LocalInertia);

            info.m_restitution= 1.0f;
            info.m_friction = 0.5f;



            // Create the rigid body object
            //btScalar mass = 0.0f;
//            if(TMass == 0)
//            {body = new btRigidBody( info );
//            body->setUserPointer((void *)(node));
//            std::cout << "mass 0" <<".\n";
//            World->addRigidBody(body);
//            Objects.push_back(body);
//            }
//            else
//           {
            body = new btRigidBody( info );
            //std::cout << "body" <<".\n";
            //body->setActivationState(DISABLE_DEACTIVATION);

            body->setUserPointer((void *)(node));
            //std::cout << "pointer"<<".\n";
            World->addRigidBody(body);
           // std::cout << " add rigid to world " <<".\n";
            Objects.push_back(body);
            //std::cout << " finish " <<".\n";
            }

           //}
        }

    }

    // Return Bullet rigid body
    return body;
}

void ApllayForce(irr::scene::ISceneNode *Tnode)
{
std::cout << "force " <<".\n";
btRigidBody *TObject;
btVector3 force = btVector3(100.,0.,0.);


for(core::list<btRigidBody *>::Iterator it = Objects.begin(); it != Objects.end(); ++it) {


		//UpdateRender(*Iterator);
		scene::ISceneNode *Node = static_cast<scene::ISceneNode *>((*it)->getUserPointer());
		TObject = *it;

		// Set position
		if (Node->getID() == Tnode->getID())
		{
		std::cout << "node match " <<".\n";
		btQuaternion worldQuat(TObject->getOrientation());
		btMatrix3x3& boxRot = TObject->getWorldTransform().getBasis();
		btVector3 appforce = boxRot*force;

		TObject->setActivationState(DISABLE_DEACTIVATION);
		TObject->applyImpulse(appforce,TObject->getCenterOfMassPosition());
		//debugdraw
		btVector3 com = TObject->getCenterOfMassPosition();
		matrix4 id;
        id.makeIdentity();
        irr::video::SMaterial debugmaterial;
        debugmaterial.Lighting = false;
        irrDriver->setMaterial(debugmaterial);
        irrDriver->setTransform(video::ETS_WORLD, id);
        irrDriver->draw3DLine (core::vector3df(com[0],com[1],com[2]), core::vector3df(appforce[0],appforce[1],appforce[2]), video::SColor(0,100,180,100)) ;
		std::cout << "from "<< com[0]<<" "<< com[1]<<" "<< com[2]<<" "<< com[3]<<".\n";
		std::cout << "to " << appforce[0]<<" "<< appforce[1]<<" "<< appforce[2]<<" "<< appforce[3]<<".\n";
		//Player->Camera->setPosition(core::vector3df(com[0],com[1],com[2]));
		Player.DomeCamera->setTarget(core::vector3df(com[0],com[1],com[2]));        //(core::vector3df(appforce[0],appforce[1],appforce[2]));
		}

	}
	}

//Resolution
const int ResX=1280;
const int ResY=1024;
const bool fullScreen=false;

//Use SplitScreen?
bool SplitScreen=true;

int main(int argc, char** argv)
{
    // Initialize irrlicht
	EventReceiverClass Receiver;
	irrDevice = createDevice(video::EDT_OPENGL, dimension2d<u32>(ResX, ResY), 32, false, false, false, &Receiver);
	irrGUI = irrDevice->getGUIEnvironment();
	irrTimer = irrDevice->getTimer();
	irrScene = irrDevice->getSceneManager();
	irrDriver = irrDevice->getVideoDriver();
	irrFile = irrDevice->getFileSystem();

	irrDevice->getCursorControl()->setVisible(0);

	// Initialize bullet
	btDefaultCollisionConfiguration *CollisionConfiguration = new btDefaultCollisionConfiguration();
	btBroadphaseInterface *BroadPhase =new btDbvtBroadphase(); // new btAxisSweep3(btVector3(-1000, -1000, -1000), btVector3(1000, 1000, 1000));//
	btCollisionDispatcher *Dispatcher = new btCollisionDispatcher(CollisionConfiguration);
	btSequentialImpulseConstraintSolver *Solver = new btSequentialImpulseConstraintSolver();
	World = new btDiscreteDynamicsWorld(Dispatcher, BroadPhase, Solver, CollisionConfiguration);
	World->setGravity(btVector3(0.,-9.81,0.));
    //World->setGravity(btVector3(0.,0.,0.));
	// Add camera
//	BulletDebugRender *m_BulletDebug = new BulletDebugRender;
     // Bullet physics world
 //   World->setDebugDrawer(&mBulletDebug);

    Dummy = new tank(irrScene,irr::core::vector3df(0.f,40.f,0.f));
	Player = tank(irrScene,irr::core::vector3df(20.,40.,10.),200);
    scene::IMeshSceneNode* terrain = irrScene->addMeshSceneNode(
    irrScene->getMesh("/home/wbr/SteamTanks/bin/Debug/Models/teren.obj")
    ,0,1//);
     ,core::vector3df(0.f,0.0,0.0f),
     core::vector3df(0.f,0.0,0.0f),
     core::vector3df(500.f,5.0,500.0f));
//

       // "/home/wbr/SteamTanks/bin/Debug/Models/teren.obj",   //"./MEDIA/Terr/mountain/mount.obj"),
       // 0,                  // parent node
       //-1,                 // node id
        //*terrpos,     // position
       // core::vector3df(-100.f, 0.f, -100.f),
      //  core::vector3df(0.f, 0.f, 0.f),     // rotation
      //  core::vector3df(40.f, 1.f, 40.f)  // scale
//                      // smoothFactor
   //         ,                                     //);
//        //core::vector3df(0.f, 0.f, 0.f),     // position
//        //core::vector3df(0.f, 0.f, 0.f),     // rotation
//       // core::vector3df(40.1f, 4.01f, 40.1f),  // scale
       // video::SColor ( 255, 255, 255, 255 ),   // vertexColor
       // 5,                  // maxLOD
       // scene::ETPS_17,             // patchSize
      //  4                   // smoothFactor
//       );
//
//

   terrain->setMaterialFlag(irr::video::EMF_LIGHTING,false);
   terrain->setVisible(draw_irr);
   //CreateBox(terrain,terrain->getScale(),0);
   std::cout << "create box " <<".\n";
   makeBulletMeshFromIrrlichtNode(Dummy->node,200);
std::cout << "igrac" <<".\n";
makeBulletMeshFromIrrlichtNode(terrain,0);
std::cout << "teren " <<".\n";
BulletDebugRender m_BulletDebug;
 makeVehlice(Player,Player.mass);
 debugCamera = irrScene->addCameraSceneNodeFPS(0,100.f,1.f);
m_BulletDebug.setDebugMode(draw_phy);
irr::gui::IGUIStaticText *fps = irrGUI->addStaticText(L"fps deltatime", rect<s32>(50,20,350,80), true);
World->setDebugDrawer(&m_BulletDebug);

u32 TimeStamp = irrTimer->getTime(), DeltaTime = 0;
irrScene->setActiveCamera(debugCamera);
	while(!Done) {

	    Player.updateCamera();

        if (draw_irr)
		{
		terrain->setVisible(true);
		}
		if (!draw_irr)
		{
		terrain->setVisible(false);
		}




		DeltaTime = irrTimer->getTime() - TimeStamp;
		TimeStamp = irrTimer->getTime();

		UpdatePhysics(DeltaTime);

		irrDriver->setViewPort(rect<s32>(0,0,ResX,ResY));
		irrDriver->beginScene(true, true, SColor(0, 255, 255, 255));

		if (SplitScreen)
        {
            //Activate camera1
            if (draw_phy)
		{
		World->debugDrawWorld();
		}
            irrScene->setActiveCamera(Player.DomeCamera);
            Player.updateCamera();
            //Set viewpoint to the first quarter (left top)
            irrDriver->setViewPort(rect<s32>(0,0,ResX,ResY/2));
            //Draw scene
            irrScene->drawAll();
            //Activate camera2
            irrScene->setActiveCamera(Player.DriverCamera);
            Player.updateCamera();
            if (draw_phy)
		{
		World->debugDrawWorld();
		}
            //Set viewpoint to the third quarter (left bottom)
            irrDriver->setViewPort(rect<s32>(0,ResY/2,ResX/2,ResY));
            //Draw scene
            irrScene->drawAll();
            irrScene->setActiveCamera(debugCamera);
            Player.updateCamera();
            if (draw_phy)
		{
		World->debugDrawWorld();
		}

            //Set viewport the last quarter (right bottom)
            irrDriver->setViewPort(rect<s32>(ResX/2,ResY/2,ResX,ResY));
            irrScene->drawAll();
        }

        irrScene->setActiveCamera(debugCamera);
	{
	if (draw_phy)
		{
		World->debugDrawWorld();
		}
            Player.updateCamera();
            core::stringw str(L"FPS: ");
            str.append(core::stringw(irrDriver->getFPS()));
            str += L" deltatime: ";
            str.append(core::stringw(DeltaTime));
            fps->setText(str.c_str());
      		fps->setText(str.c_str());
    }
//irrDriver->draw3DLine(Player->DomeCamera->getAbsolutePosition(),Player->DomeCamera->getTarget(),irr::video::SColor(0,255,0,0));

		irrScene->drawAll();
		irrGUI->drawAll();
		irrDriver->endScene();
		irrDevice->run();

//std::cout << "Rwheel force "<<Player.REngineForce <<
//" Lwheel force "<<Player.LEngineForce << ".\n";

std::cout << "Player pos "<<Player.body->getCenterOfMassPosition().x() <<
" Player pos "<<Player.body->getCenterOfMassPosition().y() <<
 " Player pos "<<Player.body->getCenterOfMassPosition().z() <<".\n";
// std::cout << "PHY pos "<<Player.body->getCenterOfMassTransform()->getX() <<
//" PHY pos "<<Player.body->ggetCenterOfMassTransform().getY() <<
// " PHY pos "<<Player.body->getCenterOfMassTransform().getZ() <<".\n";
//std::cout << "camera  "<<Player->DomeCamera->getTargetAndRotationBinding() << ".\n";

	}

	ClearObjects();
	delete World;
	delete Solver;
	delete Dispatcher;
	delete BroadPhase;
	delete CollisionConfiguration;

	irrDevice->drop();

	return 0;
}
void BulletDebugRender::drawLine( const btVector3& from,const btVector3& to, const btVector3& color )
{

    SColorf fromC;
    fromC.set(color[0],color[1],color[2],color[3]);
    SColorf toC;
    toC.set(color[0],color[1],color[2],color[3]);
    irr::video::SColor Scolor;
    Scolor = fromC.toSColor();





    matrix4 id;
    id.makeIdentity();
    irr::video::SMaterial debugmaterial;
    debugmaterial.Lighting = false;
    irrDriver->setMaterial(debugmaterial);
    irrDriver->setTransform(video::ETS_WORLD, id);
    irrDriver->draw3DLine (core::vector3df(from[0],from[1],from[2]), core::vector3df(to[0],to[1],to[2]), fromC.toSColor()  ) ;
   // std::cout<<"alpha " << Scolor.getAlpha() <<" red" <<Scolor.getRed() <<" green " << Scolor.getGreen()<<" blue "<< Scolor.getBlue()<<".\n";
}
//void BulletDebugRender::drawLine( const btVector3& from,const btVector3& to, const btVector3& fromColor, const btVector3& toColor )
//{
//
//    SColorf fromC;
//    fromC.set(fromColor[0],fromColor[1],fromColor[2],fromColor[3]);
//    SColorf toC;
//    toC.set(toColor[0],toColor[1],toColor[2],toColor[3]);
//
//
//
//
//    matrix4 id;
//    id.makeIdentity();
//    irrDriver->setTransform(video::ETS_WORLD, id);
//    irrDriver->draw3DLine (core::vector3df(from[0],from[1],from[2]), core::vector3df(to[0],to[1],to[2]), irr::video::SColor(0,fromColor[0]+254,fromColor[1]+254,fromColor[2]+254)  ) ;
//    //std::cout<<"color " << color[0] <<color[1] << color[2]<< color[3]<<".\n";
////std::cout<<"color " << fromC.toSColor <<".\n";
//}

// Implementation of Graphics::drawLine

void ClearObjects() {

	for(list<btRigidBody *>::Iterator Iterator = Objects.begin(); Iterator != Objects.end(); ++Iterator) {
		btRigidBody *Object = *Iterator;

		// Delete irrlicht node
		ISceneNode *Node = static_cast<ISceneNode *>(Object->getUserPointer());
		Node->remove();

		// Remove the object from the world
		World->removeRigidBody(Object);

		// Free memory
		delete Object->getMotionState();
		delete Object->getCollisionShape();
		delete Object;
	}

	Objects.clear();
}



	 bool EventReceiverClass::OnEvent(const SEvent &TEvent) {

		if(TEvent.EventType == EET_KEY_INPUT_EVENT && !TEvent.KeyInput.PressedDown) {
			switch(TEvent.KeyInput.Key) {
				case KEY_ESCAPE:
					Done = true;
				break;
				case KEY_KEY_Q:
				draw_phy = false;
					//CreateBox(btVector3(GetRandInt(10) - 5.0f, 7.0f, GetRandInt(10) - 5.0f), vector3df(GetRandInt(3) + 0.5f, GetRandInt(3) + 0.5f, GetRandInt(3) + 0.5f), 1.0f);
				break;
				case KEY_HOME:
				std::cout<<"engine Left" <<".\n";
				Player.setLeftTrack(10);

				break;
				case KEY_PRIOR:
				Player.setRightTrack(10);

				break;
				case KEY_END:
				Player.setLeftTrack(-10);

				break;
				case KEY_NEXT:
				Player.setRightTrack(-10);
					//CreateSphere(btVector3(GetRandInt(10) - 5.0f, 7.0f, GetRandInt(10) - 5.0f), GetRandInt(5) / 5.0f + 0.2f, 1.0f);
				break;

				case KEY_KEY_S:
				//ApllayForce(Dummy->node);
				/*for (int i = 0; i < Player->phytank->getNumWheels(); i++)
				{
				Player->phytank->applyEngineForce(0.f,i);
				Player->phytank->setBrake(0.f,i);
				}*/
//				for (int i = 0; i < Player->phytank->getNumWheels(); i++)
//				{
//				Player->phytank->applyEngineForce(0.f,i);
//				Player->phytank->setBrake(0.f,i);
//				}
//
//
//				for (int i = 0; i < Player->phytank->getNumWheels(); i++)
//				{
//				Player->phytank->applyEngineForce(-50.f,i);
//				Player->phytank->setBrake(0.f,i);
//				}
//
//				//Player->phytank->applyEngineForce(-50.f,2);
//				//Player->phytank->applyEngineForce(-50.f,3);
//
//				std::cout<<" Left foward"<<".\n";




					//CreateStartScene();
				break;
				case KEY_KEY_W:
				draw_phy = true;
//				for (int i = 0; i < Player->phytank->getNumWheels(); i++)
//				{
//				Player->phytank->applyEngineForce(0.f,i);
//				Player->phytank->setBrake(0.f,i);
//				}
//				for (int i = 0; i < Player->phytank->getNumWheels(); i++)
//				{
//				Player->phytank->applyEngineForce(50.f,i);
//				Player->phytank->setBrake(0.f,i);
//				}



				std::cout<<" R foward"<<".\n";


					//CreateStartScene();
				break;

				case KEY_KEY_D:
//				for (int i = 0; i < Player->phytank->getNumWheels(); i++)
//				{
//				Player->phytank->applyEngineForce(0.f,i);
//				Player->phytank->setBrake(0.f,i);
//				}
//				Player->phytank->applyEngineForce(50.f,5);
//				Player->phytank->applyEngineForce(50.f,6);
//				Player->phytank->applyEngineForce(50.f,7);
//				Player->phytank->applyEngineForce(50.f,8);
//				Player->phytank->applyEngineForce(50.f,9);
//
//				Player->phytank->setBrake(500.f,0);
//				Player->phytank->setBrake(500.f,1);
//				Player->phytank->setBrake(500.f,2);
//				Player->phytank->setBrake(500.f,3);
//				Player->phytank->setBrake(500.f,4);


					//CreateStartScene();
				break;

				case KEY_KEY_A:
				/*
				for (int i = 0; i < Player->phytank->getNumWheels(); i++)
				{
				Player->phytank->applyEngineForce(0.f,i);
				Player->phytank->setBrake(0.f,i);
				}
				Player->phytank->applyEngineForce(50.f,0);
				Player->phytank->applyEngineForce(50.f,1);
				Player->phytank->applyEngineForce(50.f,2);
				Player->phytank->applyEngineForce(50.f,3);
				Player->phytank->applyEngineForce(50.f,4);

				Player->phytank->setBrake(500.f,5);
				Player->phytank->setBrake(500.f,6);
				Player->phytank->setBrake(500.f,7);
				Player->phytank->setBrake(500.f,8);
				Player->phytank->setBrake(500.f,9);
*/



					//CreateStartScene();
				break;
				case KEY_KEY_X:
				for (int i = 0; i < Player.phytank->getNumWheels(); i++)
				{
				Player.phytank->applyEngineForce(0.f,i);
				Player.phytank->setBrake(50.f,i);
				std::cout<< Player.phytank->getCurrentSpeedKmHour()<<"km/h. \n";
				}

				std::cout<<" brake"<<".\n";




					//CreateStartScene();
				break;


				case KEY_KEY_O:
					Player.node->setVisible(true);
					Dummy->node->setVisible(true);
					draw_irr = true;
				//m_BulletDebug.setDebugMode(0);
				break;
				case KEY_KEY_P:
				Player.node->setVisible(false);
				Dummy->node->setVisible(false);
				draw_irr = false;
				//m_BulletDebug.setDebugMode(1);


				//Player->node->setPosition(Player->Camera->getAbsolutePosition());

					//CreateStartScene();
				break;
				case KEY_KEY_C:
				//Player->node->setVisible(false);
				//Dummy->node->setVisible(false);
				//draw_irr = false;
				//m_BulletDebug.setDebugMode(1);
				for (int i = 0; i < Player.phytank->getNumWheels(); i++)
				{

				btWheelInfo info = Player.phytank->getWheelInfo(i);
				std::cout<< info.m_engineForce << ".\n";
				std::cout<< info.m_brake << ".\n";

				}


				//Player->node->setPosition(Player->Camera->getAbsolutePosition());

					//CreateStartScene();
				break;
				case KEY_KEY_T:
					//CreateStartScene();
					//std::cout << irrFile->getWorkingDirectory().c_str() << ".\n";
					std::cout << Player.node->getAbsolutePosition().X <<" "
                << Player.node->getAbsolutePosition().Y <<" "
                << Player.node->getAbsolutePosition().Z <<" "
                << ".\n"
 //<< Player->Camera->getPosition().X <<" "
 //<< Player->Camera->getPosition().Y <<" "
 //<< Player->Camera->getPosition().Z
                << ".\n";
				break;
				case KEY_KEY_M:

                SplitScreen = !SplitScreen;
                return true;

            //Send all other events to camera4


				default:
					return false;
				break;
			}

			return true;
		}

		return false;
	}



