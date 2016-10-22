//#include <irrlicht.h>
//#include <btBulletDynamicsCommon.h>
//#include "funcs.h"
//#include "enums.h"
//#include "motionstate.h"
#include "tank.h"
#include"classes.h"
using namespace irr;



tank::tank(OBJ_typ tyep,btDiscreteDynamicsWorld *World, irr::scene::ISceneManager *scene,irr::core::vector3df position,btScalar Tmass):anchor_object(typ)
{


this->typ = tyep;
irr::io::path filename;
mass = Tmass;
filename = "./Models/Tank/body.obj";
TankNode[RIGIDS_HULL] = scene->addMeshSceneNode(scene->getMesh(filename),0,101, position);
TankNode[RIGIDS_HULL]->setMaterialFlag(irr::video::EMF_LIGHTING,true);
TankNode[RIGIDS_HULL]->setMaterialFlag(irr::video::EMF_FRONT_FACE_CULLING,false);

TankNode[RIGIDS_HULL]->getMaterial(0).SpecularColor.set(0,255,255,0);

{
/*
filename = "/home/wbr/SteamTanks/bin/Debug/Models/Tank/trackR.obj";
TankNode[TANK_LEFTRACK] = scene->addMeshSceneNode(scene->getMesh(filename),0,100,position+core::vector3df(-1.5,-0.65,-0.8));
TankNode[TANK_LEFTRACK]->setMaterialFlag(irr::video::EMF_WIREFRAME,true);
filename = "/home/wbr/SteamTanks/bin/Debug/Models/Tank/trackR.obj";
TankNode[TANK_RIGHTRACK] = scene->addMeshSceneNode(scene->getMesh(filename),0,100,position+core::vector3df(0,-1.,1));
TankNode[TANK_RIGHTRACK]->setMaterialFlag(irr::video::EMF_WIREFRAME,true);
*/
}

filename = "./Models/Tank/dome.obj";
TankNode[RIGIDS_DOME] = scene->addMeshSceneNode(scene->getMesh(filename),0,100,position+core::vector3df(1.51,0.75,0));
TankNode[RIGIDS_DOME]->setMaterialFlag(irr::video::EMF_WIREFRAME,true);
filename = "./Models/Tank/cannon.obj";
TankNode[RIGIDS_CANNON] = scene->addMeshSceneNode(scene->getMesh(filename),0,100,position+core::vector3df(-0.35,0.63,0));
TankNode[RIGIDS_CANNON]->setMaterialFlag(irr::video::EMF_WIREFRAME,true);

filename = "./Models/Tank/wheel.obj";
TankNode[RIGIDS_WHEEL] = scene->addMeshSceneNode(scene->getMesh(filename),0,100,position+core::vector3df(0,0,0));
TankNode[RIGIDS_WHEEL]->setMaterialFlag(irr::video::EMF_WIREFRAME,true);
TankNode[RIGIDS_CANNON]->setMaterialFlag(irr::video::EMF_LIGHTING,false);


//scene->addSphereSceneNode(.5f,16,DomeNode,-1,DomeCamera->getAbsolutePosition());
DriverCamera = scene->addCameraSceneNode(TankNode[RIGIDS_HULL]);
DriverCamera->setPosition(irr::core::vector3df(1.5,1,0));
//DriverCamera->setTarget(DriverCamera->getAbsolutePosition()+irr::core::vector3df(1,0,0));
DriverCamera->bindTargetAndRotation(true);
//CamHelp = scene->addSphereSceneNode(.5f,16,DomeNode,100,);
//CamHelpT = scene->addSphereSceneNode(.1f,16,DomeNode,100,CamHelp->getPosition()+irr::core::vector3df(10.,0,0));
//CamHelpT->setVisible(false);
//CamHelp->setVisible(false);
DomeCamera = scene->addCameraSceneNode(TankNode[RIGIDS_DOME]);
DomeCamera->setPosition(irr::core::vector3df(0,0,0));
//DomeCamera->setTarget(CamHelpT->getAbsolutePosition());
DomeCamera->bindTargetAndRotation(false);
DomeCamera->setNearValue(3);
DomeCamera->setFarValue(1000);
DomeCamera->setFOV(45);

TankNode[RIGIDS_HULL]->setName("Active");

//std::cout << TankNode[RIGIDS_HULL]->getName() <<".\n";
//scene->addSphereSceneNode(.2,16,DomeCamera);
//scene->addSphereSceneNode(.5,16,DriverCamera);
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

createPhy(World);
startupWheels();
}

void tank::createPhy(btDiscreteDynamicsWorld *World)

{
Body_Shapes[RIGIDS_HULL] = new btConvexTriangleMeshShape(makeMesh(this->TankNode[RIGIDS_HULL]));
Body_Shapes[RIGIDS_CANNON] = new btConvexTriangleMeshShape(makeMesh(this->TankNode[RIGIDS_CANNON]));
//Body_Shapes[TANK_LEFTRACK] = new btConvexTriangleMeshShape(makeMesh(this->TankNode[TANK_LEFTRACK]));
//Body_Shapes[TANK_LEFTRACK] = new btCylinderShape(btVector3(.5,.5,.5));
//Body_Shapes[TANK_RIGHTRACK] = new btConvexTriangleMeshShape(makeMesh(this->TankNode[TANK_RIGHTRACK]));
Body_Shapes[RIGIDS_DOME] = new btConvexTriangleMeshShape(makeMesh(this->TankNode[RIGIDS_DOME]));
Body_Shapes[RIGIDS_WHEEL] = new btConvexTriangleMeshShape(makeMesh(this->TankNode[RIGIDS_WHEEL]));
//btTransform offset;
//offset.setIdentity();
//offset.setOrigin(positionOffset);


btTransform transform;
transform.setIdentity();
transform.setOrigin(toBtVector(this->TankNode[RIGIDS_HULL]->getAbsolutePosition(),this->TankNode[RIGIDS_HULL]->getScale()));

Body_Rigs[RIGIDS_HULL] = localCreateRigidBody(btScalar(20000.), transform, Body_Shapes[RIGIDS_HULL],RIGIDS_HULL);
Body_Rigs[RIGIDS_HULL]->setUserPointer(TankNode[RIGIDS_HULL]);
World->addRigidBody(Body_Rigs[RIGIDS_HULL]);
//Objects.push_back(Body_Rigs[RIGIDS_HULL]);
Body_Rigs[RIGIDS_HULL]->setActivationState(DISABLE_DEACTIVATION);
transform.setIdentity();
transform.setOrigin(toBtVector(this->TankNode[RIGIDS_CANNON]->getAbsolutePosition(),this->TankNode[RIGIDS_CANNON]->getScale()));
Body_Rigs[RIGIDS_CANNON] = localCreateRigidBody(btScalar(.1), transform, Body_Shapes[RIGIDS_CANNON],RIGIDS_CANNON);
Body_Rigs[RIGIDS_CANNON]->setActivationState(DISABLE_DEACTIVATION);
World->addRigidBody(Body_Rigs[RIGIDS_CANNON]);
//Objects.push_back(Body_Rigs[RIGIDS_CANNON]);

{
/*
transform.setIdentity();
transform.setOrigin(toBtVector(this->TankNode[TANK_LEFTRACK]->getAbsolutePosition(),this->TankNode[TANK_LEFTRACK]->getScale()));
//transform.setRotation(btQuaternion(SIMD_HALF_PI, 0, 0));
Body_Rigs[TANK_LEFTRACK] = localCreateRigidBody(btScalar(1.), transform, Body_Shapes[TANK_LEFTRACK],TANK_LEFTRACK);
//World->addRigidBody(Body_Rigs[TANK_LEFTRACK]);
//Body_Rigs[TANK_LEFTRACK]->getWorldTransform().getBasis().setEulerZYX(0, 0, 0);
Body_Rigs[TANK_LEFTRACK]->setActivationState(DISABLE_DEACTIVATION);
}
{

transform.setIdentity();
transform.setOrigin(toBtVector(this->TankNode[TANK_RIGHTRACK]->getAbsolutePosition(),this->TankNode[TANK_RIGHTRACK]->getScale()));

Body_Rigs[TANK_RIGHTRACK] = localCreateRigidBody(btScalar(1.), transform, Body_Shapes[TANK_RIGHTRACK],TANK_RIGHTRACK);
//World->addRigidBody(Body_Rigs[TANK_RIGHTRACK]);
Body_Rigs[TANK_RIGHTRACK]->setActivationState(DISABLE_DEACTIVATION);
*/
}

transform.setIdentity();
transform.setOrigin(toBtVector(this->TankNode[RIGIDS_DOME]->getAbsolutePosition(),this->TankNode[RIGIDS_DOME]->getScale()));
//
Body_Rigs[RIGIDS_DOME] = localCreateRigidBody(btScalar(.5), transform, Body_Shapes[RIGIDS_DOME],RIGIDS_DOME);
//Body_Rigs[RIGIDS_DOME]->setUserPointer(this->TankNode[RIGIDS_DOME]);
Body_Rigs[RIGIDS_DOME]->setActivationState(DISABLE_DEACTIVATION);
World->addRigidBody(Body_Rigs[RIGIDS_DOME]);
//Objects.push_back(Body_Rigs[RIGIDS_DOME]);
btVector3 TankAbsolutePos = toBtVector(this->TankNode[RIGIDS_HULL]->getAbsolutePosition(),this->TankNode[RIGIDS_HULL]->getScale());
for (int i=0;i<WHEELS_COUNT;i++)
{
btGeneric6DofSpringConstraint* wheeljoint;
btTransform wheel_trans,spring_trans;
transform.setIdentity();
transform.setOrigin(WHEELS_positions[i]+TankAbsolutePos);
WheelsNode_L[i] = TankNode[RIGIDS_WHEEL]->clone();
{
Wheels_Rigs_L[i] = localCreateRigidWheel(500,transform,Body_Shapes[RIGIDS_WHEEL],WheelsNode_L[i]);
Wheels_Rigs_L[i]->setActivationState(DISABLE_DEACTIVATION);
World->addRigidBody(Wheels_Rigs_L[i]);
//Objects.push_back(Wheels_Rigs_L[i]);


}
{


        spring_trans.setIdentity(); wheel_trans.setIdentity();
	    spring_trans.setOrigin(WHEELS_positions[i]+btVector3(0,0.5,0));
        wheel_trans.setOrigin(btVector3(0,0.,0.));
		wheeljoint = new btGeneric6DofSpringConstraint(*Body_Rigs[RIGIDS_HULL], *Wheels_Rigs_L[i], spring_trans, wheel_trans,true);
        wheeljoint->setLinearUpperLimit(btVector3(0,-.5,0));
        wheeljoint->setLinearLowerLimit(btVector3(0,-.8,0));
        //joint6DOF->setAngularLowerLimit(btVector3(1.f, 0.f, 0.f));
       //joint6DOF->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));
        wheeljoint->setDbgDrawSize(1.);
        wheeljoint->setAngularLowerLimit(btVector3(0,0,1));
        wheeljoint->setAngularUpperLimit(btVector3(0,0,0));
        WheelsSprings_L[i] = wheeljoint;
        wheeljoint->enableSpring(0,true);
        wheeljoint->setStiffness(0,2.f);
        wheeljoint->setDamping(0,20000.f);
        btRotationalLimitMotor* rotLim = wheeljoint->getRotationalLimitMotor(0);
        rotLim->m_enableMotor=true;
        rotLim->m_targetVelocity = 0.f;
        rotLim->m_damping = 20000.f;
        rotLim->m_bounce = 0.f;

        World->addConstraint(WheelsSprings_L[i],true);


}


transform.setIdentity();
transform.setOrigin(WHEELS_positions[i]+TankAbsolutePos);
WheelsNode_R[i] = TankNode[RIGIDS_WHEEL]->clone();
{
Wheels_Rigs_R[i] = localCreateRigidWheel(500.,transform,Body_Shapes[RIGIDS_WHEEL],WheelsNode_R[i]);
Wheels_Rigs_R[i]->setActivationState(DISABLE_DEACTIVATION);
World->addRigidBody(Wheels_Rigs_R[i]);
//Objects.push_back(Wheels_Rigs_R[i]);

}
        spring_trans.setIdentity(); wheel_trans.setIdentity();
	    spring_trans.setOrigin(WHEELS_positions[i]+btVector3(0,0.5,-2));
        wheel_trans.setOrigin(btVector3(0,0.,0.));
		wheeljoint = new btGeneric6DofSpringConstraint(*Body_Rigs[RIGIDS_HULL], *Wheels_Rigs_R[i], spring_trans, wheel_trans,true);
        wheeljoint->setLinearUpperLimit(btVector3(0,-.5,0));
        wheeljoint->setLinearLowerLimit(btVector3(0,-.8,0));
        //joint6DOF->setAngularLowerLimit(btVector3(1.f, 0.f, 0.f));
       //joint6DOF->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));
        wheeljoint->setDbgDrawSize(1.);
        wheeljoint->setAngularLowerLimit(btVector3(0,0,1));
        wheeljoint->setAngularUpperLimit(btVector3(0,0,0));
        WheelsSprings_R[i] = wheeljoint;
        wheeljoint->enableSpring(0,true);
        wheeljoint->setStiffness(0,2.f);
        wheeljoint->setDamping(0,20000000.f);
        btRotationalLimitMotor* rotLim = wheeljoint->getRotationalLimitMotor(0);
        rotLim->m_enableMotor=true;
        rotLim->m_targetVelocity = 0.f;
        rotLim->m_damping = 20000.f;
        rotLim->m_bounce = 0.f;



        World->addConstraint(WheelsSprings_R[i],true);


}

btGeneric6DofSpringConstraint * joint6DOF;
btHingeConstraint * wheel6DOF;
btTransform localA, localB;
bool useLinearReferenceFrameA = true;

{
        localA.setIdentity(); localB.setIdentity();
	    localA.setOrigin(btVector3(1.51,0.75,0));
	    localB.setOrigin(btVector3(0,0,0));
	    localA.getBasis().setEulerZYX( 0, 0, SIMD_HALF_PI );
//	    Body_Rigs[RIGIDS_DOME]->getWorldTransform()
//        .getBasis().setEulerZYX( 0, 0, -SIMD_HALF_PI );
		joint6DOF = new btGeneric6DofSpringConstraint(*Body_Rigs[RIGIDS_HULL], *Body_Rigs[RIGIDS_DOME], localA, localB,useLinearReferenceFrameA);
        joint6DOF->setLinearUpperLimit(btVector3(.3, 0,0));
        joint6DOF->setLinearLowerLimit(btVector3(.2,0,0));
        //joint6DOF->setAngularLowerLimit(btVector3(0.f, 0.f, 0.f));
        //joint6DOF->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));
      //  joint6DOF->setDbgDrawSize(2.);
        joint6DOF->setAngularLowerLimit(btVector3(1,0,0));
        joint6DOF->setAngularUpperLimit(btVector3(0,0,0));

        //joint6DOF->setLimit( 9, -SIMD_HALF_PI, SIMD_HALF_PI );
        Body_Joints[JOINT_DOME] = joint6DOF;
        World->addConstraint(Body_Joints[JOINT_DOME],true);
        //btRotationalLimitMotor* rotLim = joint6DOF->getRotationalLimitMotor(0);
        //rotLim->m_enableMotor = true;

        //joint6DOF->enableSpring(0,true);
}

{

        localA.setIdentity(); localB.setIdentity();
	    localA.setOrigin(btVector3(0.35,0.63,0));
	    localB.setOrigin(btVector3(0,0,0));
	   // localB.getBasis().setEulerZYX( 0, 0, -SIMD_HALF_PI );
		Cannon_Joint = new btConeTwistConstraint(*Body_Rigs[RIGIDS_DOME], *Body_Rigs[RIGIDS_CANNON], localA, localB);
        //Cannon_Joint->setLinearUpperLimit(btVector3(0,0,0));
        //Cannon_Joint->setLinearLowerLimit(btVector3(0,0,0));
        //joint6DOF->setAngularLowerLimit(btVector3(0.f, 0.f, 0.f));
        //joint6DOF->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));
        Cannon_Joint->setDbgDrawSize(1.);
        //Cannon_Joint->setAngularLowerLimit(btVector3(-SIMD_HALF_PI/4,0,0));
        //Cannon_Joint->setAngularUpperLimit(btVector3(SIMD_HALF_PI/4,0,SIMD_HALF_PI/2));

        //Cannon_Joint->enableSpring(0,true);
        Cannon_Joint->setLimit(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON);
        //Cannon_Joint->enableMotor(true);
        Cannon_Joint->setEnabled(true);


        World->addConstraint(Cannon_Joint,true);
        //Cannon_Joint->enableSpring(0,true);
        //Cannon_Joint->setStiffness(0,2000.f);
        //Cannon_Joint->setDamping(0,20.f);

}

{
/*
        localA.setIdentity(); localB.setIdentity();
	    localA.setOrigin(btVector3(-1.5,-0.6,-0.7));
	    localB.setOrigin(btVector3(0,0.,0.));
		joint6DOF = new btGeneric6DofSpringConstraint(*Body_Rigs[RIGIDS_HULL], *Body_Rigs[TANK_LEFTRACK], localA, localB,useLinearReferenceFrameA);
        joint6DOF->setLinearUpperLimit(btVector3(0,-.1,0));
        joint6DOF->setLinearLowerLimit(btVector3(0,-.5,0));
        //joint6DOF->setAngularLowerLimit(btVector3(1.f, 0.f, 0.f));
       //joint6DOF->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));
        joint6DOF->setDbgDrawSize(1.);
        joint6DOF->setAngularLowerLimit(btVector3(0,0,1));
        joint6DOF->setAngularUpperLimit(btVector3(0,0,0));
        Body_Joints[JOINT_LTRACK] = joint6DOF;
        joint6DOF->enableSpring(0,true);
        joint6DOF->setStiffness(0,2000.f);
        joint6DOF->setDamping(0,20.f);



        World->addConstraint(Body_Joints[JOINT_LTRACK],true);
*/
}

{
//        localA.setIdentity(); localB.setIdentity();
//	    localA.setOrigin(btVector3(0,0,-1.));
//	    localB.setOrigin(btVector3(0,0.36,0.));
//        wheel6DOF = new btHingeConstraint(*Body_Rigs[TANK_LEFTRACK], *Body_Rigs[TANK_LEFTRACK], localA, localB,useLinearReferenceFrameA);
//        //wheel6DOF->setLinearUpperLimit(btVector3(0,-.2,0));
//        //wheel6DOF->setLinearLowerLimit(btVector3(0,-.6,0));
//        //joint6DOF->setAngularLowerLimit(btVector3(0.f, 0.f, 0.f));
//        //joint6DOF->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));
//        wheel6DOF->setDbgDrawSize(1.);
//        //wheel6DOF->setAngularLowerLimit(btVector3(0,0,0));
//        //wheel6DOF->setAngularUpperLimit(btVector3(0,0,0));
//        Wheels[WHEELS_L1] = wheel6DOF;
//
//
//
//        World->addConstraint(Wheels[WHEELS_L1],true);

}



}

void tank::move_Cannon(float up , float left )
{
btQuaternion quat;

this->Cannon_Joint->setMotorTargetInConstraintSpace(quat);

}
void tank::rotateTurret(float pos)
{
btRotationalLimitMotor* rotLim = this->Body_Joints[JOINT_DOME]->getRotationalLimitMotor(0);
rotLim->m_enableMotor=true;
if(pos==0)
{
rotLim->m_maxLimitForce = 1;
rotLim->m_targetVelocity = 0;
}
else
{
if (pos > 0)
{

 rotLim->m_targetVelocity += .05f;
}
if (pos < 0)
{
rotLim->m_targetVelocity += -.05f;
}
}

}
btRigidBody* tank::localCreateRigidBody(btScalar mass,const btTransform& startTransform,btCollisionShape* shape,RIGIDS type)
{

bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);


	MyMotionState* myMotionState = new MyMotionState(startTransform,this->TankNode[type]);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	rbInfo.m_additionalDamping = true;
	btRigidBody* body = new btRigidBody(rbInfo);


   return body;



}
btRigidBody* tank::localCreateRigidWheel(btScalar mass,const btTransform& startTransform,btCollisionShape* shape,scene::ISceneNode* node)
{

bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	MyMotionState* myMotionState = new MyMotionState(startTransform,node);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	rbInfo.m_additionalDamping = true;
	rbInfo.m_friction = 5000.f;
	rbInfo.m_rollingFriction = 20000000.f;
	btRigidBody* body = new btRigidBody(rbInfo);

   return body;



}
void tank::updateCamera(){
vector3df eulerRotationHull;
vector3df eulerRotationTurret;
eulerRotationHull = this->TankNode[RIGIDS_HULL]->getRotation();
eulerRotationTurret = this->TankNode[RIGIDS_DOME]->getRotation();
//this->DomeCamera->setTarget(this->CamHelpT->getAbsolutePosition());
vector3df camUpH = eulerRotationHull.rotationToDirection(vector3df(0,1,0));
vector3df camTH = eulerRotationHull.rotationToDirection(vector3df(1,0,0));
camUpH.normalize();
camTH.normalize();

vector3df camUpD = eulerRotationTurret.rotationToDirection(vector3df(1,0,0));
vector3df camTD = eulerRotationTurret.rotationToDirection(vector3df(0,1,0));

camUpD.normalize();
camTD.normalize();


this->DomeCamera->setUpVector(camUpD);
this->DomeCamera->setTarget(this->DomeCamera->getAbsolutePosition()+camTD);

this->DriverCamera->setUpVector(camUpH);
this->DriverCamera->setTarget(this->DriverCamera->getAbsolutePosition()+camTH);

//std::cout<<DriverCamera->getTarget().X<< " "<<DriverCamera->getTarget().Y<<" "<<DriverCamera->getTarget().Z<<"\n";
}
void tank::startupWheels()
{
btRotationalLimitMotor* rotLim;
for(int i = 0; i< WHEELS_COUNT;i++)
{

 rotLim = this->WheelsSprings_L[i]->getRotationalLimitMotor(2);
 rotLim->m_enableMotor=true;
 rotLim->m_targetVelocity = 0;
 rotLim->m_maxLimitForce = 50000000.f;
 rotLim->m_maxMotorForce = 50000000.f;
 rotLim = this->WheelsSprings_R[i]->getRotationalLimitMotor(2);
 rotLim->m_enableMotor=true;
 rotLim->m_targetVelocity = 0;
 rotLim->m_maxLimitForce = 50000000.f;
 rotLim->m_maxMotorForce = 50000000.f;
//this->Body_Rigs[TANK_LEFTRACK]->applyCentralForce(btVector3(LEngineForce,0,0));
}

}
void tank::setLeftTrack(float force)
{

if (force + LEngineForce > maxEngineForce)
{
LEngineForce = maxEngineForce;
}
else
{
LEngineForce += force;
}
for(int i = 0; i< WHEELS_COUNT;i++)
{
 btRotationalLimitMotor* rotLim = this->WheelsSprings_L[i]->getRotationalLimitMotor(2);
 rotLim->m_enableMotor=true;
 rotLim->m_targetVelocity = LEngineForce;
// rotLim->m_maxLimitForce = 50000.f;

//this->Body_Rigs[TANK_LEFTRACK]->applyCentralForce(btVector3(LEngineForce,0,0));
}
//std::cout<<LEngineForce<<"\n";

}
void tank::setRightTrack(float force)
{
if (force + REngineForce > maxEngineForce)
{
 REngineForce = maxEngineForce;
}
else
{
REngineForce += force;
}


for(int i = 0; i< WHEELS_COUNT;i++)
{
 btRotationalLimitMotor* rotLim = this->WheelsSprings_R[i]->getRotationalLimitMotor(2);
 rotLim->m_enableMotor=true;
 rotLim->m_targetVelocity = REngineForce;
//this->Body_Rigs[TANK_LEFTRACK]->applyCentralForce(btVector3(LEngineForce,0,0));
}
//std::cout<<REngineForce<<"\n";


}
void tank::fire(btDiscreteDynamicsWorld *World,irr::scene::ISceneManager *scene)
{
core::vector3df pos; core::vector3df rot;
rot = TankNode[RIGIDS_CANNON]->getAbsoluteTransformation().getRotationDegrees();
pos = TankNode[RIGIDS_CANNON]->getAbsolutePosition();
//std::cout<<"rot "<< rot.X<<"rot "<< rot.Y<<"rot "<< rot.Z<<".\n";
//std::cout<<"pos "<< pos.X<<"pos "<< pos.Y<<"pos "<< pos.Z<<".\n";
rot = rot.rotationToDirection(core::vector3df(0,2,0));
//std::cout<<"rot "<< rot.X<<"rot "<< rot.Y<<"rot "<< rot.Z<<".\n";

pos = pos + rot;
new Shell(World,scene,btVector3(pos.X,pos.Y,pos.Z),btVector3(rot.X,rot.Y,rot.Z),SHELL_TYPE_AP);
//std::cout<<"pos "<< pos.X<<"pos "<< pos.Y<<"pos "<< pos.Z<<".\n";

}

void tank::setPosition(irr::core::vector3df pos)
{
}
void tank::setRotation(irr::core::vector3df rot)
{
}

void tank::setWF()
{
this->TankNode[RIGIDS_HULL]->setMaterialFlag(video::EMF_WIREFRAME,true);
this->TankNode[RIGIDS_DOME]->setMaterialFlag(video::EMF_WIREFRAME,true);
this->TankNode[RIGIDS_CANNON]->setMaterialFlag(video::EMF_WIREFRAME,true);
for (int i = 0;i< WHEELS_COUNT;i++)
{
this->WheelsNode_L[i]->setMaterialFlag(video::EMF_WIREFRAME,true);
this->WheelsNode_R[i]->setMaterialFlag(video::EMF_WIREFRAME,true);
}
}
void tank::setLig()
{
this->TankNode[RIGIDS_HULL]->setMaterialFlag(video::EMF_LIGHTING,false);
this->TankNode[RIGIDS_DOME]->setMaterialFlag(video::EMF_LIGHTING,false);
this->TankNode[RIGIDS_CANNON]->setMaterialFlag(video::EMF_LIGHTING,false);
for (int i = 0;i< WHEELS_COUNT;i++)
{
this->WheelsNode_L[i]->setMaterialFlag(video::EMF_LIGHTING,false);
this->WheelsNode_R[i]->setMaterialFlag(video::EMF_LIGHTING,false);
}
}
