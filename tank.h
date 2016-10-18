#include <irrlicht.h>
#include <btBulletDynamicsCommon.h>
#include "iostream"
#include <string>
#include <vector>
#include <LinearMath/btIDebugDraw.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include "enums.h"
#include "funcs.h"
#include "motionstate.h"
#ifndef TANK_H_INCLUDED
#define TANK_H_INCLUDED


class tank
{

public:


tank();
tank(btDiscreteDynamicsWorld *World,irr::scene::ISceneManager *scene,irr::core::vector3df position,btScalar Tmass);
void createPhy(btDiscreteDynamicsWorld *World);
void move_Cannon(float up = 0.0, float left = 0.0);
void rotateTurret(float pos);
btRigidBody* localCreateRigidBody(
                                btScalar mass,
                                const btTransform& startTransform,
                                btCollisionShape* shape,
                                RIGIDS type);
btRigidBody* localCreateRigidWheel(btScalar mass,
                                const btTransform& startTransform,
                                btCollisionShape* shape,
                                irr::scene::ISceneNode* node);
void updateCamera();
void setLeftTrack(float force);
void setRightTrack(float force);
void fire(btDiscreteDynamicsWorld *World,irr::scene::ISceneManager *scene);


irr::scene::ISceneNode* TankNode[RIGIDS_COUNT];
irr::scene::ISceneNode* WheelsNode_R[WHEELS_COUNT];
irr::scene::ISceneNode* WheelsNode_L[WHEELS_COUNT];
irr::scene::ICameraSceneNode* DomeCamera;
irr::scene::ICameraSceneNode* DriverCamera;
//irr::scene::ISceneNode *CamHelp;
//irr::scene::ISceneNode *CamHelpT;
btScalar mass;
btRaycastVehicle *phytank;
btCollisionShape *Body_Shapes[RIGIDS_COUNT];
btRigidBody *Body_Rigs[RIGIDS_COUNT];
btRigidBody *Wheels_Rigs_L[WHEELS_COUNT];
btRigidBody *Wheels_Rigs_R[WHEELS_COUNT];

btGeneric6DofSpringConstraint *Body_Joints[JOINT_COUNT];
btGeneric6DofSpringConstraint *WheelsSprings_R[WHEELS_COUNT];
btGeneric6DofSpringConstraint *WheelsSprings_L[WHEELS_COUNT];
btConeTwistConstraint* Cannon_Joint;

btVector3 WHEELS_positions[6]={
                        btVector3(-2.5,-0.25,1),
                        btVector3(-1.5,-0.65,1),
                        btVector3(-.5,-0.65,1),
                        btVector3(.5,-0.65,1),
                        btVector3(1.5,-0.65,1),
                        btVector3(2.5,-0.25,1)
                        };
float maxEngineForce=10.f;
float LEngineForce = 0.f;
float REngineForce = 0.f;
float maxBrakeForce=500.f;
float BreakForce = 0.f;



};
class Shell
{
public:

float mass;
float Velocity;

SHELL_TYPE shellType;
btRigidBody *Rig;
irr::scene::ISceneNode* Node;

Shell(btDiscreteDynamicsWorld *World, irr::scene::ISceneManager *irrScene,btVector3 pos,btVector3 target,SHELL_TYPE Shell_typ = SHELL_TYPE_AP)
{
this->Node = irrScene-> addSphereSceneNode(.2,16,0,-1,core::vector3df(pos[0],pos[1],pos[2]));

btTransform localtrans;
localtrans.setOrigin(pos);
btCollisionShape *shape =new btSphereShape(0.2);
btVector3 localinertia;
shape->calculateLocalInertia(1,localinertia);
Rig = new btRigidBody(10,new MotionState(localtrans,this->Node),shape,localinertia);

Rig->applyCentralImpulse(target*1000);
shellType = Shell_typ;
World->addRigidBody(Rig);
//Objects.push_back(Rig);
}
};

#endif
