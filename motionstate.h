#include <irrlicht.h>
#include <btBulletDynamicsCommon.h>
#include "classes.h"
#ifndef MOTIONSTATE_H_INCLUDED
#define MOTIONSTATE_H_INCLUDED
class MyMotionState : public btMotionState
{


public:
 MyMotionState(const btTransform& initalTransformation, irr::scene::ISceneNode* const node) : node(node), initalTransformation(initalTransformation)
  // MotionState(const btTransform& initalTransformation, anchor_object* const obj) : obj(obj), initalTransformation(initalTransformation)
   {

   }

   void getWorldTransform(btTransform& worldTrans) const
   {
      worldTrans = this->initalTransformation;

   }

   void setWorldTransform(const btTransform& worldTrans)
   {
      worldTrans.getOpenGLMatrix(matr.pointer());
     // this->obj->setRotation(matr.getRotationDegrees());
     //  this->obj->setPosition(matr.getTranslation());

      this->node->setRotation(matr.getRotationDegrees());
      this->node->setPosition(matr.getTranslation());

   }

private:

   irr::scene::ISceneNode* const node;
//   anchor_object* obj;

   irr::core::matrix4 matr;

   btTransform initalTransformation;
};

class MotionState2 : public btMotionState
{


public:
 MotionState2(const btTransform& initalTransformation, anchor_object* const node) : node(node), initalTransformation(initalTransformation)
  // MotionState(const btTransform& initalTransformation, anchor_object* const obj) : obj(obj), initalTransformation(initalTransformation)
   {

   }

   void getWorldTransform(btTransform& worldTrans) const
   {
      worldTrans = this->initalTransformation;

   }

   void setWorldTransform(const btTransform& worldTrans)
   {
      worldTrans.getOpenGLMatrix(matr.pointer());
//      this->obj->setRotation(matr.getRotationDegrees());
   //    this->obj->setPosition(matr.getTranslation());

      node->setRotation(matr.getRotationDegrees());
      node->setPosition(matr.getTranslation());

   }

private:

   anchor_object* const node;
//   anchor_object* obj;

   irr::core::matrix4 matr;

   btTransform initalTransformation;
};


#endif // MOTIONSTATE_H_INCLUDED
