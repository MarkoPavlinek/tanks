#include "irrlicht.h"
#include "btBulletDynamicsCommon.h"




#ifndef MOTIONSTATE_H_INCLUDED
#define MOTIONSTATE_H_INCLUDED
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



#endif // MOTIONSTATE_H_INCLUDED
