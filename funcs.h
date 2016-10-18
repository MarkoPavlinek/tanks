#include "irrlicht.h"
#include "btBulletCollisionCommon.h"


using namespace irr;
using namespace irr::core;
#ifndef FUNCS_H_INCLUDED
#define FUNCS_H_INCLUDED
auto toBtVector = [ &]( const vector3df & vec,const vector3df & scal ) -> btVector3
    {
        btVector3 bt( vec.X * scal.X, vec.Y * scal.Y, vec.Z * scal.Z );

        return bt;
    };
btTriangleMesh *makeMesh(irr::scene::ISceneNode * node);

#endif // FUNCS_H_INCLUDED
